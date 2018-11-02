function [AllVariables]= ChangeArmLength(x,AllVariables,armnumber)
format compact
%Chance AllVariables with the new Armlength in 'X'
%since X can be a number of different cases, this goes through and
%determines which one it is to properly change AllVariables

if armnumber ==0
    armnumber = 1;
end

if armnumber == 1
    otherarmnumber = 2;
else
    otherarmnumber = 1;
end

if length(AllVariables.Arm(otherarmnumber).ArmLength) < 2    
    if AllVariables.Arm(otherarmnumber).IsPlanar > 0  %1= yes it is planar
        AllVariables.Arm(otherarmnumber).ArmLength =[1 1 1];
    else
        AllVariables.Arm(otherarmnumber).ArmLength =[1 1 1 1 1 1] ;
    end
end

if length(AllVariables.Arm(armnumber).ArmLength) < 2 && length(x)<6
    if AllVariables.Arm(armnumber).IsPlanar > 0  %1= yes it is planar
        AllVariables.Arm(armnumber).ArmLength =[1 1 1];
    else
        AllVariables.Arm(armnumber).ArmLength=[1 1 1 1 1 1];
    end
end

%Going to build in cases so I can run this without having to keep changing
%things
if length(x) == 1
    %'Case: Change The TotalLength for ArmNumber'
    
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
    
    AllVariables.Arm(armnumber).ArmLength = x*AllVariables.Arm(armnumber).ArmLength/AllVariables.Arm(armnumber).TotalArmLength;
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
    
    %Need to sum up the other arm too    
        L1length = 0;
    for i = 1:length(AllVariables.Arm(otherarmnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(otherarmnumber).ArmLength(i);
    end
    AllVariables.Arm(otherarmnumber).TotalArmLength = L1length;    
    
elseif length(x)==2
 %   'ChangeArmLength: Change the total length for Arm1 and Arm2'
  %normalize arm1
  for j = 1:2
      L1length = 0;
      for i = 1:length(AllVariables.Arm(j).ArmLength)
          L1length = L1length+AllVariables.Arm(j).ArmLength(i);
      end
      AllVariables.Arm(j).TotalArmLength = L1length;
      
      AllVariables.Arm(j).ArmLength = x(j)*AllVariables.Arm(j).ArmLength/AllVariables.Arm(j).TotalArmLength;
      L1length = 0;
      for i = 1:length(AllVariables.Arm(j).ArmLength)
          L1length = L1length+AllVariables.Arm(j).ArmLength(i);
      end
      AllVariables.Arm(j).TotalArmLength = L1length;
  end
    
    
elseif (length(x) ==6 && AllVariables.Arm(armnumber).IsPlanar == 0)  || (length(x)==3 && AllVariables.Arm(armnumber).IsPlanar == 1) %yes it is planar
    'Case: Change All the variables for Arm Number'
        
    AllVariables.Arm(armnumber).TotalArmLength = x;
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
    
    %still need to sum um the other arm
    L1length = 0;
    for i = 1:length(AllVariables.Arm(otherarmnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(otherarmnumber).ArmLength(i);
    end
    AllVariables.Arm(otherarmnumber).TotalArmLength = L1length;

    
else
    'Case: Change All the variables for BOTH ARMS'
    
    if AllVariables.Arm(armnumber).IsPlanar == 1
        DOFperarm1 = 3;
    else
        DOFperarm1 = 6;
    end
    
    if AllVariables.Arm(otherarmnumber).IsPlanar == 1
        DOFperarm2 = 3;
    else
        DOFperarm2 = 6;
    end

    AllVariables.Arm(1).ArmLength =  x(1:DOFperarm1);
    AllVariables.Arm(2).ArmLength = x(1+DOFperarm1:DOFperarm1+DOFperarm2);
    
     L1length = 0;
    for i = 1:length(AllVariables.Arm(1).ArmLength)
        L1length = L1length+AllVariables.Arm(1).ArmLength(i);
    end
    AllVariables.Arm(1).TotalArmLength = L1length;
    
    L1length = 0;
    for i = 1:length(AllVariables.Arm(2).ArmLength)
        L1length = L1length+AllVariables.Arm(2).ArmLength(i);
    end
    AllVariables.Arm(2).TotalArmLength = L1length;
end

clear AllVariables.bot;
for i = 1:length(AllVariables.Arm)
    teach = 0;
    AllVariables.bot(i)= GenerateBot_singleArm(AllVariables.Arm(i).ArmLength,AllVariables.Arm(i).IsPlanar, AllVariables.Arm(i).armtype, AllVariables.Arm(i).Armoffset,teach);
    AllVariables.bot(i).base = [eye(3,3) AllVariables.Arm(i).xyz_base';0 0 0 1];
    AllVariables.bot(i).qlim =AllVariables.Arm(i).JointLimits;
end

% if (length(AllVariables.Arm(otherarmnumber).ArmLength)>6  && AllVariables.Arm(otherarmnumber).IsPlanar==0)
%     AllVariables.Arm(otherarmnumber).ArmLength=0.01*ones(1,6);
%     AllVariables.Arm(otherarmnumber).TotalArmLength = .01*6;
% end
% if (length(AllVariables.Arm(otherarmnumber).ArmLength)>3  && AllVariables.Arm(otherarmnumber).IsPlanar==1)
%     AllVariables.Arm(otherarmnumber).ArmLength=0.01*ones(1,3);
%     AllVariables.Arm(otherarmnumber).TotalArmLength = .01*3;
% end

% if (length(AllVariables.Arm(otherarmnumber).ArmLength)< 3)
%     if AllVariables.Arm(otherarmnumber).IsPlanar==1        
%         AllVariables.Arm(otherarmnumber).ArmLength=0.01*ones(1,3);
%         AllVariables.Arm(otherarmnumber).TotalArmLength = .01*3;
%     else
%         AllVariables.Arm(otherarmnumber).ArmLength=0.01*ones(1,6);
%         AllVariables.Arm(otherarmnumber).TotalArmLength = .01*6;
%     end
% end