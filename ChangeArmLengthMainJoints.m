function [AllVariables]= ChangeArmLengthMainJoints(x,AllVariables,armnumber)
format compact
%chagne the main 2 boom lengths

Link1Number =2;
Link2Number =3;

if length(x) > 2    
    Link1Number =1;
    Link2Number =2;
    Link3Number =3;
    
  %  'changing 3 links'
end

if armnumber == 0
    'Error i ChangeArmLenghMainJoints. Need an armnumber'
else
    %want to c
    AllVariables.Arm(armnumber).ArmLength(Link1Number) = x(1);
    AllVariables.Arm(armnumber).ArmLength(Link2Number) = x(2);
    if length(x) > 2
        AllVariables.Arm(armnumber).ArmLength(Link3Number) = x(3);
    end
    
    
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
end

clear AllVariables.bot(armnumber);
teach = 0;
AllVariables.bot(armnumber)= GenerateBot_singleArm(AllVariables.Arm(armnumber).ArmLength,AllVariables.Arm(armnumber).IsPlanar, AllVariables.Arm(armnumber).armtype, AllVariables.Arm(armnumber).Armoffset,teach);
AllVariables.bot(armnumber).base = [eye(3,3) AllVariables.Arm(armnumber).xyz_base';0 0 0 1];
AllVariables.bot(armnumber).qlim =AllVariables.Arm(armnumber).JointLimits;

%ArmLength=  [x AllVariables.Arm(armnumber).TotalArmLength]