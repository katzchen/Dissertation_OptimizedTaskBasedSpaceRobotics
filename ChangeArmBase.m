function [AllVariables]= ChangeArmBase(xyzbase,AllVariables,armnumber)
%Chance AllVariables with the new Armlength in 'X'
%since X can be a number of different cases, this goes through and
%determines which one it is to properly change AllVariables

if armnumber ==0
    'ERROR INVALID ARMNUBMER FOR CHANGING ARM BASE'
    return;
end

AllVariables.Arm(armnumber).xyz_base =  xyzbase; %base of arm 1
if AllVariables.Arm(armnumber).IsPlanar > 0  %1= yes it is planar
    AllVariables.Arm(armnumber).xyz_base(3) =0; %make sure the z component is 0
end

i= armnumber;
AllVariables.bot(i)= GenerateBot_singleArm(AllVariables.Arm(i).ArmLength,AllVariables.Arm(i).IsPlanar,AllVariables.Arm(i).armtype,AllVariables.Arm(i).Armoffset,0);
AllVariables.bot(i).base = [eye(3,3) AllVariables.Arm(i).xyz_base';0 0 0 1];
AllVariables.bot(i).qlim =AllVariables.Arm(i).JointLimits;


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