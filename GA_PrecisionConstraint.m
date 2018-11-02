function [c,ceq] = GA_PrecisionConstraint(x,AllVariables,armnumber)
%I want to make sure I get to the correct pose

if length(x)==4
    AllVariables =  ChangeArmLengthMainJoints(x(1:2),AllVariables,1);
    AllVariables =   ChangeArmLengthMainJoints(x(3:4),AllVariables,2);
else
    AllVariables = ChangeArmLength(x,AllVariables,armnumber);
end

if min(x) < 0
    MaxPosePercent = inf;
else
    [maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
    
    clear MaxPosePercent
    for i = 1:length(AllVariables.Arm)
        MaxPosePercent(i) = (maxPoseErr(i)/AllVariables.Arm(i).TotalArmLength*100)-5;
    end
    
end
%MaxPosePercent
c =MaxPosePercent;
ceq = [];