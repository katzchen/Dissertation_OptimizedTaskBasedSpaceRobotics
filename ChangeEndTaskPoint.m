function [AllVariables]= ChangeEndTaskPoint(AllVariables,xend,angle)
%Chance AllVariables with the new Armlength in 'X'
%since X can be a number of different cases, this goes through and
%determines which one it is to properly change AllVariables

AllVariables.Task.task_end= xend;
AllVariables.Task.theta_task_end=angle; %of the main payload relative to global;

ChangetoPlanar = 0;
for i = 1:length(AllVariables.Arm)
    if AllVariables.Arm(i).IsPlanar == 1
        ChangetoPlanar = 1;
    end
end
%I have a planar arm so lets not have anything in z

if ChangetoPlanar > 0
    AllVariables.Task.task_end(3)=0; %of the main payload relative to global;
end
