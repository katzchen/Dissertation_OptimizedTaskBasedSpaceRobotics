function Ga_RunMultipleTimesDualNOTask(Ntimes)

armnumber = 0
allowsinglearm = 1; %0 means yes, allow single arms
for i = 1:Ntimes
    [x1(:,i),fval1(i),populationsize1(i)] =  GA_RunDualArmConstraintwithOutput(x,AllVariables,armnumber,allowsinglearm,armnumber,i);
end
% armnumber = 2
% for i = 6:Ntimes+6
%     [x2(:,i),fval2(i),populationsize2(i)] = GA_RunDualArmConstraintwithOutput(allowsinglearm,armnumber,i);
% end

x1
fval1
populationsize1
'endcode'