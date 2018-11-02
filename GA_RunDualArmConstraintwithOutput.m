function [x,fval,populationsize] = GA_RunDualArmConstraintwithOutput(allowsinglearmanswers,armnumber,mainiteration)
close all
%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easier
%All Task related variables
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
% 
% JointAnglesStart = [32.4 50.5 263 50.4 151 88.2]*pi/180;
% T = AllVariables.bot(1).fkine(JointAnglesStart)
% NewStartPoint = (T(1:3,4)+[0 1.5 0]')'
% NewStartAngle = [2.0989 3.2847 1.715]%tr2rpy(T,'deg')-AllVariables.Arm(1).theta_grasp_start*180/pi
% JointAnglesEnd = [82.8 217 46.8 57.6 79.2 66.6]*pi/180;
% T = AllVariables.bot(1).fkine(JointAnglesEnd)
% NewEndPoint = (T(1:3,4)+[0 1.5 0]')'
% NewEndAngle = [4.5137 4.8884 0.5093]
% 
% AllVariables= ChangeStartingTaskPoint(AllVariables,NewStartPoint,NewStartAngle); 
% AllVariables= ChangeEndTaskPoint(AllVariables,NewEndPoint,NewEndAngle);

AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot

tic
%x0 = [5 5.8 5 5.8]'; %[7.0000    2.9861    1.7959    0.0152    2.3559    0.4000 7.0000    2.9861    1.7959    0.0152    2.3559    0.4000]';

A = [];
b = [];
Aeq =[];
beq =[];
allowsinglearmanswers

if allowsinglearmanswers> 0
    'only dual arms allowed.'
    NumberofVariables = 4;    
    InitialPopulzationMatrix = [1 1 1 1; 35 35 35 35];
    populationsize = 200
else
    'only 1 arm'
    NumberofVariables = 2;    
    InitialPopulzationMatrix = [1 1; 35 35];
    populationsize = 100;
end
lb = 0.01*ones(NumberofVariables,1);
ub = 20*ones(NumberofVariables,1);


tic
ga_mincon =@(x)GA_dualarmConstraint(x,AllVariables,armnumber,allowsinglearmanswers);
optionsga = optimoptions(@ga,'PopulationSize',populationsize,'InitialPopulationRange',InitialPopulzationMatrix,'Display','iter','FunctionTolerance',10,'Generations',30,...
    'MaxStallGenerations',20,'PlotFcn',{@gaplotbestindiv,@gaplotdistance,@gaplotbestf, @gaplotstopping},'UseParallel', true, 'UseVectorized', false);
[x,fval,exitflag,output]  = ga(ga_mincon,NumberofVariables,A,b,Aeq,beq,lb,ub,[],optionsga);
'DONE WITH GA'
toc
x
fval
exitflag
output

mainiteration
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
FigHandle = FigList;
char = 'GAPlots\Arm'
charArmnumber=num2str(armnumber);
if allowsinglearmanswers==1
    char2 = '\GA_superExample_DualArm';
else
    char2 = '\GA_superExample_SingleArm';
end
char3 = '_'
charmainteration = num2str(mainiteration);
char = [char charArmnumber char2 char3 charmainteration char3];

%FigName   = get(FigHandle, 'Genetic Algorithm');
savefig(FigHandle, char);
