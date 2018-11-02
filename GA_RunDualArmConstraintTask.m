function [x,fval,AllVariables] = GA_RunDualArmConstraintTask(allowsinglearmanswers, armnumber,mainiteration)
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
x0 = [5 5.8 5 5.8]'; %[7.0000    2.9861    1.7959    0.0152    2.3559    0.4000 7.0000    2.9861    1.7959    0.0152    2.3559    0.4000]';
lb = 0.01*ones(length(x0),1);
ub = 20*ones(length(x0),1);

A = [];
b = [];
Aeq =[];
beq =[];


%armnumber =0;
%allowsinglearmanswers = 1; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%IF YOU WANT TO DO A SINGLE ARM MAK ESURE allowsinglearmanswers = 0, AND
%ARMNUMBER = 1 OR 2
if allowsinglearmanswers> 0
    'only dual arms allowed.'
    NumberofVariables = 6;
    InitialPopulzationMatrix = [1 1 1 1 -5 -5; 35 35 35 35 5 5];
    %populationsize = 250;
    lb = [0.01*ones(1,4) -10 -10]
    ub = [20*ones(1,4) 10 10]
    
    %InitialPopulzationMatrix = [1 1 1 1; 35 35 35 35];
    populationsize =300
    %lb = [0.01*ones(1,4)]
    %ub = [20*ones(1,4)]
    
else
    'only 1 arm'
    'only dual arms allowed.'
    %NumberofVariables = 3;
    %InitialPopulzationMatrix = [1 1 -5; 35 35 5];
    NumberofVariables = 1;
    InitialPopulzationMatrix = [-30; 30];
    populationsize = 100;
    lb = [-30]
    ub = [30]
    %lb = [0.01*ones(1,2) -10 ]
    %ub = [20*ones(1,2) 10]
end


tic
ga_mincon =@(x)GA_dualarmConstraint(x,AllVariables,armnumber,allowsinglearmanswers);
optionsga = optimoptions(@ga,'PopulationSize',populationsize,'InitialPopulationRange',InitialPopulzationMatrix,'Display','iter','FunctionTolerance',10,...
    'MaxStallGenerations',20,'PlotFcn',{@gaplotdistance,@gaplotrange, @gaplotscores,@gaplotbestf, @gaplotstopping , @gaplotmaxconstr,@gaplotselection,@gaplotbestindiv},'UseParallel', true, 'UseVectorized', false);
[x,fval,exitflag,output]  = ga(ga_mincon,NumberofVariables,A,b,Aeq,beq,lb,ub,[],optionsga);




'DONE WITH GA'
toc
x
fval
exitflag
output
populationsize
InitialPopulzationMatrix

mainiteration
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
FigHandle = FigList;
char = 'GAPlots\Arm'
charArmnumber=num2str(armnumber);
if allowsinglearmanswers==1
    char2 = '\GA_superExample_Base_DualArm';
else
    char2 = '\GA_superExample_3links_SingleArm';
end
char3 = '_'
charmainteration = num2str(mainiteration);
char = [char charArmnumber char2 char3 charmainteration];

%FigName   = get(FigHandle, 'Genetic Algorithm');
savefig(FigHandle, char);

% 
% 
% if allowsinglearmanswers> 0
%  [AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,1);
%  [AllVariables]= ChangeArmLengthMainJoints(x(3:4),AllVariables,2);
% else    
%  [AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,armnumber);
% end
% 
%   graphResults(x,AllVariables,armnumber)