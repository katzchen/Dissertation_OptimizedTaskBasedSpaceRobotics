function [x,fval,AllVariables] = GA_RunDualArmConstraint()
close all, clc, clear all
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


armnumber =1;
allowsinglearmanswers = 0; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%IF YOU WANT TO DO A SINGLE ARM MAK ESURE allowsinglearmanswers = 0, AND
%ARMNUMBER = 1 OR 2
if allowsinglearmanswers> 0
    'only dual arms allowed.'
    NumberofVariables = 4;    
    InitialPopulzationMatrix = [1 1 1 1; 35 35 35 35];
    populationsize = 150;
else
    'only 1 arm'
    'only dual arms allowed.'
    NumberofVariables = 2;    
    InitialPopulzationMatrix = [1 1; 35 35];
    populationsize = 50;
end
lb = 0.01*ones(NumberofVariables,1);
ub = 20*ones(NumberofVariables,1);


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

if allowsinglearmanswers> 0
 [AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,1);
 [AllVariables]= ChangeArmLengthMainJoints(x(3:4),AllVariables,2);
else    
 [AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,armnumber);
end

  graphResults(x,AllVariables,armnumber)