function [x,fval,AllVariables] = GA_RunDualArmConstraint_Lenny()
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
lb = .01*ones(length(x0),1);
ub = 20*ones(length(x0),1);

A = [];
b = [];
Aeq =[];
beq =[];
armnumber =2;
NumberofVariables = length(x0); %lets look at changin L2
allowsinglearmanswers = 0; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%armnumber =1 %either 1 or 2
ga_mincon =@(x)GA_dualarmConstraint(x,AllVariables,armnumber,allowsinglearmanswers);
ConstraintFunction = @(x)GA_PrecisionConstraint(x,AllVariables,armnumber); %THIS IS NONLINEAR
%PUTTING THE CONSTRAINT INTO THE GA CAUSE MY PARENTS WERENOT CHANGIN
%ANYTHIGN

InitialPopulzationMatrix = [1 1 1 1;35 35 35 35]%;20*rand(50,4)] %needs to have number of columns = variables, and the rows have to be less than the size of the population which is 5 for less than 5 variables, and 200 otherwise
tic
%TO TRY NEXT
%optionsga = optimoptions(@ga,'PopulationSize',50,'InitialPopulationMatrix',InitialPopulzationMatrix,'MaxGenerations',20,'Display','iter',...
%    'MaxStallGenerations',20,'PlotFcn',{@gaplotdistance,@gaplotrange, @gaplotscores,@gaplotselection,@gaplotbestindiv},'UseParallel', true, 'UseVectorized', false,...
%    'MutationFcn',@mutationadaptfeasible,'Generations',50);%{@mutationgaussian, scale, shrink});%@gaplotbestf, @gaplotstopping , @gaplotmaxconstr,


optionsga = optimoptions(@ga,'PopulationSize',150,'InitialPopulationMatrix',InitialPopulzationMatrix,'Display','iter','FunctionTolerance',10,...
    'MaxStallGenerations',20,'PlotFcn',{@gaplotbestf, @gaplotdistance,@gaplotrange, @gaplotscores,@gaplotselection,@gaplotbestindiv},'UseParallel', true, 'UseVectorized', false);%{@mutationgaussian, scale, shrink});%@gaplotbestf, @gaplotstopping , @gaplotmaxconstr,
[x,fval,exitflag,output]  = ga(ga_mincon,length(x0),A,b,Aeq,beq,lb,ub,[],optionsga)
toc
'DONE WITH GA'
x
fval
exitflag
output

 [AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,1)
 [AllVariables]= ChangeArmLengthMainJoints(x(3:4),AllVariables,2)
  graphResults(x,AllVariables,armnumber)