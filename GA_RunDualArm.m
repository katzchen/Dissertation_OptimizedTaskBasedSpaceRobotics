function [x,fval,AllVariables] = GA_RunDualArm()
close all, clc, clear all
%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easier
%All Task related variables
AllVariables = LoadAllVariables_SRMS_simple;
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
% 
% JointAnglesStart = [32.4 50.5 263 50.4 151 88.2]*pi/180;
% T = AllVariables.bot(1).fkine(JointAnglesStart)
% NewStartPoint = (T(1:3,4)+[0 1.5 0]')'
% NewStartAngle =[2.0989 3.2847 1.715]%tr2rpy(T,'deg')-AllVariables.Arm(1).theta_grasp_start*180/pi
% JointAnglesEnd = [82.8 217 46.8 57.6 79.2 66.6]*pi/180;
% T = AllVariables.bot(1).fkine(JointAnglesEnd)
% NewEndPoint = (T(1:3,4)+[0 1.5 0]')'
% NewEndAngle = [4.5137 4.8884 0.5093]
% 
% AllVariables = ChangeJointLimits(AllVariables,1,[-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi]);
% AllVariables= ChangeStartingTaskPoint(AllVariables,NewStartPoint,NewStartAngle); 
% AllVariables= ChangeEndTaskPoint(AllVariables,NewEndPoint,NewEndAngle);

AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot

tic
x0 = [9.6549 9*rand(1)]'; %[7.0000    2.9861    1.7959    0.0152    2.3559    0.4000 7.0000    2.9861    1.7959    0.0152    2.3559    0.4000]';
lb = 7*ones(length(x0),1);
ub = 20*ones(length(x0),1);

A = [];
b = [];
Aeq =[];
beq =[];
armnumber =2;
NumberofVariables = length(x0); %lets look at changin L2
allowsinglearm = 1; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%armnumber =1 %either 1 or 2

allowsinglearmanswers = 1; %0 means yes I will take single arm answer
ga_mincon =@(x)GA_dualarm(x,AllVariables,armnumber,allowsinglearmanswers);
%ensure the minimum error is within 5% of each arm lengthalso 
ConstraintFunction = @(x)GA_PrecisionConstraint(x,AllVariables,armnumber);

InitialPopulzationMatrix = [x0';x0'] %needs to have number of columns = variables, and the rows have to be less than the size of the population which is 5 for less than 5 variables, and 200 otherwise
if length(x0) == 1
      optionsga = optimoptions(@ga,'PopulationSize',20,'InitialPopulationMatrix',x0','MaxGenerations',20,'Display','diagnose','FunctionTolerance',1,...
       'MaxStallGenerations',10,'PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange, @gaplotmaxconstr},'UseParallel', true, 'UseVectorized', false);

  % optionsga = optimoptions(@ga,'InitialPopulationMatrix',x0','Display','diagnose','PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange},'UseParallel', true, 'UseVectorized', false);
else  
    %sitll only have 2 variables to a populationsize of 10 should be okay
    optionsga = optimoptions(@ga,'PopulationSize',10,'InitialPopulationMatrix',InitialPopulzationMatrix,'MaxGenerations',20,'Display','iter','FunctionTolerance',.1,...
        'MaxStallGenerations',10,'PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange, @gaplotmaxconstr},'UseParallel', true, 'UseVectorized', false);
end

%options = optimoptions(@ga,'PlotFcn',{@gaplotbestf,@gaplotmaxconstr}, ...
%    'Display','iter');
%options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',9000,'StepTolerance',1e-10,'TolCon', 1e-6, 'TolFun', 1e-5,'MaxIterations',1000);
[x,fval,exitflag,output]  = ga(ga_mincon,length(x0),A,b,Aeq,beq,lb,ub,ConstraintFunction,optionsga)

x
fval
exitflag
output
AllVariables = ChangeArmLength(x,AllVariables,0);

graphResults(x,AllVariables,armnumber)