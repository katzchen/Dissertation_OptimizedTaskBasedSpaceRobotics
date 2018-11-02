function [x,fval,AllVariables] = GA_RunSingleArm(armnumber)
isleft = 1;

path(path ,'C:/Users/Kate/Documents/MATLAB/RobotToolBox/rvctools')
path(path ,'C:/Users/Kate McBryan/Documents/MATLAB/RobotToolBox/rvctools')
startup_rvc

%%
%Task Description
totaldistancetotravel =6;
x_task_start = -totaldistancetotravel/2;
x_task_end = -x_task_start;

%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easier
%All Task related variables
AllVariables.Task.MinThickness= 0.0254*(1/16); %.0015875; %m, min thickness between outer and inner diaemters, 1/16" = 1.58mm;
AllVariables.Task.maxVelocity_xyz= .06;%m/s %SRMS
AllVariables.Task.maxVelocity_rpy= .06;%m/s %SRMS
AllVariables.Task.Mpayload=14515; %kg
AllVariables.Task.Lpayload=[3 3 3]; %length in m
AllVariables.Task.DesiredTime= 120; %seconds
AllVariables.Task.task_start= [x_task_start AllVariables.Task.Lpayload(2) 0 ]; %of the main payload relative to global;
AllVariables.Task.task_end= [x_task_end AllVariables.Task.Lpayload(2) 0 ];
AllVariables.Task.theta_task_start=[0*pi/180 0*pi/180 0*pi/180]; %of the main payload relative to global;
AllVariables.Task.theta_task_end=[0*pi/180 0*pi/180 0*pi/180];
AllVariables.Task.npoints=8; %HALF the number of points used in trajecotries
AllVariables.Task.DecelerationTime_xyz = 1;% must stop in 1 secdond
AllVariables.Task.DecelerationTime_rpy = 1;% must stop in 1 secdond
AllVariables.Task.maxAcceleration_xyz = AllVariables.Task.maxVelocity_xyz/AllVariables.Task.DecelerationTime_xyz;
AllVariables.Task.maxAcceleration_rpy = AllVariables.Task.maxVelocity_rpy/AllVariables.Task.DecelerationTime_rpy;
AllVariables.Task.ForceAtStart = [0 0 0 0 0 0]'; %these are glbal coordinates
AllVariables.Task.ForceAtEnd = [0 0 0 0 0 0]';
AllVariables.Task.ForceContinuous = [0 0 0 0 0 0]'; %continous torque while moving in addition to the moving
AllVariables.Task.Velocity_angularlimit = 2*pi/10; %1 full circle in 10 seconds
%Other
AllVariables.plottask=0; %0 = no plot
AllVariables.ploton=0;%0 = no plot

%All Arm related variables
AllVariables.Arm(1).xyz_base =  [0 0 0]; %base of arm 1
AllVariables.Arm(1).ArmLength= [7 .1  7 .1 .6 .4];
AllVariables.Arm(1).PayloadGrip=[-AllVariables.Task.Lpayload(1)/2  -AllVariables.Task.Lpayload(2)/2  0*AllVariables.Task.Lpayload(3)/2];
AllVariables.Arm(1).theta_grasp_start=[90*pi/180 0*pi/180 0*pi/180]; %relative to the payload
AllVariables.Arm(1).theta_grasp_end=AllVariables.Arm(1).theta_grasp_start;
AllVariables.Arm(1).IsPlanar = 0; %1 = yes
AllVariables.Arm(1).MotorTorqueSensitivity = .07*ones(length(AllVariables.Arm(1).ArmLength),1);
AllVariables.Arm(1).gravity =  [0 0 0]; %gravity vector
AllVariables.Arm(1).gearratio = 160;
AllVariables.Arm(1).MaxDeflectionPerLength = 1/50;
AllVariables.Arm(1).material.poisson = .2;
AllVariables.Arm(1).material.E = 160*10^9;%Pa
AllVariables.Arm(1).material.BendingStressMax = (276*10^6)/2; %FOS = 2, based on yield (tensile)
AllVariables.Arm(1).material.Density = 1700; %kg/m^3
AllVariables.Arm(1).TotalArmLength = 0; %is assigned in ChangeArmLength
AllVariables.Arm(1).material.MinThickness= 0.0254*(1/16); %.0015875; %m, min thickness between outer and inner diaemters, 1/16" = 1.58mm;
AllVariables.Arm(1).JointLimits = [0 2*pi;-2*pi 2*pi; -2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi];
AllVariables.Arm(1).name ='arm1';
AllVariables.Arm(1).color =[1 1 0];
AllVariables.Arm(1).EEMaxMovementBy1Degree = .1; %m
AllVariables.Arm(1).EEJoints = 3; %how many joints (from the end down) will have this limit


AllVariables.Arm(2).xyz_base =  [0 0 0]; %base of arm 2
AllVariables.Arm(2).PayloadGrip=[-AllVariables.Task.Lpayload(1)/2  -AllVariables.Task.Lpayload(2)/2  0*AllVariables.Task.Lpayload(3)/2];
AllVariables.Arm(2).theta_grasp_start=[90*pi/180 0*pi/180 0*pi/180];
AllVariables.Arm(2).theta_grasp_end=AllVariables.Arm(2).theta_grasp_start;
AllVariables.Arm(2).IsPlanar = 0; %1 = yes
AllVariables.Arm(2).MotorTorqueSensitivity = .07*ones(6,1);
AllVariables.Arm(2).gravity =  [0 0 0]; %gravity vector
AllVariables.Arm(2).TotalArmLength = 0;%is assigned in ChangeArmLength
AllVariables.Arm(2).gearratio = 160;
AllVariables.Arm(2).MaxDeflectionPerLength = 1/50;
AllVariables.Arm(2).material = AllVariables.Arm(1).material;
AllVariables.Arm(2).TotalArmLength = 0; %is assigned in ChangeArmLength
AllVariables.Arm(2).JointLimits = [0 2*pi;-2*pi 2*pi; -2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi];
AllVariables.Arm(2).name ='arm2';
AllVariables.Arm(2).color =[1 0 0];
AllVariables.Arm(2).EEMaxMovementBy1Degree = .025; %m
AllVariables.Arm(2).EEJoints = 3; %how many joints (from the end down) will have this limit


tic
x0 = [7.0000    2.9861    1.7959    0.0152    2.3559    0.4000]';
lb = .01*ones(length(x0),1);
ub = 10*ones(length(x0),1);
A = [];
b = [];
Aeq =[];
beq =[];
NumberofVariables = 1; %lets look at changin L2
allowsinglearm = 1; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%armnumber =2 %either 1 or 2
ga_mincon =@(x)GA_singlearm(x,AllVariables,armnumber);

%can only plot the mean of 1 variable
ConstraintFunction = @(x)GA_EEprecisionconstraint(x,AllVariables,armnumber);
InitialPopulzationMatrix = [x0';x0';x0';x0']; %needs to have number of columns = variables, and the rows have to be less than the size of the population which is 5 for less than 5 variables, and 200 otherwise
if length(x0) == 1
    optionsga = optimoptions(@ga,'PopulationSize',50,'InitialPopulationMatrix',x0','MaxGenerations',1,'Display','diagnose','FunctionTolerance',.1,...
        'MaxStallGenerations',1,'PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange},'UseParallel', true, 'UseVectorized', false);
else
    optionsga = optimoptions(@ga,'PopulationSize',50,'InitialPopulationMatrix',InitialPopulzationMatrix,'MaxGenerations',20,'Display','diagnose','FunctionTolerance',.1,...
        'MaxStallGenerations',10,'PlotFcn',{@gaplotscores,@gaplotbestf},'UseParallel', true, 'UseVectorized', false);
end

%options = optimoptions(@ga,'PlotFcn',{@gaplotbestf,@gaplotmaxconstr}, ...
%    'Display','iter');
%options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',9000,'StepTolerance',1e-10,'TolCon', 1e-6, 'TolFun', 1e-5,'MaxIterations',1000);
[x,fval,exitflag,output]  = ga(ga_mincon,length(x0),A,b,Aeq,beq,lb,ub,ConstraintFunction,optionsga)

 graphResults(x,AllVariables,armnumber)