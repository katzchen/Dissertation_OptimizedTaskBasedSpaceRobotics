function GA_RunSingleArm()

AllVariables = LoadAllVariables;
AllVariables = ChangeArmLength(12,AllVariables,1);
AllVariables = ChangeArmLength(12,AllVariables,2);
AllVariables = ChangeArmLength(7.75,AllVariables,1);
AllVariables = ChangeArmLength(9,AllVariables,2);
AllVariables = ChangeJointLimits(AllVariables,1,[pi/2 2*pi; 0 2*pi;0 2*pi]);
[AllVariables]= ChangeStartingTaskPoint(AllVariables,[-3 9 0],[3 9 0],[15*pi/180 0 0]);
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot

AllVariables.Arm(1).PayloadGrip=[-AllVariables.Task.Lpayload(1)/3  -AllVariables.Task.Lpayload(2)/2  0*AllVariables.Task.Lpayload(3)/2];
AllVariables.Arm(2).PayloadGrip=[AllVariables.Task.Lpayload(1)/3  -AllVariables.Task.Lpayload(2)/2  0*AllVariables.Task.Lpayload(3)/2];

AllVariables.Arm(1).theta_grasp_start=[-90*pi/180 0*pi/180 0*pi/180]; %relative to the payload
AllVariables.Arm(2).theta_grasp_start=[-90*pi/180 0*pi/180 0*pi/180]; %relative to the payload
AllVariables.Arm(1).theta_grasp_end=[-90*pi/180 0*pi/180 0*pi/180]; %relative to the payload
AllVariables.Arm(2).theta_grasp_end=[-90*pi/180 0*pi/180 0*pi/180]; %relative to the payload
tic
tic
x0 = [9.6549]'; %[7.0000    2.9861    1.7959    0.0152    2.3559    0.4000 7.0000    2.9861    1.7959    0.0152    2.3559    0.4000]';
lb = 8*ones(length(x0),1);
ub = 15.2*ones(length(x0),1);

A = [];
b = [];
Aeq =[];
beq =[];
armnumber =1;
 AllVariables = ChangeArmLength_single(.01,AllVariables,2);
NumberofVariables = length(x0); %lets look at changin L2
allowsinglearm = 0; % 0 = true, so 0 will allow a single arm to be seen as a result; NOT USED IN SINGLE ARM
%armnumber =1 %either 1 or 2

allowsinglearmanswers = 1; %0 means yes I will take single arm answer
ga_mincon =@(x)GA_singlearm(x,AllVariables,armnumber);

%can only plot the mean of 1 variable
ConstraintFunction = @(x)GA_EEprecisionconstraint(x,AllVariables,armnumber);
InitialPopulzationMatrix = [x0';x0'] %needs to have number of columns = variables, and the rows have to be less than the size of the population which is 5 for less than 5 variables, and 200 otherwise
if length(x0) == 1
      optionsga = optimoptions(@ga,'PopulationSize',20,'InitialPopulationMatrix',x0','MaxGenerations',20,'Display','diagnose','FunctionTolerance',1,...
       'MaxStallGenerations',10,'PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange},'UseParallel', true, 'UseVectorized', false);

  % optionsga = optimoptions(@ga,'InitialPopulationMatrix',x0','Display','diagnose','PlotFcn',{@gaplotscores,@gaplotbestf, @gaplot1drange},'UseParallel', true, 'UseVectorized', false);
else
    optionsga = optimoptions(@ga,'PopulationSize',50,'InitialPopulationMatrix',InitialPopulzationMatrix,'MaxGenerations',20,'Display','diagnose','FunctionTolerance',.1,...
        'MaxStallGenerations',10,'PlotFcn',{@gaplotscores,@gaplotbestf},'UseParallel', true, 'UseVectorized', false);
end

%options = optimoptions(@ga,'PlotFcn',{@gaplotbestf,@gaplotmaxconstr}, ...
%    'Display','iter');
%options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',9000,'StepTolerance',1e-10,'TolCon', 1e-6, 'TolFun', 1e-5,'MaxIterations',1000);
[x,fval,exitflag,output]  = ga(ga_mincon,length(x0),A,b,Aeq,beq,lb,ub,ConstraintFunction,optionsga)

x
fval
exitflag
output
 graphResults(x,AllVariables,armnumber)