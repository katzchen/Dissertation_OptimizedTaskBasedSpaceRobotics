clc
close all
clear all
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


tic
x0 = ones(1,1);
lb = .1*ones(1,1);
ub = 12*ones(1,1);
A = [];
b = [];
Aeq =[];
beq =[];
ga_mincon =@(x)RunGAonArms(x,AllVariables);
ga_minconstraint =@(x)RunGAonArmsConstraints(x,AllVariables);
optionsga = optimoptions(@ga,'MaxGenerations',100,'Display','diagnose','FunctionTolerance',10,...
    'MaxStallGenerations',50,'PlotFcn',{@gaplotbestf,@gaplotscores},'UseParallel', true, 'UseVectorized', false);
%options = optimoptions(@ga,'PlotFcn',{@gaplotbestf,@gaplotmaxconstr}, ...
%    'Display','iter');
%options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',9000,'StepTolerance',1e-10,'TolCon', 1e-6, 'TolFun', 1e-5,'MaxIterations',1000);
[x,fval,exitflag,output]  = ga(ga_mincon,length(x0),A,b,Aeq,beq,lb,ub,[],optionsga)
[TotalMass, TotalMassMotor, TotalMassLink, MaxTorque,MassFinal_arm]= ChangeArmLength(x,AllVariables)

'Done running the iterator- now onto plotting hte results line 109 manual'

figure
subplot(3,1,1)
hold on
plot(x,(TotalMassLink(1,:)),'r- o')
plot(x,(TotalMassLink(2,:)),'b- o')
plot(x,(TotalMassLink(3,:)),'g-p')
legend('Arm1','Arm2','1+2')
xlabel('ArmLength for Arm2')
ylabel('Link Mass (Kg)')

subplot(3,1,2)
hold on
plot(x,(TotalMassMotor(1,:)),'r- o')
plot(x,(TotalMassMotor(2,:)),'b- o')
plot(x,(TotalMassMotor(3,:)),'g-p')
legend('Arm1','Arm2','1+2')
xlabel('ArmLength for Arm2')
ylabel('MotorMass (Kg)')

subplot(3,1,3)
hold on
plot(x',(TotalMass(:,1)),'r- o')
plot(x',(TotalMass(:,2)),'b- o')
plot(x',(TotalMass(:,3)),'g-p')
legend('Arm1','Arm2','1+2','2 of the same')
xlabel('ArmLength for Arm2')
ylabel('(Total Mass (kg))')

Niter = length(x);
for iter = 1:Niter
    if (iter == 1)
        figure
    end
    
    subplot(Niter,1,iter)
    hold on
    tempMass(:,:,:) = MassFinal_arm(:,:,iter);
    bar(tempMass, 'stacked');%does a figure, holdon
    set(gca,'XTickLabel',{'Arm1','Arm2','Arms 1+2'});
    legend('Arm1','Arm2')
    title('Elbow Right Total Mass with individual arm masses (kg)')
    xlabel('Arm combinations');
    ylabel('Total Mass(kg)')
    %   axis([0 8 0 max(maxAxisMass)])
end
toc
