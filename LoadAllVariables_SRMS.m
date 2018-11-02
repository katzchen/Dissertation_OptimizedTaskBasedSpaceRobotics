function AllVariables = LoadAllVariables_SRMS()

%%
'Loading SRMS'
path(path ,'C:/Users/Kate/Documents/MATLAB/RobotToolBox/rvctools')
path(path ,'C:/Users/Kate McBryan/Documents/MATLAB/RobotToolBox/rvctools')
startup_rvc
format compact

%Task Description
totaldistancetotravel =0;
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
AllVariables.Depth.currentLevel = 0;
AllVariables.Depth.maxLevel = 3;

AllVariables.Task.DesiredTime= 1; %seconds, if complex this is per segment
AllVariables.Task.ComplexMotion.Code = 0;%1 for circle, 2 for square, 3= rotating square
if AllVariables.Task.ComplexMotion.Code == 3
    AllVariables.Task.ComplexMotion.Nsegments = 8; %number of segments
else
    AllVariables.Task.ComplexMotion.Nsegments = 4; %number of segments
end
AllVariables.Task.ComplexMotion.XDistance = 4; %either the radii of the circle, or the length of the planar square
AllVariables.Task.ComplexMotion.YDistance = 4; %either the radii of the circle, or the length of the planar square
AllVariables.Task.task_start= [ 8.1975   6   8.3128]; %of the main payload relative to global;
AllVariables.Task.task_end= [4.7558    1.9424    0.9391];
AllVariables.Task.theta_task_start=[2.0989 3.2847 1.715]; %of the main payload relative to global;
AllVariables.Task.theta_task_end=[4.5137 4.8884 0.5093];
AllVariables.Task.npoints=2; %HALF the number of points used in trajecotries

AllVariables.Task.DecelerationTime_xyz = 1;% must stop in 1 secdond
AllVariables.Task.DecelerationTime_rpy = 1;% must stop in 1 secdond
AllVariables.Task.maxAcceleration_xyz = AllVariables.Task.maxVelocity_xyz/AllVariables.Task.DecelerationTime_xyz;
AllVariables.Task.maxAcceleration_rpy = AllVariables.Task.maxVelocity_rpy/AllVariables.Task.DecelerationTime_rpy;
AllVariables.Task.ForceAtStart = [0 0 0 0 0 0]'; %these are glbal coordinates
AllVariables.Task.ForceAtEnd = [0 0 0 0 0 0]';
AllVariables.Task.ForceContinuous = [0 0 0 0 0 0]'; %continous torque while moving in addition to the moving in GLOBAL
AllVariables.Task.Velocity_angularlimit = 2*pi/10; %1 full circle in 10 seconds
%Other
AllVariables.Task.plot=0; %0 = no plot
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(1).plotmovie=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Arm(2).plotmovie=0;%0 = no plot

%All Arm related variables
AllVariables.Arm(1).xyz_base =  [0 0 0]; %base of arm 1
AllVariables.Arm(1).ArmLength= [1.8 5 5.8 1.6 .5 .5];% .1 .6 .4];0
AllVariables.Arm(1).Armoffset= [0 0 0 0 0 0];% .1 .6 .4];
%AllVariables.Arm(1).ArmLength= [.1 5 5.8 1.17 0 0];% .1 .6 .4];0
%AllVariables.Arm(1).Armoffset= [1.17 0 0 0 0 1.17];% .1 .6 .4];
AllVariables.Arm(1).armtype=  2; %armtype 1 == normal, %armtype 2= SRMS
AllVariables.Arm(1).PayloadGrip=[-AllVariables.Task.Lpayload(1)/4  -0*AllVariables.Task.Lpayload(2)/2 -AllVariables.Task.Lpayload(3)/2];
AllVariables.Arm(1).theta_grasp_start=[-0*pi/180 0*pi/180 0*pi/180]; %relative to the payload
AllVariables.Arm(1).theta_grasp_end=AllVariables.Arm(1).theta_grasp_start;
AllVariables.Arm(1).IsPlanar = 0; %1 = yes
AllVariables.Arm(1).MotorTorqueSensitivity = .07*ones(length(AllVariables.Arm(1).ArmLength),1);
AllVariables.Arm(1).gravity =  [0 0 0]; %gravity vector
AllVariables.Arm(1).gearratio = 160;
AllVariables.Arm(1).MaxDeflectionPerLength = 1/100;
AllVariables.Arm(1).material.poisson = .2;
AllVariables.Arm(1).material.E = 160*10^9;%Pa
AllVariables.Arm(1).material.BendingStressMax = (276*10^6)/2; %FOS = 2, based on yield (tensile)
AllVariables.Arm(1).material.Density = 1700; %kg/m^3
AllVariables.Arm(1).TotalArmLength = 15.2; %is assigned in ChangeArmLength
AllVariables.Arm(1).material.MinThickness= 0.0254*(1/16)/2; %.0015875; %m, min thickness between outer and inner diaemters, 1/16" = 1.58mm;
AllVariables.Arm(1).JointLimits = [-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi];% -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi];
AllVariables.Arm(1).name ='arm1';
AllVariables.Arm(1).linkcolor =[0 .5 0];
AllVariables.Arm(1).EEMaxMovementBy1Degree = .1; %m
AllVariables.Arm(1).EEJoints = 3; %how many joints (from the end down) will have this limit
AllVariables.Arm(1).NumberOfStillViews= 2; %used for plotting purpoes, how many still views per row
AllVariables.Arm(1).TrajGenAngle=[32.4 50.5 263 50.4 151 88.2]*pi/180; %gives an initial guess for the trajectory generation - mostly used for more complex segments

AllVariables.Arm(2).xyz_base =  [10 0 0]; %base of arm 2
AllVariables.Arm(2).ArmLength= AllVariables.Arm(1).ArmLength
AllVariables.Arm(2).Armoffset= AllVariables.Arm(1).Armoffset;% .1 .6 .4];
AllVariables.Arm(2).PayloadGrip=[AllVariables.Task.Lpayload(1)/4  -0*AllVariables.Task.Lpayload(2)/2 AllVariables.Task.Lpayload(3)/2];
AllVariables.Arm(2).theta_grasp_start=[-180*pi/180 -0*pi/180 0*pi/180];
AllVariables.Arm(2).theta_grasp_end=AllVariables.Arm(2).theta_grasp_start;
AllVariables.Arm(2).IsPlanar = 0; %1 = yes
AllVariables.Arm(2).MotorTorqueSensitivity = .07*ones(length(AllVariables.Arm(2).ArmLength),1);
AllVariables.Arm(2).gravity =  [0 0 0]; %gravity vector
AllVariables.Arm(2).gearratio = 160;
AllVariables.Arm(2).MaxDeflectionPerLength = AllVariables.Arm(1).MaxDeflectionPerLength;
AllVariables.Arm(2).material = AllVariables.Arm(1).material;
AllVariables.Arm(2).TotalArmLength = 15.2; %is assigned in ChangeArmLength
AllVariables.Arm(2).JointLimits = [-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi; -2*pi 2*pi;-2*pi 2*pi];% -2*pi 2*pi;-2*pi 2*pi;-2*pi 2*pi];
AllVariables.Arm(2).name ='arm2';
AllVariables.Arm(2).linkcolor =[.5 0 .5];
AllVariables.Arm(2).EEMaxMovementBy1Degree = .1; %m
AllVariables.Arm(2).EEJoints = 3; %how many joints (from the end down) will have this limit
AllVariables.Arm(2).NumberOfStillViews= AllVariables.Arm(1).NumberOfStillViews;
%AllVariables.Arm(2).TrajGenAngle=[0 0 0]; %gives an initial guess for the trajectory generation - mostly used for more complex segments
AllVariables.Arm(2).TrajGenAngle=[1.6780    0.1969   -0.5575    1.6761    2.9749   -0.6735]*pi/180; %gives an initial guess for the trajectory generation - mostly used for more complex segments
AllVariables.Arm(2).armtype=  2; %armtype 1 == normal, %armtype 2= SRMS

for i = 1:length(AllVariables.Arm)
    teach = 0;
    AllVariables.bot(i)= GenerateBot_singleArm(AllVariables.Arm(i).ArmLength,AllVariables.Arm(i).IsPlanar,AllVariables.Arm(i).armtype,AllVariables.Arm(i).Armoffset,teach);
    AllVariables.bot(i).base = [eye(3,3) AllVariables.Arm(i).xyz_base';0 0 0 1];
    AllVariables.bot(i).qlim =AllVariables.Arm(i).JointLimits;
end
