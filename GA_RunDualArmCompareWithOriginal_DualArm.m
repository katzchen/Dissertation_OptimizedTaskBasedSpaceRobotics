function [x,fval,AllVariables] = GA_RunDualArmCompareWithOriginal_DualArm()
close all, clc, clear all
%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easier
%All Task related variables
armnumber = 0;
allowsinglearmanswers = 1;
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
x1 = [AllVariables.Arm(1).ArmLength(2:3) AllVariables.Arm(2).ArmLength(2:3)]
%2-dual Optimization results
x3 = [4.7936	1.6087	3.0548	3.3718]

%3 = individual optimziation
x3_arm1 = [3.5963 6.0854];
x3_arm2 =  [2.9377 5.5918];
x2 = [x3_arm1 x3_arm2]

DOTHREEARMS = 0; %0 is only do 2
if DOTHREEARMS < 1
x2 = x3;
end

%Run Original
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables.Arm(1).plot=1;%0 = no plot
AllVariables.Arm(2).plot=1;%0 = no plot
AllVariables.Task.plot=1; %0 = no plot
AllVariables.Task.plot=1; %0 = no plot
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);

allowsinglearmanswers = 2;
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults1 = RunDynamicsOnIndividualAndPairNoFilter(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,allowsinglearmanswers,armnumber);

%Chagne and Run Other
'Running 2nd config'
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
[AllVariables]= ChangeArmLengthMainJoints(x2(1:2),AllVariables,1);
[AllVariables]= ChangeArmLengthMainJoints(x2(3:4),AllVariables,2);
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults2 = RunDynamicsOnIndividualAndPairNoFilter(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,allowsinglearmanswers,armnumber);

%Compare with 3rd
%plots

J1 = CompiledResults1(3).DynamicsResults.Optimize.MaxJointTorquePerJoint;
J2 = CompiledResults2(3).DynamicsResults.Optimize.MaxJointTorquePerJoint;
L1 = CompiledResults1(3).MassResults.LinkMass;
L2 = CompiledResults2(3).MassResults.LinkMass;
M1 = CompiledResults1(3).MassResults.MotorMass;
M2 = CompiledResults2(3).MassResults.MotorMass;
TM1 = [CompiledResults1(3).MassResults.SystemTotal ];
TM2 = [CompiledResults2(3).MassResults.SystemTotal ];
M1Arms = [CompiledResults1(3).MassResults.MassperArm TM1]
M2Arms = [CompiledResults2(3).MassResults.MassperArm TM2]

if DOTHREEARMS > 0
    'Running 3rd config'
    AllVariables = LoadAllVariables_SRMS_simple();
    AllVariables = ChangeArmLength(15.2,AllVariables,1);
    AllVariables = ChangeArmLength(15.2,AllVariables,2);
    [AllVariables]= ChangeArmLengthMainJoints(x3(1:2),AllVariables,1);
    [AllVariables]= ChangeArmLengthMainJoints(x3(3:4),AllVariables,2);
    [maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
    PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
    CompiledResults3 = RunDynamicsOnIndividualAndPairNoFilter(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,allowsinglearmanswers,armnumber);
    J3 = CompiledResults3(3).DynamicsResults.Optimize.MaxJointTorquePerJoint;
    L3 = CompiledResults3(3).MassResults.LinkMass;
    M3 = CompiledResults3(3).MassResults.MotorMass;
    TM3 = [CompiledResults3(3).MassResults.SystemTotal ];
    M3Arms = [CompiledResults3(3).MassResults.MassperArm TM3]
    
end


figure
list = {'Arm 1','Arm 2','Total System Mass'};
c = categorical(list,list);
if DOTHREEARMS > 0
bardata = [M1Arms;M2Arms;M3Arms]';
else
bardata = [M1Arms;M2Arms]'
end
bar(c,bardata)
ylabel('Total Mass (kg)')
if DOTHREEARMS > 0
  legend('Original','Optimize: Individual','Optimized: Dual','Location','best')
else
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
end

title('Mass Results for Dual Arm Optimization')
set(gca,'FontSize',18)
%xaxis([.5 1.5])

%x = [InnerDiameter thickness]
%xLink = [OuterDiameter InnerDiameter]
%Ltotal1 =AllVariables.Arm(1).TotalArmLength


figure
subplot(1,2,1)
clear bardata

if DOTHREEARMS > 0
bardata =[J1(1,:);J2(1,:); J3(1,:)]';
else
bardata = [J1(1,:);J2(1,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Max Torque (Nm)')
if DOTHREEARMS > 0
  legend('Original','Optimize: Individual','Optimized: Dual','Location','best')
else
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
end
title('Arm 1')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
clear bardata

if DOTHREEARMS > 0
bardata =[J1(2,:);J2(2,:); J3(2,:)]';
else
bardata = [J1(2,:);J2(2,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Max Torque (Nm)')
if DOTHREEARMS > 0
  legend('Original','Optimize: Individual','Optimized: Dual','Location','best')
else
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
end
title('Arm 2')
set(gca,'FontSize',18)
xaxis([0 7])

figure
subplot(1,2,1)
clear bardata

if DOTHREEARMS > 0
bardata =[M1(1,:);M2(1,:); M3(1,:)]';
else
bardata = [M1(1,:);M2(1,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Actuator Mass (kg)')
%legend('Original','Optimized with original base','Optimize with Base')
title('Arm 1')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
clear bardata

if DOTHREEARMS > 0
bardata =[M1(2,:);M2(2,:); M3(2,:)]';
else
bardata = [M1(2,:);M2(2,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Actuator Mass (kg)')
if DOTHREEARMS > 0
  legend('Original','Optimize: Individual','Optimized: Dual','Location','best')
else
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
end
title('Arm 2')
set(gca,'FontSize',18)
xaxis([0 7])


figure
subplot(1,2,1)
clear bardata
if DOTHREEARMS > 0
    bardata =[L1(1,:);L2(1,:); L3(1,:)]';
else
bardata = [L1(1,:);L2(1,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Link Mass (kg)')
%legend('Original','Optimized with original base','Optimize with Base')
title('Arm 1')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
clear bardata
if DOTHREEARMS > 0
bardata =[L1(2,:);L2(2,:); L3(2,:)]';
else
bardata = [L1(2,:);L2(2,:)]';
end

bar(bardata)
xlabel('Joint')
ylabel('Link Mass (kg)')
if DOTHREEARMS > 0
  legend('Original','Optimize: Individual','Optimized: Dual','Location','best')
else
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
end
title('Arm 2')
set(gca,'FontSize',18)
xaxis([0 7])




figure
bar([TM1;TM2;TM3]')
ylabel('Total Mass (kg)')
%legend('Original','Optimized with original base','Optimize with Base','Location','northoutside')
title('Comparision of Mass')
set(gca,'FontSize',18)
set(gca,'xtick',[])
xaxis([.5 1.5])


subplot(2,2,3)
list = {'Lower Link','Upper Link'};
c = categorical(list,list);
bar(c,[x1; x2; x3]')  
%legend('Original','Optimized with original base','Optimize with Base','Location','best')
set(gca,'FontSize',18)
ylabel('Link Length (m)')
title('Optimized Link Lengths')
  'encose'
  
  
  figure
%   subplot(2,2,2)
%   bar([StressLink1; StressLink2; StressLink3]')
%   ylabel('Stress')
  subplot(1,2,2)
 bar([xLink1(7:12); xLink2(7:12) ;xLink3(7:12)]'*1000)
 ylabel('Thickness (mm)') 
legend('Original','Optimized: Dual','Optimize: Individual','Location','best')
set(gca,'FontSize',18)
xlabel('Joint')
xaxis([0 7])

  subplot(1,2,1)
 bar([L_link_joint1 L_link_joint2 L_link_joint3])
 ylabel('Link Length')
set(gca,'FontSize',18)
xlabel('Joint')
xaxis([0 7])