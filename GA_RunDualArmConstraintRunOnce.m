function [x,fval,AllVariables] = GA_RunDualArmCompareWithOriginal()
close all, clc, clear all
%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easier
%All Task related variables
xyzbase = [0 4.123 0];
armnumber = 1;
x = [4.567 1.917]
allowsinglearmanswers = 0;
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
%Run Original
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults1 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);

%Chagne and Run Other
[AllVariables]= ChangeArmLengthMainJoints(x(1:2),AllVariables,armnumber);
[AllVariables]= ChangeArmBase(xyzbase,AllVariables,armnumber);
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults2 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);

%%
%plots

J1 = CompiledResults1(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
J2 = CompiledResults2(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
L1 = CompiledResults1(1).MassResults.LinkMass;
L2 = CompiledResults2(1).MassResults.LinkMass;
M1 = CompiledResults1(1).MassResults.MotorMass;
M2 = CompiledResults2(1).MassResults.MotorMass;

figure
subplot(1,2,1)
bar([J1;J2]')
xlabel('Joint')
ylabel('Max Torque (Nm)')
legend('Original','Optimized')
title('Comparision of Max Joint Torques')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
bar([M1;M2]')
xlabel('Joint')
ylabel('Actuator Mass (kg)')
legend('Original','Optimized')
title('Comparision of Actuator Mass')
set(gca,'FontSize',18)
xaxis([0 7])


figure
subplot(1,2,1)
bar([L1;L2]')
xlabel('Joint')
ylabel('Link Mass (kg)')
legend('Original','Optimized')
title('Comparision of Link Mass')
set(gca,'FontSize',18)
xaxis([0 7])


subplot(1,2,2)
bar([L1+M1;L2+M2]')
xlabel('Joint')
ylabel('JointMass+LinkMass(kg)')
legend('Original','Optimized')
title('Comparision of Mass')
set(gca,'FontSize',18)
xaxis([0 7])


  
  