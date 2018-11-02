function [x,fval,AllVariables] = GA_RunDualArmCompareWithOriginal_SingleArm1()
close all, clc, clear all
%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA/whatever easierx
%All Task related variables
xyzbase = [0 4.1225 0];
armnumber = 2;
allowsinglearmanswers = 0;
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
x1 = AllVariables.Arm(1).ArmLength(2:3)
if armnumber == 1
x2 = [3.5963 6.0854]
x3 = [4.567 1.917]
else
x2 = [3.0309 5.6512]
x3 = [4.567 1.917]    
end

Dmax =AllVariables.Arm(1).MaxDeflectionPerLength;
E =AllVariables.Arm(1).material.E;
Density =AllVariables.Arm(1).material.Density;
MinThickness =AllVariables.Arm(1).material.MinThickness;
poisson =AllVariables.Arm(1).material.poisson;
GraspingDistance = sqrt(AllVariables.Arm(1).PayloadGrip(1)^2+ AllVariables.Arm(1).PayloadGrip(2)^2+AllVariables.Arm(1).PayloadGrip(3)^2);
PayloadForce =  abs(AllVariables.Task.Mpayload*AllVariables.Task.maxAcceleration_xyz);
PayloadMoment = abs(AllVariables.Task.maxAcceleration_rpy*AllVariables.Task.Mpayload*GraspingDistance^2);



%Run Original
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults1 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
xLink1 =[CompiledResults1(1).MassResults.Link.InnerDiameter CompiledResults1(1).MassResults.Link.OuterDiameter-CompiledResults1(1).MassResults.Link.InnerDiameter];
L_link_joint1 = zeros(6,1);
for i =1:6
    L_link_joint1(i) = AllVariables.bot(1).a(i)+AllVariables.bot(1).d(i);
end
[c,ceq,Dcalc,StressLink1] = mass_constraintFunction_thickness_rev2(xLink1,L_link_joint1,Dmax,E,MinThickness,PayloadForce, PayloadMoment,poisson);
[c,linkmass1]=  mass_objectiveFunction_thickness(xLink1,L_link_joint1,Density);


%Chagne and Run Other
'Running 2nd config'
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
[AllVariables]= ChangeArmLengthMainJoints(x2(1:2),AllVariables,armnumber);
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults2 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
xLink2=[CompiledResults2(1).MassResults.Link.InnerDiameter CompiledResults2(1).MassResults.Link.OuterDiameter-CompiledResults2(1).MassResults.Link.InnerDiameter];
L_link_joint2 = zeros(6,1);
for i =1:6
    L_link_joint2(i) = AllVariables.bot(1).a(i)+AllVariables.bot(1).d(i);
end
[c,ceq,Dcalc,StressLink2] = mass_constraintFunction_thickness_rev2(xLink2,L_link_joint2,Dmax,E,MinThickness,PayloadForce, PayloadMoment,poisson);
[c,linkmass2]=  mass_objectiveFunction_thickness(xLink2,L_link_joint2,Density);


%Compare with 3rd
'Running 3rd config'
clear AllVaribles bot Dynamics Kinematics maxPoseErr
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1); %reset variables
[AllVariables]= ChangeArmLengthMainJoints(x3,AllVariables,armnumber);
[AllVariables]= ChangeArmBase(xyzbase,AllVariables,armnumber);
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults3 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
xLink3=[CompiledResults3(1).MassResults.Link.InnerDiameter CompiledResults3(1).MassResults.Link.OuterDiameter-CompiledResults3(1).MassResults.Link.InnerDiameter];
L_link_joint3 = zeros(6,1);
for i =1:6
    L_link_joint3(i) = AllVariables.bot(1).a(i)+AllVariables.bot(1).d(i);
end
[c,ceq,Dcalc,StressLink3] = mass_constraintFunction_thickness_rev2(xLink3,L_link_joint3,Dmax,E,MinThickness,PayloadForce, PayloadMoment,poisson);
[c,linkmass3]=  mass_objectiveFunction_thickness(xLink3,L_link_joint3,Density);


%%
%plots

J1 = CompiledResults1(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
J2 = CompiledResults2(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
J3 = CompiledResults3(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
L1 = CompiledResults1(1).MassResults.LinkMass;
L2 = CompiledResults2(1).MassResults.LinkMass;
L3 = CompiledResults3(1).MassResults.LinkMass;
M1 = CompiledResults1(1).MassResults.MotorMass;
M2 = CompiledResults2(1).MassResults.MotorMass;
M3 = CompiledResults3(1).MassResults.MotorMass;
TM1 = [CompiledResults1(1).MassResults.SystemTotal NaN];
TM2 = [CompiledResults2(1).MassResults.SystemTotal NaN];
TM3 = [CompiledResults3(1).MassResults.SystemTotal NaN];


%x = [InnerDiameter thickness]
%xLink = [OuterDiameter InnerDiameter]
%Ltotal1 =AllVariables.Arm(1).TotalArmLength

figure
subplot(2,1,1)
plot(CompiledResults1(1).DynamicsResults.Optimize.JointTorqueAtTimeStep)
xlabel('Time Step')
ylabel('Torque (Nm)')
legend('Joint 1','2','3','4','5','6','Location','northeastoutside')
title('Joint Torques')
set(gca,'FontSize',18)

npoints = length(CompiledResults1(1).DynamicsResults.Optimize.JointTorqueAtTimeStep(:,1));
subplot(2,1,2)
plot(CompiledResults1(1).DynamicsResults.Optimize.JointTorqueAtTimeStep(2:npoints-1,:))
xlabel('Time Step')
ylabel('Torque (Nm)')
legend('Joint 1','2','3','4','5','6','Location','northeastoutside')
title('Zoomed in Joint Torques')
set(gca,'FontSize',18)

%want to plot masses now
PercentageLinkMass1 = CompiledResults1(1).MassResults.LinkMass/CompiledResults1(1).MassResults.SystemTotal*100;
PercentageMotorMass1 = CompiledResults1(1).MassResults.MotorMass/CompiledResults1(1).MassResults.SystemTotal*100;
JointMass1 = CompiledResults1(1).MassResults.LinkMass+CompiledResults1(1).MassResults.MotorMass;
PercentageLinktoJoint = CompiledResults1(1).MassResults.LinkMass./JointMass1*100;
PercentageMotortoJoint = CompiledResults1(1).MassResults.MotorMass./JointMass1*100;
%lets plot Link Mass, Motor Mass, Percentage of Each per joint
figure
subplot(2,2,1)
bar([CompiledResults1(1).MassResults.LinkMass])
xlabel('Joint Number')
ylabel({'Link Mass','(kg)'})
set(gca,'FontSize',18)
xaxis([0 7])
yaxis([0 max(CompiledResults1(1).MassResults.LinkMass)*1.1])

subplot(2,2,2)
b=bar([CompiledResults1(1).MassResults.MotorMass]');
set(b,'FaceColor','yellow');
xlabel('Joint Number')
ylabel({'Motor Mass','(kg)'})
set(gca,'FontSize',18)
xaxis([0 7])
yaxis([0 max(CompiledResults1(1).MassResults.MotorMass)*1.1])


subplot(2,2,3:4)
bar([PercentageLinkMass1;PercentageMotorMass1]','stacked')
legend('Link Mass','Actuator Mass')
xlabel('Joint Number')
ylabel({'Percentage of'; 'Total Arm Mass (%)'})
set(gca,'FontSize',18)
yaxis([0 max(PercentageLinkMass1+PercentageMotorMass1)*1.1])


figure
subplot(1,2,1)
bar([J1;J2; J3]')
xlabel('Joint')
ylabel('Max Torque (Nm)')
legend('Original','Optimized with original base','Optimize with Base')
title('Comparision of Max Joint Torques')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
bar([M1;M2;M3]')
xlabel('Joint')
ylabel('Actuator Mass (kg)')
legend('Original','Optimized with original base','Optimize with Base')
title('Comparision of Actuator Mass')
set(gca,'FontSize',18)
xaxis([0 7])


figure
subplot(1,2,1)
bar([L1;L2;L3]')
xlabel('Joint')
ylabel('Link Mass (kg)')
legend('Original','Optimized with original base','Optimize with Base')
title('Comparision of Link Mass')
set(gca,'FontSize',18)
xaxis([0 7])


subplot(1,2,2)
bar([L1+M1;L2+M2;L3+M3]')
xlabel('Joint')
ylabel('Total Segment Mass (kg)')
legend('Original','Optimized with original base','Optimize with Base')
title('Comparision of Mass')
set(gca,'FontSize',18)
xaxis([0 7])

figure
subplot(2,2,1)
bar([L1;L2;L3]')
xlabel('Joint')
ylabel('Link Mass (kg)')
legend('Original','Optimized with original base','Optimize with Base','Location','best')
title('Comparision of Link Mass')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(2,2,2)
bar([M1;M2;M3]')
xlabel('Joint')
ylabel('Actuator Mass (kg)')
%legend('Original','Optimized with original base','Optimize with Base','Location','northoutside')
title('Comparision of Actuator Mass')
set(gca,'FontSize',18)
xaxis([0 7])


subplot(2,2,4)
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
legend('Original','Optimized with original base','Optimize with Base','Location','best')
set(gca,'FontSize',18)
xlabel('Joint')
xaxis([0 7])

  subplot(1,2,1)
 bar([L_link_joint1 L_link_joint2 L_link_joint3])
 ylabel('Link Length')
set(gca,'FontSize',18)
xlabel('Joint')
xaxis([0 7])