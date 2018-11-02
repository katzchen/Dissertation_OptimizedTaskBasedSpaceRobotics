function [x,fval,AllVariables] = GA_RunDualArmCompareWithOriginal_SingleArm1()
close all, clc, clear all

%setting up colors to make nice plots. To keep arm1 purple and arm2 green
ncolors = 2;
greenmap = [ 0 1/3 0; 191/256 191/256 0]
greenmap = [zeros(ncolors,1) [1:2]' zeros(ncolors,1)]/(ncolors+1); %form arm 1
pmap = [[1:2]' zeros(ncolors,1) [1:2]']/(ncolors+1); %form arm 2
bmap = [zeros(ncolors,1) zeros(ncolors,1) [2 1]']/(ncolors+1); %for optimized arm 1
rmap = [[1 .5]' [1 .5]' zeros(ncolors,1) ]; %for optimized arm 2
lightblue = [zeros(ncolors,1) [2 1]' [2 1]']/(ncolors+1); %for optimized arm 1


%%
%Create an array of all other variables htat will be passed in
%This is to just make it easier to keep track/pass all information through
%the GA


%All Task related variables
armnumber =0;
allowsinglearmanswers =0;
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
x1 = AllVariables.Arm(1).ArmLength(2:3)

if armnumber == 1
    x2= [3.5963 6.0854]
    %x3 = [4.2939 3.7613]
    x3=[3.1317  5.8886]
    x2 = [5 5.8]
    xyzbase1 = [0 0.078 0]
    xyzbase2 = [0 0.3868 0]
    
elseif armnumber == 2
    x2 = [3.0309 5.6512]
    x3 = [4.567 1.917]
else
     %  x2 = [4.966 1.627 3.5 3.223] %OPTIMIZED INDIVIDUALLY
     x2 = [3.5963 6.0854 3.0309 5.6512]
    
    
    x3 = [4.7936 1.6087 3.0548 3.3718] %OPTIMZIED SIMULTANEOUSLY
    base1y = AllVariables.Arm(1).xyz_base(2)
    base2y = AllVariables.Arm(2).xyz_base(2)
    xyzbase3 =[ 0 base1y 0; 0 base2y 0];
    
    %  x2 = [3.5963 6.0854 3.0309 5.6512]
    %   x3 = [5.236712 1.5872749 2.769334984 4.13407859]
    %  x3=[5.236711959008125   0.961452404227979   3.506954863251417   3.336173458449831]
    % x3=[5.236711959008125   0.97  3.506954863251417   3.336173458449831]
    % xyzbase1 = [0 -1.802081539842639 0; 0 4.561822920062879 0];
 
    %BASE OPTIMZIATION
 %   x3 =[5.062 1.571 2.34 2.781]
%    xyzbase3=[0 -1.189 0;0 3.689 0];
    %    xyzbase1=[0 -2.89861221 0; 0 3.76079095 0]
    
    base1y = AllVariables.Arm(1).xyz_base(2)
    base2y = AllVariables.Arm(2).xyz_base(2)
    figure
    subplot(2,4,1)
    list = {'Arm1','Arm2'};
    c = categorical(list,list);
    b=bar(c,[[x1 0 0]' [0 0 x1]']','stacked')
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=pmap(1,:)
    b(4).FaceColor=pmap(2,:)
   % b(5).FaceColor = greenmap(1,:)*3;
    %b(8).FaceColor = pmap(1,:)*3;
    title('Original Design')
    ylabel('Distance (m)')
    yaxis([0 12.2])
    set(gca,'FontSize',18)
    
    subplot(2,4,2)
    b=bar(c,[[x3(1:2) 0 0]' [0 0 x3(3:4)]']','stacked')
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=pmap(1,:)
    b(4).FaceColor=pmap(2,:)
    
    title({'Individual';'Optimization'})
    ylabel('Length (m)')
    yaxis([0 12.2])
    set(gca,'FontSize',18)
    
    subplot(2,4,3)
    b=bar(c,[[x2(1:2) 0 0]' [0 0 x2(3:4)]']','stacked')
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=pmap(1,:)
    b(4).FaceColor=pmap(2,:)
    title({'Simultaneous';'Optimization'})
    ylabel('Length (m)')
    legend('Arm1: Link 2','Arm1: Link 3','Arm2: Link 2','Arm2: Link 3')
    yaxis([0 12.2])
    set(gca,'FontSize',18)
    
    
        base1y = AllVariables.Arm(1).xyz_base(2)
    base2y = AllVariables.Arm(2).xyz_base(2)
    subplot(2,4,5)
    list = {'Arm1','Arm2'};
    c = categorical(list,list);
    b=bar(c,[[base1y 0]' [0 base2y]']','stacked')
    b(1).FaceColor=greenmap(1,:)*2.75
    b(2).FaceColor=greenmap(2,:)
    b(2).FaceColor=pmap(1,:)*2.75
   % b(5).FaceColor = greenmap(1,:)*3;
    %b(8).FaceColor = pmap(1,:)*3;
   % title('Original Design')
    ylabel('Distance (m)')
    yaxis([-3 4])
    set(gca,'FontSize',18)
  
            base1y = AllVariables.Arm(1).xyz_base(2)
    base2y = AllVariables.Arm(2).xyz_base(2)
    subplot(2,4,6)
    list = {'Arm1','Arm2'};
    c = categorical(list,list);
    b=bar(c,[[base1y 0]' [0 base2y]']','stacked')
    b(1).FaceColor=greenmap(1,:)*2.75
    b(2).FaceColor=greenmap(2,:)
    b(2).FaceColor=pmap(1,:)*2.75
   % b(5).FaceColor = greenmap(1,:)*3;
    %b(8).FaceColor = pmap(1,:)*3;
  %  title('Original Design')
    ylabel('Distance (m)')
    yaxis([-3 4])
    set(gca,'FontSize',18)
  
    subplot(2,4,7)
    b=bar(c,[[xyzbase3(1,2) 0]' [0 xyzbase3(2,2)]']','stacked')
    b(1).FaceColor=greenmap(1,:)*2.75
    b(2).FaceColor=greenmap(2,:)
    b(2).FaceColor=pmap(1,:)*2.75
  % % b(4).FaceColor=pmap(2,:)
  %  title('Parallel Optimization')
    ylabel('Distance (m)')
    legend('Base Arm1','Base Arm 2','Location','best')
    yaxis([-3 4])
    set(gca,'FontSize',18)
    
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
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
if armnumber < 1
    Links1 = [AllVariables.bot(1).a;AllVariables.bot(2).a];
else
    Links1=AllVariables.bot(armnumber).a
end
[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
PlotArms_structure(AllVariables.Arm,AllVariables.Task,Kinematics,Dynamics,bot);
CompiledResults1 = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
xLink1=[CompiledResults1(1).MassResults.Link.InnerDiameter CompiledResults1(1).MassResults.Link.OuterDiameter-CompiledResults1(1).MassResults.Link.InnerDiameter];
L_link_joint1 = zeros(6,1);

%Chagne and Run Other
'Running 2nd config'
AllVariables = LoadAllVariables_SRMS_simple();
AllVariables = ChangeArmLength(15.2,AllVariables,1);
AllVariables = ChangeArmLength(15.2,AllVariables,2);
if armnumber < 1
    [AllVariables]= ChangeArmLengthMainJoints(x2(1:2),AllVariables,1);
    [AllVariables]= ChangeArmLengthMainJoints(x2(3:4),AllVariables,2);
    Links2 = [AllVariables.bot(1).a;AllVariables.bot(2).a];
else
    [AllVariables]= ChangeArmLengthMainJoints(x2,AllVariables,armnumber);
    %[AllVariables]= ChangeArmLengthMainJoints([3.5963 6.0854],AllVariables,1);
    [AllVariables]= ChangeArmBase(xyzbase1,AllVariables,armnumber);
    Links2 = AllVariables.bot(armnumber).a;
end
AllVariables.Arm(1).plot=0;%0 = no plot
AllVariables.Arm(2).plot=0;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
AllVariables.Arm(1).linkcolor =bmap(1,:);
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
AllVariables = LoadAllVariables_SRMS_simple(); %reset everything
AllVariables = ChangeArmLength(15.2,AllVariables,1); %reset variables
AllVariables = ChangeArmLength(15.2,AllVariables,2); %reset variables
if armnumber < 1
    [AllVariables]= ChangeArmLengthMainJoints(x3(1:2),AllVariables,1);
    [AllVariables]= ChangeArmLengthMainJoints(x3(3:4),AllVariables,2);
    [AllVariables]= ChangeArmBase(xyzbase3(1,:),AllVariables,1);
    [AllVariables]= ChangeArmBase(xyzbase3(2,:),AllVariables,2);
    Links3 = [AllVariables.bot(1).a;AllVariables.bot(2).a];
else
    [AllVariables]= ChangeArmBase(xyzbase2,AllVariables,armnumber);
    [AllVariables]= ChangeArmLengthMainJoints(x3,AllVariables,armnumber);
    Links3 = AllVariables.bot(armnumber).a;
end
AllVariables.Arm(1).plot=1;%0 = no plot
AllVariables.Arm(2).plot=1;%0 = no plot
AllVariables.Task.plot=0; %0 = no plot
%AllVariables.Arm(1).linkcolor =lightblue(1,:)
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

%percentage of structural mass in original design
SM1= 0;
SM2 = 0;
SM1b = 0;
SM2b=0;
JM1 = 0;
JM2 = 0;
JM1b=0;
JM2b = 0;

for i = 1:6
    SM1= SM1+CompiledResults1(1).MassResults.LinkMass(i);
    SM2= SM2+CompiledResults1(2).MassResults.LinkMass(i);
    SM1b= SM1b+CompiledResults1(3).MassResults.LinkMass(1,i);
    SM2b= SM2b+CompiledResults1(3).MassResults.LinkMass(2,i);
    JM1 = JM1+CompiledResults1(1).MassResults.MotorMass(i);
    JM2 = JM2+CompiledResults1(2).MassResults.MotorMass(i);
    JM1b = JM1b+CompiledResults1(3).MassResults.MotorMass(1,i);
    JM2b = JM2b+CompiledResults1(3).MassResults.MotorMass(1,i);
end
PercentM1 = SM1/CompiledResults1(1).MassResults.SystemTotal*100;
PercentM2 = SM2/CompiledResults1(2).MassResults.SystemTotal*100;
PercentM1b = SM1b/CompiledResults1(3).MassResults.SystemTotal*100;
PercentM2b = SM2b/CompiledResults1(3).MassResults.SystemTotal*100;

PercentJM1 = JM1/CompiledResults1(1).MassResults.SystemTotal*100;
PercentJM2 = JM2/CompiledResults1(2).MassResults.SystemTotal*100;
PercentJM1b = JM1b/CompiledResults1(3).MassResults.SystemTotal*100;
PercentJM2b = JM2b/CompiledResults1(3).MassResults.SystemTotal*100;

figure
data=[[PercentM1 0]; [0 PercentM2]; [PercentM1b PercentM2b]]
b=bar(data,'stacked')
b(1).FaceColor=AllVariables.Arm(1).linkcolor
b(2).FaceColor=AllVariables.Arm(2).linkcolor
set(gca,'FontSize',24)
legend('Arm 1','Arm 2')
ylabel('Percent of Mass due to Links (%)')
box off

figure
clear data
data=[[PercentJM1 0]; [0 PercentJM2]; [PercentJM1b PercentJM2b]]
b=bar(data,'stacked')
b(1).FaceColor=AllVariables.Arm(1).linkcolor
b(2).FaceColor=AllVariables.Arm(2).linkcolor
set(gca,'FontSize',24)
legend('Arm 1','Arm 2')
ylabel('Percent of Mass due to Joints(%)')
box off

figure
clear data
data =[[CompiledResults1(3).MassResults.SystemTotal 0 0];[0  CompiledResults2(3).MassResults.SystemTotal 0];[0 0 CompiledResults3(3).MassResults.SystemTotal]]
bar(data,4)
set(gca,'FontSize',24)
ylabel('Total Mass (kg)')
box off


AllVariables = LoadAllVariables_SRMS_simple();
figure
list = {'Single Arm 1','Single Arm 2','Dual Arm Arm1+Arm2'};
c = categorical(list,list);
data =[ [CompiledResults1(1).MassResults.SystemTotal 0 CompiledResults1(3).MassResults.MassperArm(1)]; [0 CompiledResults1(2).MassResults.SystemTotal CompiledResults1(3).MassResults.MassperArm(2)] ];
b= bar(c,data','stacked')
b(1).FaceColor=AllVariables.Arm(1).linkcolor
b(2).FaceColor=AllVariables.Arm(2).linkcolor
set(gca,'FontSize',24)
legend('Arm 1','Arm 2')
ylabel('Mass (kg)')
box off

figure
RestofMass = CompiledResults1(1).MassResults.SystemTotal-CompiledResults1(1).MassResults.LinkMass(2)-CompiledResults1(1).MassResults.LinkMass(3);
clear data
list = {'Actual','Estimated'};
c = categorical(list,list);
dataCalc =[CompiledResults1(1).MassResults.LinkMass(3) CompiledResults1(1).MassResults.LinkMass(2) RestofMass]
dataActual=[23 22.7 431-23-22.7]
data =[dataActual;dataCalc];
b= bar(c,data,'stacked')
legend('Link 2','Link 3','Total Mass')
set(gca,'FontSize',24)
box off
ylabel('Mass (kg)')


if armnumber > 0
    for i = 1:6
        if CompiledResults1(armnumber).DynamicsResults.UseStopping(i) == 1
            J1(i) = CompiledResults1(armnumber).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(i);
        else
            J1(i) = CompiledResults1(armnumber).DynamicsResults.Optimize.MaxJointTorquePerJoint(i);
        end
        
        if CompiledResults2(armnumber).DynamicsResults.UseStopping(i) == 1
            J2(i) = CompiledResults2(armnumber).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(i);
        else
            J2(i) = CompiledResults2(armnumber).DynamicsResults.Optimize.MaxJointTorquePerJoint(i);
        end
        
        if CompiledResults3(armnumber).DynamicsResults.UseStopping(i) == 1
            J3(i) = CompiledResults3(armnumber).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(i);
        else
            J3(i) = CompiledResults3(armnumber).DynamicsResults.Optimize.MaxJointTorquePerJoint(i);
        end
    end
else
    for i = 1:6
        for j = 1:2
            if CompiledResults1(3).DynamicsResults.UseStopping(j,i) == 1
                J1(j,i) = CompiledResults1(3).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(j,i);
            else
                J1(j,i) = CompiledResults1(3).DynamicsResults.Optimize.MaxJointTorquePerJoint(j,i);
            end
            
            if CompiledResults2(3).DynamicsResults.UseStopping(j,i) == 1
                J2(j,i) = CompiledResults2(3).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(j,i);
            else
                J2(j,i) = CompiledResults2(3).DynamicsResults.Optimize.MaxJointTorquePerJoint(j,i);
            end
            
            if CompiledResults3(3).DynamicsResults.UseStopping(j,i) == 1
                J3(j,i) = CompiledResults3(3).DynamicsResults.Stop.Optimize.MaxJointTorquePerJoint(j,i);
            else
                J3(j,i) = CompiledResults3(3).DynamicsResults.Optimize.MaxJointTorquePerJoint(j,i);
            end
        end
    end
end


PercentM1 = SM1/CompiledResults1(1).MassResults.SystemTotal*100;
PercentM2 = SM2/CompiledResults1(2).MassResults.SystemTotal*100;
PercentM1b = SM1b/CompiledResults1(3).MassResults.SystemTotal*100;
PercentM2b = SM2b/CompiledResults1(3).MassResults.SystemTotal*100;

figure
data=[[PercentM1 0]; [0 PercentM2]; [PercentM1b PercentM2b]]
b=bar(data,'stacked')
b(1).FaceColor=AllVariables.Arm(1).linkcolor
b(2).FaceColor=AllVariables.Arm(2).linkcolor
set(gca,'FontSize',24)
legend('Arm 1','Arm 2')
ylabel('Percent of Mass due to Links (%)')
box off



%J2 = CompiledResults2(1).DynamicsResults.Optimize.MaxJointTorquePerJoint;
if armnumber == 0
    armnumber = 3;
end

L1 = CompiledResults1(armnumber).MassResults.LinkMass;
L2 = CompiledResults2(armnumber).MassResults.LinkMass;
M1 = CompiledResults1(armnumber).MassResults.MotorMass;
M2 = CompiledResults2(armnumber).MassResults.MotorMass;

TM1 = [CompiledResults1(armnumber).MassResults.SystemTotal NaN];
TM2 = [CompiledResults2(armnumber).MassResults.SystemTotal NaN];

M3 = CompiledResults3(armnumber).MassResults.MotorMass;
L3 = CompiledResults3(armnumber).MassResults.LinkMass;
TM3 = [CompiledResults3(armnumber).MassResults.SystemTotal NaN];


figure
hold on
b=bar([TM1;TM2;TM3]',.85)
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=greenmap(2,:)
b(3).FaceColor=lightblue(1,:)
Y = [TM1(1);TM2(1);TM3(1)]';
Ystring = num2str(Y','%0.1f')
%text([1 2 3],Y,{'450kg','400kg','336kg'},'HorizontalAlignment','center','VerticalAlignment','bottom','fontsize',16);
box off
ylabel('Total Mass (kg)')
%legend('Original, Base0','Original, Base1','Optimize3,Base3')
%title('Comparision of Mass')
set(gca,'FontSize',18)
xaxis([.5 1.5])
xt = get(gca, 'XTick')
set(gca,'xtick',[])
%text(xt, Y,Ystring, 'HorizontalAlignment','center', 'VerticalAlignment','bottom','fontsize',16)



% Links2 = bot(1).a;
% AllVariables = LoadAllVariables_SRMS_simple();
% Links1 = AllVariables.bot(1).a;
% [AllVariables]= ChangeArmLengthMainJoints(x3(1:2),AllVariables,armnumber);
% Links3 = AllVariables.bot(1).a;
%
%x = [InnerDiameter thickness]
%xLink = [OuterDiameter InnerDiameter]
%Ltotal1 =AllVariables.Arm(1).TotalArmLength
figure
subplot(2,2,1)
b=bar([Links1;Links2;Links3]')
legend('Original, Base0','Original, Base1','Optimize3, Base3')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=lightblue(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
xlabel('Arm Segment')
ylabel('Link Length (m)')
%title('Comparison Link Lengths')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(2,2,2)
b=bar([L1;L2;L3]')
legend('Original, Base0','Original, Base1','Optimize3, Base3')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=lightblue(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
xlabel('Arm Segment')
ylabel('Link Mass (kg)')
%title('Comparision of Link Mass')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(2,2,3)
b= bar([J1;J2;J3]')
xlabel('Joint')
ylabel('Max Torque (Nm)')
legend('Original','Optimize: 2 Variable','Optimize: 3 Variable')
legend('Original, Base0','Original, Base1','Optimize3,Base3')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=lightblue(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
%title('Comparision of Max Joint Torques')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(2,2,4)
b=bar([M1;M2;M3]')
xlabel('Joint')
ylabel('Joint Mass (kg)')
legend('Original, Base0','Original, Base1','Optimize3,Base3')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    b(3).FaceColor=lightblue(1,:)
    %  b(4).FaceColor=lightblue(2,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
%legend('Original','Optimize: 2 Variable','Optimize: 3 Variable')
%title('Comparision of Joint Mass')
set(gca,'FontSize',18)
xaxis([0 7])

figure
list = {'Arm1','Arm2'};
c = categorical(list,list);
subplot(1,4,1)
b1=bar(c,J1,'stacked')
title({'Original';'Design'})
ylabel('Max |Torque| (Nm)')
set(gca,'FontSize',24)
yaxis([0 10000])
colormap(parula)
hold all
b2=subplot(1,4,2)
bar(c,J2,'stacked')
title({'Individually';'Optimized'})
ylabel('Max |Torque| (Nm)')
set(gca,'FontSize',24)
yaxis([0 10000])
b3=subplot(1,4,3)
bar(c,J3,'stacked')
title({'Simultaneously';'Optimized'})
set(gca,'FontSize',24)
yaxis([0 10000])
ylabel('Max |Torque| (Nm)')
legend('Joint 1','Joint 2','Joint 3','Joint 4','Jonit 5','Joint 6')



figure
list = {'Arm1','Arm2'};
c = categorical(list,list);
subplot(1,3,1)
J1b = [J1(1,:) 0*J1(1,:); 0*J1(1,:) J1(2,:)];
b1=bar(c,J1b,'stacked')
gmap2 = linspace(.33,1,6);
gmap2 = linspace(.33,1,6);
for i = 1:6
b1(i).FaceColor = [0 gmap2(i) 0];
b1(i+6).FaceColor = [gmap2(i) 0 gmap2(i)];
end
title({'Original';'Design'})
ylabel('Max |Torque| (Nm)')
set(gca,'FontSize',24)
yaxis([0 10000])

subplot(1,3,2)
J1b = [J2(1,:) 0*J1(1,:); 0*J1(1,:) J2(2,:)];
b1=bar(c,J1b,'stacked')
gmap2 = linspace(.33,1,6);
gmap2 = linspace(.33,1,6);
for i = 1:6
b1(i).FaceColor = [0 gmap2(i) 0];
b1(i+6).FaceColor = [gmap2(i) 0 gmap2(i)];
end
title({'Individually';'Optimized'})
ylabel('Max |Torque| (Nm)')
set(gca,'FontSize',24)
yaxis([0 10000])
subplot(1,3,3)
J1b = [J3(1,:) 0*J1(1,:); 0*J1(1,:) J3(2,:)];
b1=bar(c,J1b,'stacked')
gmap2 = linspace(.33,1,6);
gmap2 = linspace(.33,1,6);
for i = 1:6
b1(i).FaceColor = [0 gmap2(i) 0];
b1(i+6).FaceColor = [gmap2(i) 0 gmap2(i)];
end
title({'Simultaneously';'Optimized'})
ylabel('Max |Torque| (Nm)')
set(gca,'FontSize',24)
yaxis([0 10000])
%legend('Arm1: Joint 1','Arm1: Joint 2','Arm1: Joint 3','Arm1: Joint 4','Arm1: Joint 5','Arm1: Joint 6','Arm2: Joint 1','Arm2: Joint 2','Arm2: Joint 3','Arm2: Joint 4','Arm2: Joint 5','Arm2: Joint 6')





figure
subplot(1,2,1)
b= bar([J1;J2]')
xlabel('Joint')
ylabel('Max Torque (Nm)')
legend('Original, Base0','Original, Base1')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    %  b(3).FaceColor=lightblue(1,:)
    %  b(4).FaceColor=lightblue(2,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
%title('Comparision of Max Joint Torques')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
b=bar([M1;M2]')
xlabel('Joint')
ylabel('Joint Mass (kg)')
legend('Original, Base0','Original, Base1')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=greenmap(2,:)
    %  b(3).FaceColor=lightblue(1,:)
    %  b(4).FaceColor=lightblue(2,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
set(gca,'FontSize',18)
xaxis([0 7])

figure
subplot(1,2,1)
list = {'Original Design','Optimized Individually'};
c = categorical(list,list);
data = [[CompiledResults1(1).MassResults.SystemTotal 0]' [0 CompiledResults2(1).MassResults.SystemTotal]']% CompiledResults3(2).MassResults.SystemTotal ]
b=bar(data,4)
b(1).FaceColor = greenmap(1,:);
b(2).FaceColor = greenmap(2,:);
xaxis([-.25 3])
set(gca,'xtick',[])
set(gca,'FontSize',18)
title('Arm 1')
ylabel('Total System Mass (kg)')
box off

subplot(1,2,2)
list = {'Original Design','Optimized Individually'};
c = categorical(list,list);
data = [[CompiledResults1(2).MassResults.SystemTotal 0]' [0 CompiledResults2(2).MassResults.SystemTotal]']% CompiledResults3(2).MassResults.SystemTotal ]
b=bar(data,4)
b(1).FaceColor = pmap(1,:);
b(2).FaceColor = pmap(2,:);
xaxis([-.25 3])
set(gca,'xtick',[])
set(gca,'FontSize',18)
title('Arm 2')
ylabel('Total System Mass (kg)')

figure
hold on
list = {'Original Design','Optimized Individually','Optimized Together'};
c = categorical(list,list);
data =[CompiledResults1(3).MassResults.MassperArm;CompiledResults2(3).MassResults.MassperArm;CompiledResults3(3).MassResults.MassperArm]
b=bar(data,'stacked')
ylabel('Total System Mass (kg)')
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=pmap(1,:)
set(gca,'FontSize',18)
legend('Arm 1','Arm 2')
xaxis([.5 3.5])
set(gca,'xtick',[])
box off


figure
hold on
list = {'Original Design','Optimized Individually'};
c = categorical(list,list);
data =[CompiledResults1(3).MassResults.MassperArm;CompiledResults2(3).MassResults.MassperArm]
b=bar(data,'stacked')
ylabel('Total System Mass (kg)')
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=pmap(1,:)
set(gca,'FontSize',18)
legend('Arm 1','Arm 2')
set(gca,'xtick',[])
box off


figure
hold on
list = {'Optimized Individually','Optimized Together'};
c = categorical(list,list);
data =[CompiledResults2(3).MassResults.MassperArm;CompiledResults3(3).MassResults.MassperArm]
b=bar(data,'stacked')
ylabel('Total System Mass (kg)')
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=pmap(1,:)
set(gca,'FontSize',18)
legend('Arm 1','Arm 2')
set(gca,'xtick',[])
box off

figure
hold on
b=bar([TM1;TM2;TM3]')
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=greenmap(2,:)
b(3).FaceColor=lightblue(1,:)
Y = [TM1(1);TM2(1);TM3(1)]';
Ystring = num2str(Y','%0.1f')
%text([1 2 3],Y,{'450kg','400kg','336kg'},'HorizontalAlignment','center','VerticalAlignment','bottom','fontsize',16);
box off
ylabel('Total Mass (kg)')
legend('Original, Base0','Original, Base1','Optimize3,Base3')
title('Comparision of Mass')
set(gca,'FontSize',18)
xaxis([.5 1.5])
xt = get(gca, 'XTick')
set(gca,'xtick',[])
text(xt, Y,Ystring, 'HorizontalAlignment','center', 'VerticalAlignment','bottom','fontsize',16)






figure
subplot(2,2,2)
b= bar([J1;J2;J3]')
xlabel('Joint')
ylabel('Max Torque (Nm)')
legend('Original','Optimize: 2 Variable','Optimize: 3 Variable')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=bmap(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
b(3).FaceColor=lightblue(1,:)
title('Comparision of Max Joint Torques')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
b=bar([M1;M2;M3]')
xlabel('Joint')
ylabel('Joint Mass (kg)')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=bmap(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
b(3).FaceColor=lightblue(1,:)
legend('Original','Optimized','Optimize with Base Change')
title('Comparision of Actuator Mass')
set(gca,'FontSize',18)
xaxis([0 7])


figure
subplot(1,2,1)
b=bar([Links1;Links2;Links3]')

if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=bmap(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end

b(3).FaceColor=lightblue(1,:)
xlabel('Arm Segment')
ylabel('Link Length (m)')
legend('Original','Optimized','Optimize with Base')
title('Comparison Link Lengths')
set(gca,'FontSize',18)
xaxis([0 7])

subplot(1,2,2)
b=bar([L1;L2;L3]')
if armnumber == 1
    b(1).FaceColor=greenmap(1,:)
    b(2).FaceColor=bmap(1,:)
else
    b(1).FaceColor=pmap(1,:)
    b(2).FaceColor=rmap(1,:)
end
b(3).FaceColor=lightblue(1,:)
xlabel('Arm Segment')
ylabel('Link Mass (kg)')
legend('Original','Optimized','Optimize with Base')
title('Comparision of Link Mass')
set(gca,'FontSize',18)
xaxis([0 7])


%
% figure
% subplot(1,2,2)
% bar([L1+M1;L2+M2;L3+M3]')
% xlabel('Joint')
% ylabel('Total Segment Mass (kg)')
% legend('Original','Optimized with original base','Optimize with Base')
% title('Comparision of Mass')
% set(gca,'FontSize',18)
% xaxis([0 7])

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





figure
hold on
b=bar([TM1;TM2;TM3]')
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=bmap(1,:)
b(3).FaceColor=lightblue(1,:)
Y = [TM1(1);TM2(1);TM3(1)]';
Ystring = num2str(Y','%0.1f')
%text([1 2 3],Y,{'450kg','400kg','336kg'},'HorizontalAlignment','center','VerticalAlignment','bottom','fontsize',16);
box off
ylabel('Total Mass (kg)')
legend('Original','Optimized','Optimize with Base')
title('Comparision of Mass')
set(gca,'FontSize',18)
xaxis([.5 1.5])
xt = get(gca, 'XTick')
set(gca,'xtick',[])
text(xt, Y, {'450kg','400kg','336kg'}, 'HorizontalAlignment','center', 'VerticalAlignment','bottom','fontsize',16)


if armnumber == 3
    %arm number was changed above so lets do this
    
    %2 figures for the arm1, arm2 and total mass for the original and
    %optimized arm
    figure
    list = {'Arm 1','Arm 2','Arm1+Arm2'};
    c = categorical(list,list);
    bardata=[CompiledResults1(3).MassResults.MassperArm' CompiledResults2(3).MassResults.MassperArm'; [CompiledResults1(3).MassResults.SystemTotal CompiledResults2(3).MassResults.SystemTotal]]
    b=   bar(c,bardata)
    legend('Original Design','Optimized')
    ylabel('Total Mass (kg)')
    set(gca,'FontSize',18)
    
    %plot joint torques per arm
    figure
    for i = 1:2
        subplot(1,2,i)
        b = bar([J1(i,:);J2(i,:)]')
        ylabel('Max Torque (Nm)')
        xlabel('Joint')
        set(gca,'FontSize',18)
        if i == 1
            title('Arm 1')
        else
            title('Arm 2')
        end
        legend('Original Design','Optimized')
        xaxis([0 7])
        
        figure
        for i = 1:2
            subplot(1,2,i)
            b = bar([L1(i,:);L2(i,:)]')
            ylabel('Link Mass (kg)')
            xlabel('Joint')
            set(gca,'FontSize',18)
            if i == 1
                title('Arm 1')
            else
                title('Arm 2')
            end
            legend('Original Design','Optimized')
            xaxis([0 7])
        end
        
    end
    legend('Original Design','Optimized')
    
    
    figure
    for i = 1:2
        subplot(1,2,i)
        b = bar([M1(i,:);M2(i,:)]')
        ylabel('Joint Mass(kg)')
        xlabel('Joint')
        set(gca,'FontSize',18)
        if i == 1
            title('Arm 1')
        else
            title('Arm 2')
        end
        xaxis([0 7])
    end
    legend('Original Design','Optimized')
    
    
end



dataset=[450 400 440; 440 400 336]

list = {'Original base','New Base'};
c = categorical(list,list);
figure
hold on
b=bar(c,dataset)
b(1).FaceColor=greenmap(1,:)
b(2).FaceColor=bmap(1,:)
b(3).FaceColor=lightblue(1,:)
Y = dataset;
Ystring = num2str(Y','%0.1f')
%text(1:6,Y,{'450kg','400kg','336kg'},'HorizontalAlignment','center','VerticalAlignment','bottom','fontsize',16);
box off
ylabel('Total Mass (kg)')
legend('Original','Optimized','Optimize with Base')
set(gca,'FontSize',18)



subplot(2,2,3)
list = {'Lower Link','Upper Link'};
c = categorical(list,list);
bar(c,[x1; x2; x3]')
%legend('Original','Optimized with original base','Optimize with Base','Location','best')
set(gca,'FontSize',18)
ylabel('Link Length (m)')
title('Optimized Link Lengths')
'encose'


list = {'Arm 1: Original','Arm 1: Otpimized','Arm 2: Original','Arm 2: Optimized'};
c = categorical(list,list);
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

'endcode'