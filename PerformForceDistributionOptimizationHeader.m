function [T,MassPerTime,MassPerArm,MassAll,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe] = PerformForceDistributionOptimizationHeader(Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,MassInertia,GearRatio,...
    ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,qdot_all,Payload_XYZ)
%the purpose of this is to allow thish file to be editted with whatever
%I'm hacking in my optimization plot code here because there is a
%discrepency with OptimizationMass Clean Results. 

%WHEN I"M DONE UNCOMMENT THIS PART OUT!

%[T,MassPerTime,MassPerArm,MassAll,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformLoptimization_choose(Narms,Tpoints,Njoints,Di_all0,Di_all0ArmthenPlanar_inversetranspose,...
%    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all);

%%
if Narms > 1
[T,MassPerTime,MassPerArm,MassAll,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe] =PerformLoptimization_choose(Narms,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);
else
  [T,MassPerTime,MassPerArm,MassAll,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformSingleoptimization_choose(1,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);
end

    
testoptimization=0;

if testoptimization > 0 &&  Narms > 1
    
[Tcalc0,MaxMassPerJointL,MassPerArmL,MassSystemL,ForceOnEachArm_baseframeL,ForceOnEachArm_ArmEEframeL] =PerformLoptimization_choose(Narms,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);

[Tcalc0S1,MaxMassPerJointS1,MassPerArmS1,MassSystemS1,ForceOnEachArm_baseframeS1,ForceOnEachArm_ArmEEframeS1]=PerformSingleoptimization_choose(1,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);
[Tcalc0S2,MaxMassPerJointS2,MassPerArmS2,MassSystemS2,ForceOnEachArm_baseframeS2,ForceOnEachArm_ArmEEframeS2]=PerformSingleoptimization_choose(2,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);
% 
[Tcalc0Even,MaxMassPerJointEven,MassPerArmEven,MassSystemEven,ForceOnEachArm_baseframeEven,ForceOnEachArm_ArmEEframeEven]=PerformEvenoptimization_choose(Narms,Tpoints,Njoints,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdot_all,Payload_XYZ);

x0 = reshape(ForceOnEachArm_baseframeL,[Tpoints*Narms*6 1]);
choose = 'e'
[Tcalc0E,MaxMassPerJointE,MassPerArmE,MassSystemE,ForceOnEachArm_baseframeE,ForceOnEachArm_ArmEEframeE]= PerformNMoptimization_choose(x0,choose,Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,...
    MassInertia,GearRatio,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,qdot_all);
choose = 'm'
[Tcalc0M,MaxMassPerJointM,MassPerArmM,MassSystemM,ForceOnEachArm_baseframeM,ForceOnEachArm_ArmEEframeM]= PerformNMoptimization_choose(x0,choose,Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,...
    MassInertia,GearRatio,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,qdot_all);
choose = 't'
[Tcalc0T,MaxMassPerJointT,MassPerArmT,MassSystemT,ForceOnEachArm_baseframeT,ForceOnEachArm_ArmEEframeT]= PerformNMoptimization_choose(x0,choose,Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,...
    MassInertia,GearRatio,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,qdot_all);
choose = 'p'
[Tcalc0P,MaxMassPerJointP,MassPerArmP,MassSystemP,ForceOnEachArm_baseframeP,ForceOnEachArm_ArmEEframeP]= PerformNMoptimization_choose(x0,choose,Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,...
    MassInertia,GearRatio,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_allEE,Di_all0ArmthenPlanar_inversetranspose,qdot_all);


% %Actuator Mass- NO DYNAMICS DUE TO STRUCTURE
 MassSystemS1
 MassSystemS2
 MassSystemP
 MassSystemT
 MassSystemM
 MassSystemE
 MassSystemL
 Tcalc0S1
 Tcalc0S2
 
 
 
 A1= ForceOnEachArm_baseframeL(:,1,:);
A1b= reshape(A1,[6 Tpoints]);
A2= ForceOnEachArm_baseframeL(:,2,:);
A2b= reshape(A2,[6 Tpoints]);
clear ForceOnEachArm_baseframeL
ForceOnEachArm_baseframeL(:,:,1) = A1b;
ForceOnEachArm_baseframeL(:,:,2) = A2b;

%%
%Force Plots- single and even
for graph = 1
    figure
    subplot(3,2,1)
    hold all
    plot(ForceOnEachArm_baseframeS1(1,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(1,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(1,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(1,:,2),'*')
    xlabel('Time Step')
    ylabel('Force (N) on Arm')
    title('Force in X')
    set(gca,'FontSize',18)
set(gca,'FontSize',18)
    
    subplot(3,2,3)
    hold all
    plot(ForceOnEachArm_baseframeS1(2,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(2,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(2,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(2,:,2),'*')
    xlabel('Time Step')
    ylabel('Force (N) on Arm')
    title('Force in Y')
    set(gca,'FontSize',18)
    
    subplot(3,2,5)
    hold all
    plot(ForceOnEachArm_baseframeS1(3,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(3,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(3,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(3,:,2),'*')
    xlabel('Time Step')
    ylabel('Force (N) on Arm')
    title('Force in Z')
    set(gca,'FontSize',18)
    
    subplot(3,2,2)
    hold all
    plot(ForceOnEachArm_baseframeS1(4,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(4,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(4,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(4,:,2),'*')
    xlabel('Time Step')
    ylabel('Moment (Nm) on Arm')
    title('Moment about X')
    legend('Single Arm1','Single Arm2','Evenly Distributed Arm 1','Evenly Distributed Arm 2','Location','best')
    set(gca,'FontSize',18)
    
    subplot(3,2,4)
    hold all
    plot(ForceOnEachArm_baseframeS1(5,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(5,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(5,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(5,:,2),'*')
    xlabel('Time Step')
    ylabel('Moment (Nm) on Arm')
    title('Moment about Y')
    set(gca,'FontSize',18)
    subplot(3,2,6)
    hold all
    plot(ForceOnEachArm_baseframeS1(6,:,1),'o')
    plot(ForceOnEachArm_baseframeS2(6,:,1),'x')
    plot(ForceOnEachArm_baseframeEven(6,:,1),'s black')
    plot(ForceOnEachArm_baseframeEven(6,:,2),'*')
    xlabel('Time Step')
    ylabel('Moment (Nm) on Arm')
    title('Moment about Z')
    set(gca,'FontSize',18)
    end
%%
%Force Plots- Linear and Single
for graph = 1
figure
subplot(3,2,1)
hold all
plot(ForceOnEachArm_baseframeS1(1,:,1),'o')
plot(ForceOnEachArm_baseframeS2(1,:,1),'x')
plot(ForceOnEachArm_baseframeL(1,:,1),'s black')
plot(ForceOnEachArm_baseframeL(1,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in X')
set(gca,'FontSize',18)

subplot(3,2,3)
hold all
plot(ForceOnEachArm_baseframeS1(2,:,1),'o')
plot(ForceOnEachArm_baseframeS2(2,:,1),'x')
plot(ForceOnEachArm_baseframeL(2,:,1),'s black')
plot(ForceOnEachArm_baseframeL(2,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Y')
set(gca,'FontSize',18)

subplot(3,2,5)
hold all
plot(ForceOnEachArm_baseframeS1(3,:,1),'o')
plot(ForceOnEachArm_baseframeS2(3,:,1),'x')
plot(ForceOnEachArm_baseframeL(3,:,1),'s black')
plot(ForceOnEachArm_baseframeL(3,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Z')
set(gca,'FontSize',18)

subplot(3,2,2)
hold all
plot(ForceOnEachArm_baseframeS1(4,:,1),'o')
plot(ForceOnEachArm_baseframeS2(4,:,1),'x')
plot(ForceOnEachArm_baseframeL(4,:,1),'s black')
plot(ForceOnEachArm_baseframeL(4,:,2),'*')
xlabel('Time Step')
ylabel('Moment (Nm) on Arm')
title('Moment about X')
legend('Single Arm1','Single Arm2','Mass Distributed Arm 1','Mass Distributed Arm 2','Location','best')
set(gca,'FontSize',18)

subplot(3,2,4)
hold all
plot(ForceOnEachArm_baseframeS1(5,:,1),'o')
plot(ForceOnEachArm_baseframeS2(5,:,1),'x')
plot(ForceOnEachArm_baseframeL(5,:,1),'s black')
plot(ForceOnEachArm_baseframeL(5,:,2),'*')
xlabel('Time Step')
ylabel('Moment (Nm) on Arm')
title('Moment about Y')
set(gca,'FontSize',18)
subplot(3,2,6)
hold all
plot(ForceOnEachArm_baseframeS1(6,:,1),'o')
plot(ForceOnEachArm_baseframeS2(6,:,1),'x')
plot(ForceOnEachArm_baseframeL(6,:,1),'s black')
plot(ForceOnEachArm_baseframeL(6,:,2),'*')
xlabel('Time Step')
ylabel('Moment (Nm) on Arm')
title('Moment about Z')
set(gca,'FontSize',18)
end
%%
%Force Plots- Total Energy and Single
for graph = 1
figure
subplot(3,2,1)
hold all
plot(ForceOnEachArm_baseframeS1(1,:,1),'o')
plot(ForceOnEachArm_baseframeS2(1,:,1),'x')
plot(ForceOnEachArm_baseframeE(1,:,1),'s black')
plot(ForceOnEachArm_baseframeE(1,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in X')

subplot(3,2,3)
hold all
plot(ForceOnEachArm_baseframeS1(2,:,1),'o')
plot(ForceOnEachArm_baseframeS2(2,:,1),'x')
plot(ForceOnEachArm_baseframeE(2,:,1),'s black')
plot(ForceOnEachArm_baseframeE(2,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Y')

subplot(3,2,5)
hold all
plot(ForceOnEachArm_baseframeS1(3,:,1),'o')
plot(ForceOnEachArm_baseframeS2(3,:,1),'x')
plot(ForceOnEachArm_baseframeE(3,:,1),'s black')
plot(ForceOnEachArm_baseframeE(3,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Z')

subplot(3,2,2)
hold all
plot(ForceOnEachArm_baseframeS1(4,:,1),'o')
plot(ForceOnEachArm_baseframeS2(4,:,1),'x')
plot(ForceOnEachArm_baseframeE(4,:,1),'s black')
plot(ForceOnEachArm_baseframeE(4,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about X')
legend('Single Arm1','Single Arm2','Energy Distributed Arm 1','Energy Distributed Arm 2','Location','best')


subplot(3,2,4)
hold all
plot(ForceOnEachArm_baseframeS1(5,:,1),'o')
plot(ForceOnEachArm_baseframeS2(5,:,1),'x')
plot(ForceOnEachArm_baseframeE(5,:,1),'s black')
plot(ForceOnEachArm_baseframeE(5,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about Y')
subplot(3,2,6)
hold all
plot(ForceOnEachArm_baseframeS1(6,:,1),'o')
plot(ForceOnEachArm_baseframeS2(6,:,1),'x')
plot(ForceOnEachArm_baseframeE(6,:,1),'s black')
plot(ForceOnEachArm_baseframeE(6,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about Z')
end

%%
%Energy comparision
%Force Plots- Total and Peak
for graph = 1
figure
subplot(3,2,1)
hold all
plot(ForceOnEachArm_baseframeP(1,:,1),'o')
plot(ForceOnEachArm_baseframeP(1,:,2),'x')
plot(ForceOnEachArm_baseframeE(1,:,1),'s black')
plot(ForceOnEachArm_baseframeE(1,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in X')

subplot(3,2,3)
hold all
plot(ForceOnEachArm_baseframeP(2,:,1),'o')
plot(ForceOnEachArm_baseframeP(2,:,2),'x')
plot(ForceOnEachArm_baseframeE(2,:,1),'s black')
plot(ForceOnEachArm_baseframeE(2,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Y')

subplot(3,2,5)
hold all
plot(ForceOnEachArm_baseframeP(3,:,1),'o')
plot(ForceOnEachArm_baseframeP(3,:,2),'x')
plot(ForceOnEachArm_baseframeE(3,:,1),'s black')
plot(ForceOnEachArm_baseframeE(3,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Force in Z')

subplot(3,2,2)
hold all
plot(ForceOnEachArm_baseframeP(4,:,1),'o')
plot(ForceOnEachArm_baseframeP(4,:,2),'x')
plot(ForceOnEachArm_baseframeE(4,:,1),'s black')
plot(ForceOnEachArm_baseframeE(4,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about X')
legend('Min Peak Energy Arm1','Min Peak Energy Arm2','Min Total Energy Arm 1','Min Total Energy Arm 2','Location','best')


subplot(3,2,4)
hold all
plot(ForceOnEachArm_baseframeP(5,:,1),'o')
plot(ForceOnEachArm_baseframeP(5,:,2),'x')
plot(ForceOnEachArm_baseframeE(5,:,1),'s black')
plot(ForceOnEachArm_baseframeE(5,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about Y')
subplot(3,2,6)
hold all
plot(ForceOnEachArm_baseframeP(6,:,1),'o')
plot(ForceOnEachArm_baseframeP(6,:,2),'x')
plot(ForceOnEachArm_baseframeE(6,:,1),'s black')
plot(ForceOnEachArm_baseframeE(6,:,2),'*')
xlabel('Time Step')
ylabel('Force (N) on Arm')
title('Moment about Z')
end

%%
%NOT PLOTTED: force diagram at EVERY timestep
for graph = 1
% 
% list = {'x','y','z','rot x','rot y','rot z'};
% c = categorical(list,list);
% figure
% iter= 1;
% for i = 1:Tpoints
%     for armnumber = 1:2
%         subplot(Tpoints,2,iter)
%         iter = iter+1;
%         bar([ForceOnEachArm_baseframeEven(:,i,armnumber) ForceOnEachArm_baseframeL(:,i,armnumber) ForceOnEachArm_baseframeE(:,i,armnumber) ForceOnEachArm_baseframeT(:,i,armnumber) ForceOnEachArm_baseframeM(:,i,armnumber)]);
%         set(gca,'xticklabel',list);
%         ylabel('Force (N)')
%         if (i <2 && armnumber<2)
%             title('Arm1 timestap = 1')
%             legend('Even','Mass MinimizationMethod','Min Energy Method','Min Torque','NL Mass')
%             
%         elseif i == 1 && armnumber == 2
%             title('Arm2, timestap = 1')
%             legend('Even','Mass MinimizationMethod','Min Energy Method','Min Torque','NL Mass')
%         else
%             title(i);
%         end
%     end
% end
end

%plot forces at timestep 1 in bar graph
for graph = 1
    list = {'x','y','z','rot x','rot y','rot z'};
    c = categorical(list,list);
    figure
    iter= 1;
    ForceOnEachArm_baseFrameSingle(:,:,1) =ForceOnEachArm_baseframeS1;
    ForceOnEachArm_baseFrameSingle(:,:,2) =ForceOnEachArm_baseframeS2;
    for i = 1
        minvalue = min(min(min([ForceOnEachArm_baseFrameSingle(:,i,:) ForceOnEachArm_baseframeEven(:,i,:)  ForceOnEachArm_baseframeE(:,i,:) ForceOnEachArm_baseframeL(:,i,:)])));
        maxvalue = max(max(max([ForceOnEachArm_baseFrameSingle(:,i,:) ForceOnEachArm_baseframeEven(:,i,:)  ForceOnEachArm_baseframeE(:,i,:) ForceOnEachArm_baseframeL(:,i,:)])));
        
        for armnumber = 1:2
            subplot(1,2,iter)
            iter = iter+1;
            bar([ForceOnEachArm_baseFrameSingle(:,i,armnumber) ForceOnEachArm_baseframeEven(:,i,armnumber)  ForceOnEachArm_baseframeE(:,i,armnumber) ForceOnEachArm_baseframeL(:,i,armnumber)]);
            set(gca,'xticklabel',list);
            ylabel('Force (N)')
            ylim([minvalue maxvalue])
            if (i <2 && armnumber<2)
                title('Arm1 timestap = 1')
                legend('Single Arm1','Even','Min Energy Method','Linear','Location','best')
                
            elseif i == 1 && armnumber == 2
                title('Arm2, timestap = 1')
                legend('Single Arm2','Even','Min Energy Method','Linear','Location','best')
            else
                title(i);
            end
        end
    end
end

%%
%Lets look at energy
%

%Lets find the max torque for each joint AT TIMESTEP 1
LinTorque(:,1) =((abs(Tcalc0(:,1,1))));
LinTorque(:,2) =((abs(Tcalc0(:,1,2))));
ETorque(:,1) =((abs(Tcalc0E(:,1,1))));
ETorque(:,2) =((abs(Tcalc0E(:,1,2))));
MTorque(:,1) =((abs(Tcalc0M(:,1,1))));
MTorque(:,2) =((abs(Tcalc0M(:,1,2))));
TTorque(:,1) =((abs(Tcalc0T(:,1,1))));
TTorque(:,2) =((abs(Tcalc0T(:,1,2))));
VTorque(:,1) =((abs(Tcalc0Even(:,1,1))));
VTorque(:,2) =((abs(Tcalc0Even(:,1,2))));
PTorque(:,1) =((abs(Tcalc0P(:,1,1))));
PTorque(:,2) =((abs(Tcalc0P(:,1,2))));
S1Torque(:,1) =(abs((Tcalc0S1(:,1)')));
S2Torque(:,1) =(abs((Tcalc0S2(:,1)')));

%bar graph with the timestep 1 torques
for graph= 1
figure
subplot(2,1,1)
bar([S1Torque 0*S2Torque])%; ETorque; TTorque; MTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Single Arm 1')
subplot(2,1,2)
bar([0*S2Torque S2Torque])%; ETorque; TTorque; MTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Single Arm2 ')

figure
subplot(3,2,1)
bar([S1Torque 0*S2Torque])%; ETorque; TTorque; MTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Single Arm 1')
subplot(3,2,2)
bar([0*S2Torque S2Torque])%; ETorque; TTorque; MTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Single Arm2 ')
subplot(3,2,3)
bar([VTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('EVen')
subplot(3,2,4)
bar([LinTorque])%; ETorque; TTorque; MTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Min Mass')
subplot(3,2,5)
bar([ETorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Min Total Energy')
subplot(3,2,6)
bar([PTorque])
legend('Arm1','Arm2')
ylabel('Absolute Torque at Timestep 1')
xlabel('Joint')
title('Peak Energy')
end



figure
Arm1 = [MassPerArmM(1) MassPerArmE(1) MassPerArmT(1) MassPerArmL(1) MassPerArmEven(1) MassPerArmS1(1)]
Arm2 = [MassPerArmM(2) MassPerArmE(2) MassPerArmT(2) MassPerArmL(2) MassPerArmEven(2) MassPerArmS2(2)]
ArmTotal = [MassSystemM MassSystemE MassSystemT MassSystemL MassSystemEven 0];
bar([Arm1; Arm2; ArmTotal])
legend('NL Mass','NL Energy','NL Torque','Lin Mass' ,'Even','Single Arm')
ylabel('Mass(kg)')


figure
Arm1L = [MassPerArmS1(1)  MassPerArmEven(1) MassPerArmE(1) MassPerArmL(1)]
Arm2L = [MassPerArmS2(2)  MassPerArmEven(2) MassPerArmE(2) MassPerArmL(2)]
ArmTotalL = [0 MassSystemEven MassSystemE MassSystemL];
bar([Arm1L; Arm2L; ArmTotalL])
legend('Single Arm','Even','Energy','Mass')
ylabel('Mass(kg)')


end