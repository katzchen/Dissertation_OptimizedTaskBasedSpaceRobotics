function [MaxTotalTorque_optimize,DynamicResults]=Narm_Ndof_dynamic_GetTorqueToPayloadStopping(Kinematics, Dynamics, TaskVariables, ArmVariables,ploton,bot)

%I want to be able to stop the payload At any point along hte payload
%for that I have a max decelerationg - since I want it to stop in 1 second
%or less.
%what is if just take each set of joint angles I've calculated for the
%trajectory and assume I'm moving at Maximum speed - so set qddot to... max
%somehow... and then set the acceleration to max deceleration. Do I need to
%change qdot?
%qdot = AllVariables.Task.Velocity_angularlimit
%qddot = ??? well go from qdot to 0 in the time limit
%AllVariables.Task.DecelerationTime_xyz = 1;% must stop in 1 secdond
%AllVariables.Task.DecelerationTime_rpy = 1;% must stop in 1 secdond
%qddot = qdot/min(AllVariables.Task.DecelerationTime_xyz)


%I have the relationship for optimal torque distribution where DiTi = DjTj for all i and j
%want to just have a single program for a system given Ftask
%Is Planar - if its planar than I don't want the optimizer to put in any
%forces in the Fz, Tx, Tz places, 1 = YES

%Task Variables
%maxtotaltime = Kinematics.finaltime;
Aworld(:,:) = [Kinematics(1).Payload.A_xyz Kinematics(1).Payload.A_rpy]; %should be the same for all arms since its the payload
Tworld(:,:) = [Kinematics(1).Payload.T_xyz Kinematics(1).Payload.T_rpy];
%Vworld(:,:) = [Kinematics(1).Payload.V_xyz Kinematics(1).Payload.V_rpy];

%Arm Variables
for i = 1:length(bot)
    [S1,S2] = size(Kinematics(i).Arm.Velocity);
    qfew(:,:,i) = Kinematics(i).Arm.Angle;
    qdotfew(:,:,i) = ones(S1,S2)*TaskVariables.Velocity_angularlimit;
    qddotfew(:,:,i) =  qdotfew(:,:,i)/TaskVariables.DecelerationTime_xyz;
    Torque_final(:,:,i) = Dynamics(i).Arm.Torque;
    Time_traj = Kinematics(i).time_all;
    MaxDeflection(i) = ArmVariables(i).MaxDeflectionPerLength*ArmVariables(i).TotalArmLength;
    Velocity_angularlimit(i,:) = TaskVariables.Velocity_angularlimit*ones(length(ArmVariables(i).ArmLength),1);
    XGripAlongPayload(i)=ArmVariables(i).PayloadGrip(1);
    YGripAlongPayload(i)=ArmVariables(i).PayloadGrip(2);
    ZGripAlongPayload(i)=ArmVariables(i).PayloadGrip(3);
    theta_grasp(i,:)= ArmVariables(i).theta_grasp_start;
    IsPlanar(i) = ArmVariables(i).IsPlanar;
end

%Task Variables
xyz_task_start = TaskVariables.task_start;
xyz_task_end = TaskVariables.task_end;
tpoints =  TaskVariables.npoints; %I use it so often, Lets make it shorter
DesiredTime = TaskVariables.DesiredTime;
theta_task = TaskVariables.theta_task_start;
theta_task_end =TaskVariables.theta_task_end;
Mpayload = TaskVariables.Mpayload;
Lpayload = TaskVariables.Lpayload;
Ftask_cont = TaskVariables.ForceContinuous;
Pmax = TaskVariables.maxAcceleration_xyz*TaskVariables.Mpayload;
PayloadAcc =TaskVariables.maxAcceleration_xyz;
PayloadMoment =TaskVariables.maxAcceleration_rpy; %actually a torque
%%
Narms = length(bot);
NDOF = length(bot(1).links);
tpoints = 2*TaskVariables.npoints;


TorqueGoingtoMass_all = zeros(Narms,NDOF);
for i = 1:Narms
    for j = 1:tpoints
        Ji(:,:,j,i) = bot(i).jacob0(Kinematics(i).Arm.Angle(j,:)); %world frame
    end
end

%M = [1 1 1 1 1 1];

MassTotalAll = 9E99;
mass_err = 100;
dynamiciteration = 1;

ForceRequiredOnPayload =Mpayload*Aworld'; %yes. tihs is the forcerequired at the end-effector.

%I THINK I CAN JUST DO MY DYNAMICS CONVERGENCE LOOP HERE
if Narms > 1
    %        'optimization at start, with Jacobian in base(world) frame'
    %'Stopping and with arms over 1'
    %Narms
    for i = 1:Narms
        GearRatio(i) = ArmVariables(i).gearratio;
    end
    Me = Mpayload;
    [TotalTorque_optimize,~,ArmForces_optimize,fval,ArmForces_worldFrame] =narmsOptimization_trajectory_givenbot_rev3mass(Tworld,Aworld,Me,qddotfew,Ji,bot,qfew,...
        Ftask_cont,qdotfew,XGripAlongPayload,YGripAlongPayload,ZGripAlongPayload,theta_grasp,Tworld(:,1:3),Tworld(:,4:6),IsPlanar,GearRatio,ArmVariables);
    ArmForces_optimize_ArmFrame = ArmForces_optimize;
    [shouldbeNpoints,shouldbe6] = size(Aworld);
    
    for i  = 1:shouldbeNpoints
        SumForces_EvenlyDividedPerArm_WorldFrame(:,i)  = zeros(1,6);
        SumForces_Optimized_WorldFrame(:,i)  = zeros(1,6);
        GraspingDistance = 0;
        for j = 1:Narms
            GraspingDistance(j) = sqrt(ArmVariables(j).PayloadGrip(1)^2+ ArmVariables(j).PayloadGrip(2)^2+ArmVariables(j).PayloadGrip(3)^2);
            %Transform the results (ArmForces_optimize) from teh arm
            %frame to teh World Frame
            %BUT since I'm looking at the force on the PAYLOAD and not
            %ON THE ARMS. I Need ForcePayload= -Force on the Arms
            F_original = ArmForces_optimize(:,j,i);
            deltaXYZ_original = [ArmVariables(j).PayloadGrip(1),ArmVariables(j).PayloadGrip(2),ArmVariables(j).PayloadGrip(3)];
            deltaRPY_original = theta_grasp(j,:);
            ArmForces_optimize_PayloadFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original,F_original) ;
            
            deltaXYZ_original = Tworld(i,1:3);
            deltaRPY_original = Tworld(i,4:6);
            ArmForces_optimize_WorldFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original,ArmForces_optimize_PayloadFrame(:,j,i)) ;
            
            ArmForces_optimize_WorldFrame(:,j,i)=ArmForces_worldFrame(:,i,j);
            
            %Transform our reference Force from the World Frame to the
            %Tool frames
            F_onPayloadinWorldFrame = -((Mpayload*Aworld(i,:))/Narms)'; %equal and oposite reaction, ACTING on the PAYLAOD
            %  F_actingontheArm = Jei^(-T)*F_e / Narms
            
            %force transformation matrix from teh tool frame to teh base frame
            %this is the transpose of what Craig defines in equation (3)
            T0_e(:,:,i,j) = FindJe_rev2(ArmVariables(j).PayloadGrip(1),ArmVariables(j).PayloadGrip(2),ArmVariables(j).PayloadGrip(3),theta_grasp(j,:),Tworld(i,4:6),Tworld(i,1:3));
            %force transofmration matrix for the Ftask to teh global frame
            T0_p(:,:,i,j) = FindJe_rev2(0,0,0,0*theta_grasp(j,:),Tworld(i,4:6),Tworld(i,1:3));
            ArmForces_EvenlyDividedPerArm_ArmFrame(:,j,i) = T0_e(:,:,i,j)^(-1)* F_onPayloadinWorldFrame;
            
            deltaXYZ_original = -[ArmVariables(j).PayloadGrip(1),ArmVariables(j).PayloadGrip(2),ArmVariables(j).PayloadGrip(3)];
            deltaRPY_original = -theta_grasp(j,:);
            ArmForces_EvenlyDividedPerArm_PayloadFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original, ArmForces_EvenlyDividedPerArm_ArmFrame(:,j,i)) ;
            
            deltaXYZ_original = -Tworld(i,1:3);
            deltaRPY_original = -Tworld(i,4:6);
            ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original,ArmForces_EvenlyDividedPerArm_PayloadFrame(:,j,i)) ;
            
            %Sum forces
            [tau_optimization(i,:,j)] = bot(j).rne(Kinematics(j).Arm.Angle(i,:),  Kinematics(j).Arm.Velocity(i,:),  Kinematics(j).Arm.Acc(i,:),ArmVariables(j).gravity, ArmForces_optimize(:,j,i));
            [tau_evenlydistributed(i,:,j)] = bot(j).rne(Kinematics(j).Arm.Angle(i,:),  Kinematics(j).Arm.Velocity(i,:),  Kinematics(j).Arm.Acc(i,:),ArmVariables(j).gravity,ArmForces_EvenlyDividedPerArm_ArmFrame(:,j,i));
            SumForces_EvenlyDividedPerArm_WorldFrame(:,i)  =ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i)+SumForces_EvenlyDividedPerArm_WorldFrame(:,i) ;
            SumForces_Optimized_WorldFrame(:,i)  = ArmForces_optimize_WorldFrame(:,j,i)+SumForces_Optimized_WorldFrame(:,i);
            
        end
    end
else
    %only 1 arm no need for optimizaiton
    [shouldbeNpoints,~] = size(Aworld);
    for i  = 1:shouldbeNpoints
        j = 1; %because there is only 1 arm but I don't want to go in and change my notation that leads to trouble
        GraspingDistance(j) = sqrt(ArmVariables.PayloadGrip(1)^2+ ArmVariables.PayloadGrip(2)^2+ArmVariables.PayloadGrip(3)^2);
        
        SumForces_EvenlyDividedPerArm_WorldFrame(:,i)  = zeros(1,6);
        SumForces_Optimized_WorldFrame(:,i)  = zeros(1,6);
        
        %put reference force into the Payload Frame
        F_eachArmWorld= -(Mpayload*Aworld(i,:))'; %equal and oposite reaction, CTING on the PAYLAOD
        %  F_actingontheArm = Jei^(-T)*F_e / Narms
        
        %force transformation matrix from teh tool frame to teh base frame
        %this is the transpose of what Craig defines in equation (3)
        T0_e(:,:,i,j) = FindJe_rev2(ArmVariables(j).PayloadGrip(1),ArmVariables(j).PayloadGrip(2),ArmVariables(j).PayloadGrip(3), ArmVariables(j).theta_grasp_start(:),Tworld(i,4:6),Tworld(i,1:3));
        %force transofmration matrix for the Ftask to teh global frame
        T0_p(:,:,i,j) = FindJe_rev2(0,0,0,0*ArmVariables(j).theta_grasp_start,Tworld(i,4:6),Tworld(i,1:3));
        ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i) = T0_e(:,:,i,j)^(-1)* F_eachArmWorld;
        
        deltaXYZ_original = Tworld(i,1:3);
        deltaRPY_original = Tworld(i,4:6);
        ArmForces_optimize_PayloadFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original, ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i)) ;
        ArmForces_EvenlyDividedPerArm_PayloadFrame(:,j,i) = ArmForces_optimize_PayloadFrame(:,j,i);
        
        %Put reference force into the Arm Frame
        deltaXYZ_original = [ArmVariables(j).PayloadGrip(1),ArmVariables(j).PayloadGrip(2),ArmVariables(j).PayloadGrip(3)];
        deltaRPY_original = ArmVariables(j).theta_grasp_start;
        F_final_ArmFrame(:,j,i)=ForceTransformation(deltaXYZ_original,deltaRPY_original,  ArmForces_optimize_PayloadFrame(:,j,i)) ;
        ArmForces_optimize_ArmFrame(:,j,i) =   F_final_ArmFrame(:,j,i);
        ArmForces_EvenlyDividedPerArm_ArmFrame(:,j,i) =   F_final_ArmFrame(:,j,i);
        
        %Sum forces and get torques
        [TotalTorque_optimize(i,:,j)] = bot.rne(Kinematics(j).Arm.Angle(i,:),  Kinematics(j).Arm.Velocity(i,:),  Kinematics(j).Arm.Acc(i,:),ArmVariables(j).gravity',F_final_ArmFrame(:,j,i)); %Need to recalculate since this changes with arm mass
        [tau_evenlydistributed(i,:,j)] = bot.rne(Kinematics(j).Arm.Angle(i,:),  Kinematics(j).Arm.Velocity(i,:),  Kinematics(j).Arm.Acc(i,:),ArmVariables(j).gravity',ArmForces_EvenlyDividedPerArm_ArmFrame(:,j,i));
        
        %clean up
        ArmForces_optimize_WorldFrame(:,j,i) = F_eachArmWorld; %equal and oposite reaction
        ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i) = F_eachArmWorld ; %equal and oposite reaction
        SumForces_EvenlyDividedPerArm_WorldFrame(:,i)  =ArmForces_EvenlyDividedPerArm_WorldFrame(:,j,i)+SumForces_EvenlyDividedPerArm_WorldFrame(:,i) ;
        SumForces_Optimized_WorldFrame(:,i)  = ArmForces_optimize_WorldFrame(:,j,i)+SumForces_Optimized_WorldFrame(:,i);
    end
end
ArmForces_EvenlyDividedPerArm =  ArmForces_EvenlyDividedPerArm_ArmFrame;

maxMaxTorque = 0;
for i = 1:Narms
    MaxTorqueEachArm(i) = max(max(abs(TotalTorque_optimize(:,:,i))));
    if maxMaxTorque <  MaxTorqueEachArm(i)
        maxMaxTorque= MaxTorqueEachArm(i) ;
    end
end

MassTotalAll = 0;
[shouldbetpoints,njoints,shouldbe1] = size(Kinematics(j).Arm.Angle);
[Shouldbetpoints_tt,shouldbenjoints_tt,shouldbenarms] = size(TotalTorque_optimize);
if (Shouldbetpoints_tt ~= shouldbetpoints)
 %   'Need to rotate the TotalTorque_optimize'
    for i = 1:Narms
        TotalTorque_optimize_new(:,:,i) = TotalTorque_optimize(:,:,i)';
    end
    clear TotalTorque_optimize
TotalTorque_optimize = TotalTorque_optimize_new;
end

for i = 1:Narms
    for j = 1:njoints
        MaxTotalTorque_optimize(i,j) = max(abs(TotalTorque_optimize(:,j,i)));
    end
end
%MaxTotalTorque_optimize

SumTorque_optimize = zeros(Narms,tpoints);
SumTorque_nooptimization = SumTorque_optimize;

for j = 1:Narms
    [shouldbetpoints,njoints,shouldbe1] = size(Kinematics(j).Arm.Angle);
    for k = 1:njoints
        for i = 1:tpoints
            SumTorque_optimize(j,i) =  SumTorque_optimize(j,i)+abs(TotalTorque_optimize(i,k,j));
            SumTorque_nooptimization(j,i) = SumTorque_nooptimization(j,i)+abs(tau_evenlydistributed(i,k,j));
        end
    end
end


MaxSumTorque_optimize = zeros(tpoints,1);
MaxSumTorque_nooptimize = MaxSumTorque_optimize;

for i = 1:tpoints
    MaxSumTorque_optimize(i) =  max(SumTorque_optimize(:,i));
    MaxSumTorque_nooptimize(i) =  max(SumTorque_nooptimization(:,i));
end

DynamicResults.WF.ForceRequiredOnPayload = ForceRequiredOnPayload;
DynamicResults.Optimize.JointTorqueAtTimeStep=TotalTorque_optimize;
DynamicResults.Optimize.SumJointTorqueAtTimeStep=SumTorque_optimize;
%DynamicResults.Optimize.MaxJointTorquePerJoint = [max(abs(DynamicResults.Optimize.JointTorqueAtTimeStep(:,1))) max(abs(DynamicResults.Optimize.JointTorqueAtTimeStep(:,2))) max(abs(DynamicResults.Optimize.JointTorqueAtTimeStep(:,3)))];
DynamicResults.Optimize.MaxJointTorquePerJoint = MaxTotalTorque_optimize;
DynamicResults.Optimize.MaxSumJointTorque=MaxSumTorque_optimize;

DynamicResults.Optimize.PF.ForcesOnEachArmTip=ArmForces_optimize_PayloadFrame; %payloadframe
DynamicResults.Optimize.AF.ForcesOnEachArmTip=ArmForces_optimize_ArmFrame;
DynamicResults.Optimize.WF.SumForces=SumForces_Optimized_WorldFrame;
DynamicResults.Optimize.WF.ForcesOnEachArmTip= ArmForces_optimize_WorldFrame;

%DynamicResults.StaticTorque = TorqueStatic;

DynamicResults.EvenlyDistributed.JointTorqueAtTimeStep=tau_evenlydistributed;
DynamicResults.EvenlyDistributed.SumJointTorqueAtTimeStep=SumTorque_nooptimization;
DynamicResults.EvenlyDistributed.MaxSumJointTorque=MaxSumTorque_nooptimize;
DynamicResults.EvenlyDistributed.PF.ForcesOnEachArmTip=ArmForces_EvenlyDividedPerArm_PayloadFrame; %payloadframe
DynamicResults.EvenlyDistributed.AF.ForcesOnEachArmTip=ArmForces_optimize_ArmFrame;
DynamicResults.EvenlyDistributed.WF.SumForces =SumForces_EvenlyDividedPerArm_WorldFrame;
DynamicResults.EvenlyDistributed.WF.ForcesOnEachArmTip= ArmForces_EvenlyDividedPerArm_WorldFrame;
%'end GetTorquetoPaylaodStopping'