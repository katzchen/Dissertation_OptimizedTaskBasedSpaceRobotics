function [FinalResults,HowMany, CompiledResults_onearm,CompiledResults_2arm,CompiledResults_botharm,Manip1,Manip2,maxAccelerationJoint,maxVelocityJoint,MaxPosePercent] =GA_dualarm_changetask_checkManip(StartAngle,EndAngle,StartPoint,EndPoint,AllVariables,armnumber,allowsinglearmanswers)
format compact
% %OUTPUT IS CompiledResults--------------------------------------
% CompiledResults(ArmsThatCanReach).ArmNumber = i;
% CompiledResults(ArmsThatCanReach).MassResults=MassResults;
% CompiledResults(ArmsThatCanReach).DynamicsResults=Dynamics;
%CompiledResults(1).JointAngle = Kinematics(1).Arm.Angle
%   MassResults.SystemTotal = MassTotalAll;
%   MassResults.LinkMass = LinkMass;
%   MassResults.MotorMass = MotorMass;
%   MassResults.MassperArm = MassTotal_arm;
%   MassResults.Joint.OuterDiameter = Diameter_Out_Joint;
%   MassResults.Joint.InnerDiameter = Diameter_In_Joint;s
%   MassResults.Link.OuterDiameter = Diameter_Out_Link;
%   MassResults.Link.InnerDiameter = Diameter_In_Link;
%   DynamicResults.WF.ForceRequiredOnPayload = ForceRequiredOnPayload;
%   DynamicResults.Optimize.JointTorqueAtTimeStep=TotalTorque_optimize;
%   DynamicResults.Optimize.SumJointTorqueAtTimeStep=SumTorque_optimize;
%   DynamicResults.Optimize.MaxSumJointTorque=MaxSumTorque_optimize;
%   DynamicResults.Optimize.PF.ForcesOnEachArmTip=ArmForces_optimize_PayloadFrame; %payloadframe
%   DynamicResults.Optimize.AF.ForcesOnEachArmTip=ArmForces_optimize_ArmFrame;
%   DynamicResults.Optimize.WF.SumForces=SumForces_Optimized_WorldFrame;
%   DynamicResults.Optimize.WF.ForcesOnEachArmTip= ArmForces_optimize_WorldFrame;
%   DynamicResults.Optimize.MaxJointTorquePerJoint
%   DynamicResults.StaticTorque = TorqueStatic;
%   DynamicResults.EvenlyDistributed.JointTorqueAtTimeStep=tau_evenlydistributed;
%   DynamicResults.EvenlyDistributed.SumJointTorqueAtTimeStep=SumTorque_nooptimization;
%   DynamicResults.EvenlyDistributed.MaxSumJointTorque=MaxSumTorque_nooptimize;
%   DynamicResults.EvenlyDistributed.PF.ForcesOnEachArmTip=ArmForces_EvenlyDividedPerArm_PayloadFrame; %payloadframe
%   DynamicResults.EvenlyDistributed.AF.ForcesOnEachArmTip=ArmForces_optimize_ArmFrame;
%   DynamicResults.EvenlyDistributed.WF.SumForces =SumForces_EvenlyDividedPerArm_WorldFrame;
%   DynamicResults.EvenlyDistributed.WF.ForcesOnEachArmTip= ArmForces_EvenlyDividedPerArm_WorldFrame;


%assign the link lengths 'x' to the Arm variables
[AllVariables]= ChangeStartingTaskPoint(AllVariables,StartPoint,StartAngle);
[AllVariables]= ChangeEndTaskPoint(AllVariables,EndPoint,EndAngle);


[maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);

% for i = 1:length(maxPoseErr)
%     for k = 1:length(Kinematics(i).Arm.Angle(1,:))
%         for j = 1:length(Kinematics(i).Arm.Angle(:,1))
%             while (Kinematics(i).Arm.Angle(j,k)<0)
%                 Kinematics(i).Arm.Angle(j,k) = Kinematics(i).Arm.Angle(j,k)+2*pi;
%             end
%
%             while (Kinematics(i).Arm.Angle(j,k)>2*pi)
%                 Kinematics(i).Arm.Angle(j,k) = Kinematics(i).Arm.Angle(j,k)-2*pi;
%             end
%
%         end
%     end
% end
%
maxAccelerationJoint = max(abs(Kinematics(1).Arm(1).Acc));
maxVelocityJoint =  max(abs(Kinematics(1).Arm(1).Velocity));

for i = 1:length(AllVariables.Arm)
    MaxPosePercent(i) =( maxPoseErr(i)/AllVariables.Arm(i).TotalArmLength)*100;
end


%MaxPosePercent
Manip1T = [NaN NaN];
Manip1R = [NaN NaN];
Manip1TA = [NaN NaN];
Manip1RA = [NaN NaN];
Manip2 = [inf 0];
Manip1 = [ Manip1T(1,:)  Manip1R(1,:) Manip1TA(1,:)  Manip1RA(1,:)];
MaxPosePercent
maxPoseErr
allowsinglearmanswers
if min(MaxPosePercent) > 5
    %  'Error is too high for BOTH arms'
    %   MaxPosePercent
    CompiledResults(1).ArmNumber = 0;
else
    CompiledResults = RunDynamicsOnIndividualAndPairNoFilter(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,allowsinglearmanswers);
    CompiledResults(1).JointAngle = Kinematics(1).Arm.Angle;
    clear ManipT ManipR ManipTA ManipRA
    
end

FinalResults=inf;
if (CompiledResults(1).ArmNumber == 0 )
    %no arms could reach
    'No arms could reach GA singlearm Line 54'
    FinalResults=NaN;
elseif length(CompiledResults) == 1
    if(allowsinglearmanswers==1)
        'Only single arm answer'
        FinalResults=NaN;
    else
        %only 1 arm could reach
        'one arm could reach'
        FinalResults = [CompiledResults(1).MassResults.SystemTotal];% CompiledResults(1).MassResults.SystemTotal 0];
    end
    for i = 1:length(Kinematics(1).time_all)
        if (max(isnan(Kinematics(1).Arm.Angle(i,:)))<1 | max(isinf(Kinematics(1).Arm.Angle(i,:)))<1)
            ManipTA(i) =  bot(1).maniplty(Kinematics(1).Arm.Angle(i,:),'asada','T');
            ManipRA(i) =  bot(1).maniplty(Kinematics(1).Arm.Angle(i,:),'asada','R');
            ManipT(i) =  bot(1).maniplty(Kinematics(1).Arm.Angle(i,:),'T');
            ManipR(i) =  bot(1).maniplty(Kinematics(1).Arm.Angle(i,:),'R');
        else
            ManipTA(i) = 0;
            ManipRA(i) = 0;
            ManipT(i) = 0;
            ManipR(i) = 0;
        end
    end
    Manip1T(1,:) = [min(ManipT) max(ManipT)];
    Manip1R(1,:) = [min(ManipR) max(ManipR)];
    Manip1TA(1,:) = [min(ManipTA) max(ManipTA)];
    Manip1RA(1,:) = [min(ManipRA) max(ManipRA)];
    Manip1 = [ Manip1T(1,:)  Manip1R(1,:)  Manip1TA(1,:)  Manip1RA(1,:)];
else
    %both arms fit
    'both arms could reach'
    FinalResults =[CompiledResults(3).MassResults.SystemTotal];% CompiledResults(3).MassResults.MassperArm ];
    CompiledResults(2).JointAngle = Kinematics(2).Arm.Angle;
    clear Manip
    for i = 1:length(Kinematics(2).time_all)
        Manip(i) =  bot(2).maniplty(Kinematics(2).Arm.Angle(i,:));
    end
    Manip2(1,:) = [min(Manip) max(Manip)];
end

%This is just garbage
if FinalResults > 5*10^15
    %  'Somehow ended up with terrible mass results'
    CompiledResults(1).ArmNumber= 0 ;
    
    FinalResults=NaN;
end

%results=[x FinalResults]
%FinalResults
%MaxPosePercent
%FinalResults



CompiledResults_onearm = inf;
CompiledResults_2arm= inf;
CompiledResults_botharm = inf;

if (allowsinglearmanswers == 1)
    %only allow dual
    
    CompiledResults_botharm = CompiledResults(3).MassResults.SystemTotal;
    HowMany=2;
else
    
    if (CompiledResults(1).ArmNumber == 0 )
        HowMany = 0;
        
    elseif length(CompiledResults) == 1
        HowMany = 1;
        if CompiledResults(1).ArmNumber==1
            CompiledResults_onearm = CompiledResults(1).MassResults.SystemTotal;
        else% CompiledResults(1).ArmNumber==2
            CompiledResults_2arm = CompiledResults(1).MassResults.SystemTotal;
        end
    else
        HowMany= 2;
        CompiledResults_onearm = CompiledResults(1).MassResults.SystemTotal;
        CompiledResults_2arm= CompiledResults(2).MassResults.SystemTotal;
        CompiledResults_botharm = CompiledResults(3).MassResults.SystemTotal;
    end
end
%FinalResults
%HowMany
%CompiledResults_onearm
%CompiledResults_2arm
%CompiledResults_botharm

%'Done with GA_dualarm'

%   graphResults(AllVariables,armnumber)

%Summary = [MaxPosePercent(1) Manip1 FinalResults]