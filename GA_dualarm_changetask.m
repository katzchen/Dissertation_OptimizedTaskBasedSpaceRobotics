function [FinalResults,HowMany, CompiledResults_onearm,CompiledResults_2arm,CompiledResults_botharm] =GA_dualarm_changetask(StartAngle,EndAngle,StartPoint,EndPoint,AllVariables,armnumber,allowsinglearmanswers)
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
for i = 1:length(AllVariables.Arm)
    MaxPosePercent(i) = maxPoseErr(i)/AllVariables.Arm(i).TotalArmLength*100;
end

%MaxPosePercent

if min(MaxPosePercent) > 5
  %  'Error is too high for BOTH arms'
 %   MaxPosePercent
    CompiledResults(1).ArmNumber = 0;
else
    CompiledResults = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
    CompiledResults(1).JointAngle = Kinematics(1).Arm.Angle;
end

    FinalResults=NaN;
if (CompiledResults(1).ArmNumber == 0 )
    %no arms could reach
    %'No arms could reach GA singlearm Line 54'
    FinalResults=NaN;
elseif length(CompiledResults) == 1
    if(allowsinglearmanswers==1)
  %      'Only single arm answer'
        FinalResults=NaN;
    else
        %only 1 arm could reach
        % 'one arm could reach'
        FinalResults = [CompiledResults(1).MassResults.SystemTotal];% CompiledResults(1).MassResults.SystemTotal 0];
    end
else
    %both arms fit
    %   'both arms could reach'
    FinalResults =[CompiledResults(3).MassResults.SystemTotal];% CompiledResults(3).MassResults.MassperArm ];
    CompiledResults(2).JointAngle = Kinematics(2).Arm.Angle;
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

%FinalResults
%HowMany
%CompiledResults_onearm
%CompiledResults_2arm
%CompiledResults_botharm

%'Done with GA_dualarm'

Summary = [MaxPosePercent(1) FinalResults]