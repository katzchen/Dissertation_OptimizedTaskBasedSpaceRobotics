function  [FinalResults,HowMany, CompiledResults_onearm,CompiledResults_2arm,CompiledResults_botharm] =GA_singlearm(x,AllVariables,armnumber)

%assign the link lengths 'x' to the Arm variables
 AllVariables = ChangeArmLength_single(x,AllVariables,armnumber);

 [maxPoseErr,Dynamics,Kinematics,bot]= GetTrajectoryBothArms(AllVariables);
 x
 MaxPosePercent = maxPoseErr(armnumber)/AllVariables.Arm(armnumber).TotalArmLength*100;
 MaxPosePercent
 
 if min(MaxPosePercent) > 5
     'Error is too high for BOTH arms'
     CompiledResults(1).ArmNumber = 0;
 else
     CompiledResults = RunDynamicsOnIndividualAndPair(AllVariables.Arm,AllVariables.Task,bot,Kinematics,Dynamics,maxPoseErr);
     CompiledResults(1).JointAngle = Kinematics(1).Arm.Angle;
 end
 
 if (CompiledResults(1).ArmNumber == 0 )
     %no arms could reach
     %'No arms could reach GA singlearm Line 54'
     FinalResults=inf;
 else length(CompiledResults) == 1     
     FinalResults = [CompiledResults(1).MassResults.SystemTotal];% CompiledResults(1).MassResults.SystemTotal 0];
 end

%This is just garbage
if FinalResults > 5*10^15
    'Somehow ended up with terrible mass results'
    CompiledResults(1).ArmNumber= 0 ;
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
% 
% FinalResults
% HowMany
% CompiledResults_onearm
% CompiledResults_2arm
% CompiledResults_botharm 
% 
% 'Done with GA_dualarm'


