function CompiledResults = RunDynamicsOnIndividualAndPair(ArmVariables,TaskVariables,bot,Kinematics,Dynamics,maxPoseErr)
%I want to minimize the total sytsem mass for a pair of arms
%I ALSO want to minimize the maxPoseErr

%So this takes the results from the trajectory and if the arm can reach
%then we run the mass optimizer. That way we only run the pair of arms if
%we have more than one arm that can reach. I relaxed the maxposeError
%criteria for this because I only want to throw out the really bad points-
%the ones not worth calculating.

ploton = 0;
ArmsThatCanReach = 0;
%shouldbe2= length(ArmVariables);
for i = 1:length(ArmVariables)
    if maxPoseErr(i) < ArmVariables(i).TotalArmLength/50
        %Then its good
        ArmsThatCanReach=ArmsThatCanReach+1;
        [DynamicResults,MassResults]=Narm_Ndof_dynamic_Structure_NotIterative(Kinematics(i), Dynamics(i), TaskVariables, ArmVariables(i),ploton,bot(i));
        CompiledResults(ArmsThatCanReach).ArmNumber = i;
        CompiledResults(ArmsThatCanReach).MassResults=MassResults;
        CompiledResults(ArmsThatCanReach).DynamicsResults=DynamicResults;
MassResults_all(i) = MassResults;
MassResults
        clear DynamicResults MassResults
    else
       % 'error pose is too high'
       % maxPoseErr
        Armlength = ArmVariables(i).TotalArmLength;
    end
end

if ArmsThatCanReach > 1
    [DynamicResultsPair,MassResultsPair]=Narm_Ndof_dynamic_Structure_NotIterative(Kinematics, Dynamics, TaskVariables, ArmVariables,ploton,[bot(1) bot(2)]);
    CompiledResults(3).ArmNumber = 3;
    CompiledResults(3).MassResults=MassResultsPair;
    CompiledResults(3).DynamicsResults=DynamicResultsPair;
end

if ArmsThatCanReach == 0
    %no arms can reach
    CompiledResults.ArmNumber = 0;
end

'endcode'