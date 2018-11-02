
function [Dynamics,Kinematics,bot_1,maxPoseError,ComplexTrajResults] = AnalysisNarms_OnlyTraj(TaskVariables,ArmVariables,bot_1,isfirstsegment)
%Goal: Produces the dynamics (forces and torques) or the arm as it moves
%through a trajectory

%Divide this up into a single Arm analysis because when I'm keeping 1 arm
%the same I dont need to do this mulitple times
%MassFinal_arm = zeros(1,7,3,1);
%%
%Step 1: Make arm- MOVED TO LOAD VARIABLES
%stationary
% [bot_1] = GenerateBot_singleArm(ArmVariables.ArmLength,ArmVariables.IsPlanar);
% bot_1.base = [eye(3,3) ArmVariables.xyz_base';0 0 0 1];
% bot_1.qlim = ArmVariables.JointLimits;


%%
%'Find trajectory for arm 1'
[Dynamics, Kinematics,maxPoseError,ComplexTrajResults]=TrapezoidalTrajectory_NoPowerCartesian(TaskVariables,ArmVariables,bot_1,isfirstsegment);

