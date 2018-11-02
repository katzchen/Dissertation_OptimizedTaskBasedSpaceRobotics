
function  [maxPoseError,DynamicResults,Kinematics,bot]= GetTrajectoryBothArms(AllVariables)

for i = 1:length(AllVariables.Arm)
    
    [DynamicResults(i),Kinematics(i),bot(i),maxPoseError(i)] = AnalysisNarms_OnlyTraj(AllVariables.Task,AllVariables.Arm(i),AllVariables.bot(i), 1);
end


% if AllVariables.Task.ComplexMotion.Code>0
%     %then I have a complex motion
%     if AllVariables.Task.ComplexMotion.Code ==3
%         %rotating square
%         
%         [ComplexTrajectory.start,ComplexTrajectory.end,...
%             ComplexTrajectory.theta.start,ComplexTrajectory.theta.end,...
%             ComplexTrajectory.Nsegment,ComplexTrajectory.grasp.start, ComplexTrajectory.grasp.end]=Trajectory_RotatingBox(AllVariables);
%     elseif AllVariables.Task.ComplexMotion.Code ==2
%         %square, no rotation
%         [ComplexTrajectory.start,ComplexTrajectory.end,...
%             ComplexTrajectory.theta.start,ComplexTrajectory.theta.end,...
%             ComplexTrajectory.Nsegment]=Trajectory_MovingBox(AllVariables);
%     end
%     
%     %Combine the complex trajectories into the expected otuput
%     notvalidstarting = 0;
%     for i = 1:length(AllVariables.Arm)
%         NDOF = length(AllVariables.Arm(i).ArmLength);
%         pointspersegment =AllVariables.Task.npoints*2;
%         clear NewStartingAngle NewEndAngle maxPoseErrorPiece
%         NewStartingAngle(1,:) =   AllVariables.Arm(i).TrajGenAngle;
%         for j = 1: ComplexTrajectory.Nsegment
%             if j > 1
%                 DesiredStartingAngle =   NewEndAngle(j-1,:);
%             else
%                 DesiredStartingAngle =   NewStartingAngle(j,:) ;
%             end
%             [NewStartingAngle(j,:),NewEndAngle(j,:),ComplexTrajResults_temp,DynamicResultsPiece(j),KinematicPieces(j),maxPoseErrorPiece(j),bot(j)] = calculateSegments(j, i, DesiredStartingAngle,...
%                 ComplexTrajectory, j,AllVariables);
%             ComplexTrajResults(j).notvalidstarting = ComplexTrajResults_temp.notvalidstarting;
%             ComplexTrajResults(j).currentIteration = ComplexTrajResults_temp.currentIteration;
%             
%             %lets check to make sure that if j > 1 I'm not flipping joints
%             if j > 1
%                 if norm(DesiredStartingAngle-NewStartingAngle(j,:)) > 5*pi/180
%                     'Im flipping joints, so I cannot do it with the given starting angle'
%                     %I have options for new starting angles
%                     %Lets take a look at the PRVIOUS SEGMENT
%                     %Lets see if we can go from the PREVIOUS STARTING to
%                     %the NEW ENDING FOR THE Previous Joint.
%                     for k = 1:j-1
%                         %starting with j-1, go backwards
%                         %The DesiredStartingAngle for segment j-1 is the start of segment j
%                         
%                         backwardsj = j-k;  %k =1 => backwardsj = j-1, k = j-1 =>backwardsj 1
%                         DesiredStartingAngle =   NewStartingAngle(backwardsj+1,:);
%                         backwardsj
%                         DesiredStartingAngle
%                         [NewStartingAngleBW(backwardsj,:),NewEndAngleBW(backwardsj,:)] = calculateSegments(backwardsj, i,...
%                             DesiredStartingAngle, ComplexTrajectory,notvalidstarting, AllVariables);
%                         NewStartingAngleBW
%                         NewEndAngleBW
%                         %now I have j segment going backwards. and then j-1
%                         %and then j-2
%                         
%                         if backwardsj < j
%                             if norm(DesiredStartingAngle-NewStartingAngleBW(backwardsj,:)) < 5*pi/180
%                                 %this works, so lets run it forward
%                                 [NewStartingAngle(backwardsj,:),NewEndAngle(backwardsj,:)] = calculateSegments(backwardsj, i,NewEndAngleBW(backwardsj,:), ComplexTrajectory,AllVariables);
%                             else
%                                 '2nd chance is over'
%                             end
%                         else
%                             AllVariables.Arm(i).TrajGenAngle = KinematicPiecesBW(backwardsj).Arm.Angle(length(KinematicPieces(backwardsj).Arm.Angle(:,1)),:); %need to set the starting angle to the end fo the previous segment
%                         end
%                     end %end k
%                 end %end compare starting angels
%             end %end for j > 1
%         end %end for j= 1:Nsegment
%         
%         for j = 1:ComplexTrajectory.Nsegment
%             if j > 1
%                 
%                 DynamicResults(i).Arm.Torque = [DynamicResults(i).Arm.Torque DynamicResultsPiece(j).Arm.Torque];
%                 DynamicResults(i).Arm.CorGrav = [DynamicResults(i).Arm.CorGrav DynamicResultsPiece(j).Arm.CorGrav];
%                 Kinematics(i).Arm.Velocity = [Kinematics(i).Arm.Velocity; KinematicPieces(j).Arm.Velocity];
%                 Kinematics(i).Arm.Angle = [Kinematics(i).Arm.Angle; KinematicPieces(j).Arm.Angle];
%                 Kinematics(i).Arm.Acc = [Kinematics(i).Arm.Acc; KinematicPieces(j).Arm.Acc];
%                 Kinematics(i).Arm.J = [Kinematics(i).Arm.J; KinematicPieces(j).Arm.J];
%                 Kinematics(i).Arm.time = [Kinematics(i).Arm.time KinematicPieces(j).Arm.time];
%                 Kinematics(i).Arm.time_all = [Kinematics(i).Arm.time_all KinematicPieces(j).Arm.time_all+max(Kinematics(i).Arm.time_all)*ones(1,length(KinematicPieces(j).Arm.time_all))];
%                 Kinematics(i).Arm.error= [Kinematics(i).Arm.error; KinematicPieces(j).Arm.error];
%                 Kinematics(i).Payload.A_xyz = [Kinematics(i).Payload.A_xyz; KinematicPieces(j).Payload.A_xyz];
%                 Kinematics(i).Payload.A_rpy = [Kinematics(i).Payload.A_rpy; KinematicPieces(j).Payload.A_rpy];
%                 Kinematics(i).Payload.T_xyz = [Kinematics(i).Payload.T_xyz; KinematicPieces(j).Payload.T_xyz];
%                 Kinematics(i).Payload.T_rpy = [Kinematics(i).Payload.T_rpy; KinematicPieces(j).Payload.T_rpy];
%                 Kinematics(i).Payload.V_xyz = [Kinematics(i).Payload.V_xyz; KinematicPieces(j).Payload.V_xyz];
%                 Kinematics(i).Payload.V_rpy = [Kinematics(i).Payload.V_rpy; KinematicPieces(j).Payload.V_rpy];
%                 maxPoseError(i) = max(maxPoseError(i), maxPoseErrorPiece(j));
%                 
%                 
%             else
%                 DynamicResults(i)= DynamicResultsPiece(j);
%                 Kinematics(i) = KinematicPieces(j);
%                 maxPoseError(i) = maxPoseErrorPiece(j);
%             end
%             
%         end
%         figure
%         hold all
%         for j = 1:length(Kinematics(i).Arm.Angle(:,1))
%             bot(i).plot( Kinematics(i).Arm.Angle(j,:),'top')
%             plotpayload_prism(Kinematics(i).Payload.T_xyz(j,:),Kinematics(i).Payload.T_rpy(j,:), AllVariables.Task.Lpayload,0);
%         end
%     end
% % else
%     for i = 1:length(AllVariables.Arm)
%         [DynamicResults(i),Kinematics(i),bot(i),maxPoseError(i)] = AnalysisNarms_OnlyTraj(AllVariables.Task,AllVariables.Arm(i),1);
%     end
%     
% end
