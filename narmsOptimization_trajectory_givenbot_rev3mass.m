function [Tcalc0,DiTi0,ForceOnEachArm_ArmEEframe,fval_all,ForceOnEachArm_WorldFrame] = narmsOptimization_trajectory_givenbot_rev3mass(Xe,Xeddot,Me,qddot_all,Ji_base,...
    bot_all,JointAngles_traj,Ftask,qdot_all,grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z,...
    theta_grasp,Payload_XYZ,Payload_RPY,IsPlanar,gearratio,ArmVariables)




%Inputs:
%Xe - XYZ coordinate of the payload in the global frame
%Xeddot - acceleration of the payload in the global frame
%Me - mass inertia matrix of the paylaod
%qddot - joint acceleration in the base frame (??)
%Ji = jacobian in the tool frame
%bot_all - all the bots
%Joint angles_traj - Joint angles
%Ftask - Task force in teh payload frame
%qdot_all - joint velocity
%grippingalongPayloadLength - grasping points in the PAYLOAD frame
%Payload_xyz - payload in the global frame
%Payload_RPY- angle of payload in teh global frame
%Is Planar - if its planar than I don't want the optimizer to put in any forces in the Fz, Tx, Tz places, 1 = YES

%Outputs:
%Tcalc - motor torques for each arm
%DiTi - should be the same for all arms
%X_all - force on each arm in the tool frames
%fval_all - a measure of how well the optimization worked

%%
%cost function: minimize Ta'WaTa+Tb'WbTb
%u = [Ta Tb 0] %joint torques
%J = [Ja Jb 0] %jacobians in the tool frame
%Te = [T0a T0b 0] %force transofmration matrices from grasping point ot
%global frame
%x  = [Fa Fb 0] Optimized force on each arm due to the payload, in the
%respective tool frames

%constraint function(s)
%MeXeddot +ge +Jea'Fa+Jeb'Fb+J*Ftask  = 0;
%MaQaddot +ga-Ja'Fa = Ta
%MbQbddot + gb-Jb'Fb = Tb

%ignore coriolis on the payload for now because thats not in a rotating
%frame so there shouldn't be any... now gravity loads... nooooope
%So I need to calculate J, D, and g for the manipulators

%I need to ensure everythign is in the same frame.
%make EVERYTHING global at this point

%%
%get the force transformation matrix for the forces 'caused by the arms to
%global
[Tpoints,Njoints,Narms] = size(JointAngles_traj);
PlanarConstraint=zeros(1,6*Narms);
clear Ji
for j = 1:Narms
    if IsPlanar == 1
        PlanarConstraint(1,(j-1)*6+3) = 1;
        PlanarConstraint(1,(j-1)*6+4) = 1;
        PlanarConstraint(1,(j-1)*6+5) = 1;
    end
    
    for i = 1:Tpoints
        %THIS IS TO THE BASE FRAME OF EACH ROBOT/GLOBAL
        clear j_trans0 j_rot0
        Angle = JointAngles_traj(i,:,j);
        j_trans0 = bot_all(j).jacob0(Angle, 'trans');
        j_rot0 =  bot_all(j).jacob0(Angle, 'rot');
        Ji0(:,:,i,j) = [j_trans0; j_rot0];
        
        %THIS IS IN THE END_EFFECTOR FRAME
        j_transn = bot_all(j).jacobn(Angle, 'trans');
        j_rotn =  bot_all(j).jacobn(Angle, 'rot');
        JiEE(:,:,i,j) = [j_transn; j_rotn];
        
        ForceTransformationToPayloadFrame(:,:,i,j) = FindJe_rev2(grippingpointalongPayloadLength_x(j),grippingpointalongPayloadLength_y(j),grippingpointalongPayloadLength_z(j), theta_grasp(j,:),Payload_RPY(i,:),Payload_XYZ(i,:)*0);
        ForceTransformationToGlobalFrame(:,:,i,j) = FindJe_rev2(grippingpointalongPayloadLength_x(j),grippingpointalongPayloadLength_y(j),grippingpointalongPayloadLength_z(j), theta_grasp(j,:)+Payload_RPY(i,:),Payload_RPY(i,:),Payload_XYZ(i,:)*0);
        ForceTransformationToGlobalFrame2(:,:,i,j) = FindJe_rev2(grippingpointalongPayloadLength_x(j),grippingpointalongPayloadLength_y(j),grippingpointalongPayloadLength_z(j), theta_grasp(j,:)+Payload_RPY(i,:),Payload_RPY(i,:),Payload_XYZ(i,:));
        ForceTransformationArmToPayloadGlobalFrame(:,:,i,j) = FindJe_rev2(grippingpointalongPayloadLength_x(j),grippingpointalongPayloadLength_y(j),grippingpointalongPayloadLength_z(j),0*theta_grasp(j,:),0*Payload_RPY(i,:),0*Payload_XYZ(i,:)*0);
     
    end
end

%force transofmration matrix for the Ftask to teh global frame
%I believe the theta_grasp here is just to get hte dimensions right
for i = 1:Tpoints
    ForceTransformationFromPayloadToWorld(:,:,i) = FindJe_rev2(0,0,0,Payload_RPY(i,:),0*Payload_RPY(i,:),Payload_XYZ(i,:)*0);
    ForceTransformationFromPayloadToWorld2(:,:,i) = FindJe_rev2(0,0,0,Payload_RPY(i,:),0*Payload_RPY(i,:),Payload_XYZ(i,:));

    %ForceOnpayloadInGlobalFrame(:,i) = Me*Xeddot(i,:,:);
    Ftask_world(:,i) =  ForceTransformationFromPayloadToWorld(:,:,i)*Ftask;
    ForceOnpayloadInGlobalFrame(:,i) = -Me*Xeddot(i,:,:)+ Ftask_world(:,i)' ;
    ForceOnpayloadInPayloadFrame(:,i) =ForceTransformationFromPayloadToWorld(:,:,i)* ForceOnpayloadInGlobalFrame(:,i);
end    

Tcalc = zeros(Njoints,Tpoints,Narms);
Ji_all = zeros(6*Narms,Njoints,Tpoints);
shouldbe6 = 6;


clear MassInertia CorGrav
for j = 1:Narms
    for i = 1:Tpoints 
        %Do for each point in the trajectory
        %These are in terms of EE to Payload/Global
        ForceTransformationToPayloadFrame_all(:,(shouldbe6*(j-1)+1):shouldbe6*j,i) =ForceTransformationToPayloadFrame(:,:,i,j);
        ForceTransformationToGlobalFrame_all(:,(shouldbe6*(j-1)+1):shouldbe6*j,i) =ForceTransformationToGlobalFrame(:,:,i,j);
        ForceTransformationArmToPayloadGlobalFrame_all(:,(shouldbe6*(j-1)+1):shouldbe6*j,i) =ForceTransformationArmToPayloadGlobalFrame(:,:,i,j);
        Ji0_all((shouldbe6*(j-1)+1):shouldbe6*j,:,i) = Ji0(:,:,i,j);
        JiEE_all((shouldbe6*(j-1)+1):shouldbe6*j,:,i) = JiEE(:,:,i,j);
        %ForceTransformationToPayloadFrame(:,:,1,1)*F is the FORCE ON THE
        %ARM TIP. This is the same as
        %pinv(ForceTransformationToPayloadFrame(:,:,1,1)). Which makes
        %sense since I clacuate the TRANSPOSE OF CRAIGS. 
        %JEA^-T = pinv(ForceTransformationToPayloadFrame(:,:,1,1))<--- may or
        %maynot be hte invrese here. Need a more complicated example to
        %double check
        %JEA = ForceTransformationToPayloadFrame(:,:,1,1)'
        Di_all0(:,:,i,j) =pinv(ForceTransformationToGlobalFrame(:,:,i,j)')*JiEE(:,:,i,j); %need to transpose it because when I calcualte it I give the transpoze
        Di_allEE(:,:,i,j) =pinv(ForceTransformationToGlobalFrame(:,:,i,j)')*JiEE(:,:,i,j); %need to transpose it because when I calcualte it I give the transpoze
        Di_all0ArmthenP(:,:,i,j) =pinv(ForceTransformationToGlobalFrame(:,:,i,j)')*JiEE(:,:,i,j); %need to transpose it because when I calcualte it I give the transpoze
        
        if IsPlanar == 1
            Di_all0planar(:,:,i,j) =([Di_all0(1,:,i,j); Di_all0(2,:,i,j); Di_all0(6,:,i,j)]);
            Di_all0planar_inversetranspose(:,:,i,j)= (Di_all0planar(:,:,i,j)')^(-1);
           % Di_all0ArmthenPlanar_inversetranspose(:,:,i,j) =([Di_all0ArmthenP(1,:,i,j); Di_all0ArmthenP(2,:,i,j); Di_all0ArmthenP(6,:,i,j)]^(-1))';
           % Di_allLPlanar((j-1)*3+1:3*j,(j-1)*3+1:3*j)= [Di_all0(1,:,i,j);Di_all0(2,:,i,j);Di_all0(6,:,i,j)];
            Di_all0ArmthenPlanar_inversetranspose(:,:,i,j) =  (Di_all0planar(:,:,i,j)')^(-1);
        else
            Di_all0planar_inversetranspose(:,:,i,j)= (Di_all0(:,:,i,j)')^(-1);
            % Di_all0ArmthenPlanar_inversetranspose(:,:,i,j)=Di_all0ArmthenP(:,:,i,j)^(-1);
            Di_all0ArmthenPlanar_inversetranspose(:,:,i,j) = Di_all0planar_inversetranspose(:,:,i,j);   
            Di_allLPlanar = Di_all0;
        end
        
        %This Di is all in teh robots base frame which is the same as the
        %global frame. AT THE MOMENT.
        
        Cor =(bot_all(j).coriolis(JointAngles_traj(i,:,j),qdot_all(i,:,j)))*qdot_all(i,:,j)';
        %The product C*qd is the vector of joint force/torque due to velocity coupling
        GravLoad = bot_all(j).gravload(JointAngles_traj(i,:,j))';
        CorGrav(:,j) = Cor+GravLoad;
        CorGravall(i,:,j)= CorGrav(:,j);
        qtemp = JointAngles_traj(i,:,j)';
        MassInertia(:,:,j,i) = bot_all(j).inertia(JointAngles_traj(i,:,j));
    end
end


%Payload_XYZ = Kinematics(1).Payload.T_xyz;
if isnan(MassInertia) == 0 
    [Tcalc0,MassPerJoint,MassPerArm,M_All,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformForceDistributionOptimizationHeader(Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,MassInertia,gearratio,...
        ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_all0,Di_all0ArmthenPlanar_inversetranspose,qdot_all,Payload_XYZ);
else
    'error with mass inertia, resetting the Mass inertia??'
    for j = 1:Narms
        for i = 1:length(bot_all(j).links)
            %  bot_all(j).links(i).m
            bot_all(j).links(i).Jm = 0;
            %           bot_all(j).links(i).m = 0;
        end
    end
    for j = 1:Narms
        for i = 1:Tpoints
            MassInertia(:,:,j,i) = bot_all(j).inertia(JointAngles_traj(i,:,j));
            
            Cor =(bot_all(j).coriolis(JointAngles_traj(i,:,j),qdot_all(i,:,j)))*qdot_all(i,:,j)';
            GravLoad = bot_all(j).gravload(JointAngles_traj(i,:,j))';
            CorGrav(:,j) = Cor+GravLoad;
            CorGravall(i,:,j)= CorGrav(:,j);            
        end
    end
  %  ForceOnpayloadInGlobalFrame
 
   [Tcalc0,MassPerJoint,MassPerArm,M_All,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformForceDistributionOptimizationHeader(Narms,Tpoints,Njoints,JiEE,qddot_all,CorGravall,MassInertia,gearratio,...
        ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_all0,Di_all0ArmthenPlanar_inversetranspose,qdot_all,Payload_XYZ);
end

fval_all=0;


for i = 1:Tpoints
    for j = 1:Narms
        if Narms == 1
                     
        DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
        %need to transform to world frame which is just a translation
        ForceOnEachArm_WorldFrame(:,i,j) =ForceTransformation(ArmVariables(j).xyz_base,[0 0 0],ForceOnEachArm_baseframe(:,i));
        ForceOnEachArm_WorldFrame2(:,j,i) = ForceOnEachArm_WorldFrame(:,i,j);
        else
        DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
        %need to transform to world frame which is just a translation
        ForceOnEachArm_WorldFrame(:,i,j) =ForceTransformation(ArmVariables(j).xyz_base,[0 0 0],ForceOnEachArm_baseframe(:,j,i));
        ForceOnEachArm_WorldFrame2(:,j,i) = ForceOnEachArm_WorldFrame(:,i,j);
        end
    end
end

% %set up the optimization
% %X is the force on each arm in the global frame AT THE EE
% A= [];
% b = [];
% lb = [];
% ub = [];
% Aeq = [];
% beq = [];
% isGA = 0;
% x0 = rand*ones(Narms*6*Tpoints,1)*100*0;
% %in a planar case there should be nothing in the Fz
% x0(3) = 0;
% x0(4)=0;
% x0(5)=0;
% options = optimoptions(@fmincon,'Display','none','Algorithm','sqp','MaxFunctionEvaluations',200000,'StepTolerance',1e-15,'TolCon', 1e-3, 'TolFun', 1e-3,'MaxIterations',1200);
%
% %
% % Qddot(:,:) = qddot_all(i,:,:);
% % Qdot(:,:) = qdot_all(i,:,:);
% % Jtemp0(:,:,:)= Ji0(:,:,i,:);
% % JtempEE(:,:,:)= JiEE(:,:,i,:);
% clear f_mincon nonlcon_mincon


% if max(MassInertia < 9E25)
% %     f_mincon = @(x)narm_objectivefunctionForce_PieceWise_trajAllPoints(x,Ji0,qddot_all,CorGravall,MassInertia,Narms,Njoints,Tpoints,gearratio);
% %     %f_mincon = @(x)narm_objectivefunctionForce_PieceWise_traj(x,Jtemp0,Qddot,CorGrav,MassInertia,Narms,Njoints);
% %
% %     T0_etemp(:,:,:) = ForceTransformationToGlobalFrame_all(:,:,i,:);
% %     nonlcon_mincon=  @(x)narm_constraintfunction_PieceWise_trajAllPoints(x,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,PlanarConstraint,Narms,Njoints,Tpoints);
% %     [x,fval_all, exitflag, output] = fmincon(f_mincon,x0,A,b,Aeq,beq,lb,ub,nonlcon_mincon,options);
% %     output
% %     [c,ceq] = narm_constraintfunction_PieceWise_trajAllPoints(x,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,PlanarConstraint,Narms,Njoints,Tpoints); % Check the constraint values at x
% %     [C,M_all,T_all] = narm_objectivefunctionForce_PieceWise_trajAllPoints(x,Ji0,qddot_all,CorGravall,MassInertia,Narms,Njoints,Tpoints,gearratio);
% [T_All,MassPerTime,MassPerArm,M_All]=PerformForceDistributionOptimizationHeader(choose,Narms,Tpoints,Njoints,Ji0,qddot_all,CorGravall,MassInertia,GearRatio,...
%     ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_all0,Di_all0ArmthenPlanar_inversetranspose)
%
% else
%    % x = zeros(Narms*6,1);
%     fval_all = 0;
% end

%x_all(:,i) = x;
% %fval_all(i) = fval;
% for i = 1:Tpoints
%     for j = 1:Narms
%         %FIND THE NUMBER OF JOINTS SO WE CAN SPERATE THE TORQUE VALUES PER ARM
%         
%         DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
%         
%         TotalPointsPerTime = Narms*6;
%         Timestep = i;
%         StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
%         EndPoint = StartPoint+6*Narms-1;
%         xpertime = x(StartPoint:EndPoint);
%         
%         Qddot(:,:) = qddot_all(i,:,:);
%         Qdot(:,:) = qdot_all(i,:,:);
%         Jtemp0(:,:,:)= Ji0(:,:,i,:);
%         JtempEE(:,:,:)= JiEE(:,:,i,:);
%         
%         for j = 1:Narms
%             TotalPointsPerTime = Narms*6;
%             Timestep = i;
%             StartPoint = (j-1)*6+(i-1)*TotalPointsPerTime+1;
%             EndPoint = StartPoint+6-1;
%             Qd1=Qdot(:,j);
%             Qdd1=Qddot(:,j);
%             grav = [0 0 0];
%             xterm = x(StartPoint:EndPoint);
%             ForceOnEachArm(:,j,i) = xterm; %In PAYLOAD FRAME
%             Jangles = JointAngles_traj(i,:,j);
%             
%             %So this works when the jacobian is in the tool frame, X should be in tool frame
%             Jrecalc0 = bot_all(j).jacob0(Jangles);
%             %Ji0(:,:,i,j)
%             JrecalcEE = bot_all(j).jacobn(Jangles);
%             
%             %WhY is my Ji reporting the same as Jacobn, it only does when the
%             %angle is 0 between EE
%             
%             term1 =  MassInertia(:,:,j)*Qddot(:,j);
%             term2 = CorGravall(i,:,j)';
%             term3 = Jrecalc0'*xterm;
%             
%             %M*qddot+g-J'F = T
%             Tcalc0(:,i,j) =MassInertia(:,:,j)*Qddot(:,j)+CorGrav(:,j)-Jrecalc0'*xterm;%LinkMasses(:,:,j)*Qddot(:,j)+g_all(:,:,j)-Jtemp(:,:,j)'*x_all((j-1)*6+1:6*j);
%             %Tcalc0
%             
%             
%             %So I'm pretty sure this xterm is the problem. Its global here. Which works well when I calculate it by hand (??)
%             %so... lets multiply it by teh Jrecalc which should take the xterm
%             %and put it in the... arm base frame. I think. Yes. because its
%             %right there. So maybe thats my issue. The bot.rne assumes the force
%             %vector is in the base frame and not the... end-effector frame or global frame?
%             %Dman it. I lost it. am I in global, or end-effector... I just found Jei in terms of global so xterm should be global.
%             %So if I'm in global.. I need eeew this is dumb so I need to convert
%             %that to the end-effector frame
%             % [Tarm_global, Rarm_global]=  FindGlobalGrippingCoordinates(Payload_XYZ,...
%             %     Payload_RPY, grippingpointalongPayloadLength_x(j),grippingpointalongPayloadLength_y(j),grippingpointalongPayloadLength_z(j), theta_grasp(j,:));
%             
%             %so Rarm_global
%             %xarm = Rarm_global^-1*xterm(1:3);
%             %xarm(4:6) = Rarm_global^-1*xterm(4:6);
%             
%             %T(:,i,j) = bot_all(j).rne(Jangles, Qdot(:,j)',Qddot(:,j)',grav,-1*xterm);
%             %Ttest(:,i,j) = bot_all(j).rne(Jangles, Qdot(:,j)',Qddot(:,j)',grav,-1*xarm);
%             %DiTi_old(:,i,j) =  Di_all(:,:,i,j)*Ttest(:,i,j);
%             DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
%             %I need to figure out what I'm getting different results from these
%             %too torque calculations...
%             ForceOnEachArm_WorldFrame(:,i,j)= xterm;
%             ForceOnEachArm_WorldFrame2(:,j,i) = xterm;
%         end
%     end
%     
%     %ForceOnEachArm_WorldFrame2=ForceOnEachArm_WorldFrame;
%     for i = 1:Tpoints
%         TotalPointsPerTime = Narms*6;
%         Timestep = i;
%         StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
%         EndPoint = StartPoint+6*Narms-1;
%         xpertime = x(StartPoint:EndPoint);
%         verifyFtask(:,i) = -ForceTransformationToGlobalFrame_all(:,:,i)*xpertime; %should be the same as ForceOnpayloadInGlobalFrame
%     end
%     
%     CA = 1.0042;
%     Ck = .997938;
%     Cb = .361785;
%     MassTotal = 0;
%     MassPerTime = zeros(1,Tpoints);
%     TorquePerTime = zeros(Tpoints,1);
%     for i = 1:Tpoints
%         for j = 1:Narms
%             for k = 1:Njoints
%                 MassAll(k,i,j) = CA*(abs(Tcalc0(k,i,j))^Ck)+Cb;
%                 MassPerTime(1,i) = MassPerTime(1,i)+CA*(abs(Tcalc0(k,i,j)/gearratio(j))^Ck)+Cb;
%                 TorquePerTime(i) = TorquePerTime(i)+abs(Tcalc0(k,i,j));
%             end
%         end
%     end
%     
%     %MassPerTime
%     [EndMass,IndexMass] = max(MassPerTime);
%     %ForceOnEachArm_WorldFrame
%     %ForceOnpayloadInGlobalFrame
%     %Tcalc0
%     %verifyFtask
%     %DiTi0
%     
%     % %LETS PLOT THE RESUTLS
%     % figure
%     % subplot(2,1,1)
%     % plot(1:Tpoints,MassPerTime,'-o')
%     % hold all
%     % for i = 1:Njoints
%     %     for j=1:Narms
%     %         if j ==1
%     %             plot(1:Tpoints,MassAll(i,:,j),'-s')
%     %         else
%     %             plot(1:Tpoints,MassAll(i,:,j),'-p')
%     %         end
%     %     end
%     % end
%     % legend('Total Mass','Arm1, Joint1')
%     %
%     % subplot(2,1,2)
%     % plot(1:Tpoints,TorquePerTime,'-o')
%     % hold all
%     % for i = 1:Njoints
%     %     for j=1:Narms
%     %         if j ==1
%     %             plot(1:Tpoints,abs(Tcalc0(i,:,j)),'-s')
%     %         else
%     %             plot(1:Tpoints,abs(Tcalc0(i,:,j)),'-p')
%     %         end
%     %     end
%     % end
%     % legend('Total Torque','Arm1, Joint1')
%     
%     %ForceOnEachArm
%     %ForceOnEachArm_WorldFrame