function [Tcalc,MaxMassPerJoint,MassPerArm,MassSystem,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformLoptimization_choose(Narms,Tpoints,Njoints,Di_all0,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdotfew,PayloadXYZ)


%I have optimized it taking the whole trajectory and individual. I'm going
%to leave the trajectory one up and comment out the other. 

    clear Fall_payload
    iter = 1;
for j = 1:Narms
    for k = 1:Njoints
        for i = 1:Tpoints
            friction(i) = abs(bot_all(j).links(k).friction(qdotfew(i,k,j)));
            iter = iter+1;
        end        
        maxfriction(k,j) = max(friction)        ;
    end
end
maxfriction_all = [maxfriction(:,1); maxfriction(:,2) ];

clear friction_traj
friction_traj = maxfriction_all';
for i = 2:Tpoints
    friction_traj = [friction_traj maxfriction_all'];
end


%qdotfew are the joint velocities, in... radians per second?
%lets convert to RPM - rotations per minute    radians/2*pi = revolutoin
%radians/second*(1rev/2*pi rad)*(60seconds/minute) = rpm

[Ttraj,total,minValue]= LinProgTorque_DynamicsMassSmall_Traj(Tpoints,Narms, Njoints,qddot_all,Di_all0ArmthenPlanar_inversetranspose,ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,friction_traj');
%
if Njoints < 6
    Ndof = 3;
else
    Ndof  = 6;
end


startiter = 1;
for i = 1:Tpoints
    enditer = startiter-1+Njoints*Narms;
    Ttraj_seperate(1:Njoints*Narms,i) = Ttraj(startiter:enditer);
    startiter = enditer+1;
    
    Fall_payload(:,i) =zeros(6,1);
    %I have the torque. Now i wnat to find the Force from the base? EE?
    for j= 1:Narms
        clear Tarm
        if j == 1
            Tarm = Ttraj_seperate(1:Njoints,i);
            ForceTransformationToGlobalFrame=ForceTransformationToGlobalFrame_all(:,1:6,i);
        else
            Tarm = Ttraj_seperate(Njoints+1:Njoints*2,i);
            ForceTransformationToGlobalFrame=ForceTransformationToGlobalFrame_all(:,6+1:6*2,i);
        end
        Tcalc(:,i,j) = Tarm;
        
        term1 = MassInertia(:,:,j,i)*qddot_all(i,:,j)';
        term2 = CorGravall(i,:,j)';
         JTF = Tarm-(term1+term2);
        
        Jangles = JointAngles_traj(i,:,j);
        Jrecalc0 = bot_all(j).jacobn(Jangles);
        if Ndof < 6
            Jrecalc0planar = [Jrecalc0(1,:); Jrecalc0(2,:); Jrecalc0(6,:)];
            clear Jrecalc0
            Jrecalc0=Jrecalc0planar;           
        end
        
        JT = Jrecalc0';
        Ftemp = JT^(-1)*(JTF); %Force at the EE.
        if Ndof < 6
            temp = [Ftemp(1) Ftemp(2) 0 0 0 Ftemp(3)];
            F(:,j,i) = temp'; %Force at the EE.
        else            
            F(:,j,i) = Ftemp; %Force at the EE.            
        end
      % ForceTransformationToGlobalFrame is actually ForceTransformationArmToPayloadGlobalFrame_all
        
        Fall_payload_perarm(:,j,i)= ForceTransformationToGlobalFrame*F(:,j,i);
        Fall_payload(:,i) = ForceTransformationToGlobalFrame*F(:,j,i)+Fall_payload(:,i);
    end
end

Tcalc_traj = Tcalc;
%ForceOnpayloadInGlobalFrame
verifyTask_traj = Fall_payload;

MassPerArmLin = zeros(Narms,Tpoints);
MassPerTimeLin = zeros(Tpoints,1);
MassAllLin = zeros(Tpoints,Njoints,Narms);
MassPerArmLin = zeros(2,Tpoints);
MassPerTimeLin = zeros(Tpoints,1);

 T = [Tcalc_traj(:,:,1); Tcalc_traj(:,:,2)];
 
for i = 1:Tpoints
AngularVelocity = reshape(qdotfew(i,:,:),[Ndof*2,1]);
    for j = 1:size(T(:,i))
        [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(T(j,i)), AngularVelocity(j)*60/(2*pi));
        if j < Ndof+1
            k = 1;
        MassAllLin(i,j,k) = FinalMotorMass+FinalHarmonicMass;
        else
            k = 2;
        MassAllLin(i,j-Ndof,k ) = FinalMotorMass+FinalHarmonicMass;
        end       
    end
end


for k = 1:Narms
    for j= 1:Njoints
        MaxMassPerJoint(j,k) =max(MassAllLin(:,j,k ));
    end
end
MassPerArm =zeros(Narms,1);
for k = 1:Narms
    for j= 1:Njoints
        MassPerArm(k)= MaxMassPerJoint(j,k) +MassPerArm(k);
    end
end
MassSystem = MassPerArm(1)+MassPerArm(2);

ForceOnEachArm_baseframe = Fall_payload_perarm; %this is in the payload frame
% for i = 1:Tpoints
%     for j = 1:Narms
%         Jangles = JointAngles_traj(i,:,j);
%         TEE=  bot_all(j).A(1:length(Jangles),Jangles);
%         ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_baseframe(:,j,i));
%     end
% end
for i = 1:Tpoints
    for j = 1:Narms
        Jangles = JointAngles_traj(i,:,j);
        TEE=  bot_all(j).A(1:length(Jangles),Jangles);
        ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),(TEE(1:3,4)+bot_all(j).base(1:3,4))-PayloadXYZ(i,:)', ForceOnEachArm_baseframe(:,j,i));
    end
end

%ForceOnEachArm_ArmEEframe
%'end linear'


%%
% 
% clear Fall_payload
% for i = 1:Tpoints
%     %  IsPlanar(i) = AllVariables.Arm(i).IsPlanar;
%     iter = 1;
%     for j = 1:Narms
%         for k = 1:Njoints
%             friction(iter) = bot_all(j).links(k).friction(qdotfew(i,k,j));
%             iter = iter+1;
%         end
%     end
%     
%     friction_all(i,:) = friction;
%     [T(:,i),total(i),minValue(i)]= LinProgTorque_DynamicsMassSmall(Narms, Njoints,qdotfew(i,:,:),Di_all0ArmthenPlanar_inversetranspose(:,:,:,i),ForceOnpayloadInGlobalFrame(:,i),MassInertia(:,:,:,i),qddot_all(i,:,:),CorGravall(i,:,:),IsPlanar,maxfriction_all);
%     Fall_payload(:,i) =zeros(6,1);
%     %I have the torque. Now i wnat to find the Force from the base? EE?
%     for j= 1:Narms
%         if j == 1
%             Tarm = T(1:6,i);
%             ForceTransformationToGlobalFrame=ForceTransformationToGlobalFrame_all(:,1:6,i);
%         else
%             Tarm = T(7:12,i);
%             ForceTransformationToGlobalFrame=ForceTransformationToGlobalFrame_all(:,7:12,i);
%         end
%         Tcalc(:,i,j) = Tarm;
%         
%         term1 = MassInertia(:,:,j,i)*qddot_all(i,:,j)';
%         term2 = CorGravall(i,:,j)';
%         JTF = Tarm-(term1+term2);
%         
%         Jangles = JointAngles_traj(i,:,j);
%         Jrecalc0 = bot_all(j).jacob0(Jangles);
%         JT = Jrecalc0';
%         F(:,j,i) = JT^(-1)*(JTF); %Force at the base.
%         Fall_payload(:,i) = -ForceTransformationToGlobalFrame*F(:,j,i)+Fall_payload(:,i);
%     end
% end
% Tcalc_traj
% Tcalc
% verifyTask = Fall_payload;
% verifyTask
% verifyTask_traj
% 
% 
% MassPerArmLin = zeros(Narms,Tpoints);
% MassPerTimeLin = zeros(Tpoints,1);
% MassAllLin = zeros(Tpoints,Njoints,Narms);
% MassPerArmLin = zeros(2,Tpoints);
% MassPerTimeLin = zeros(Tpoints,1);
% 
% for i = 1:Tpoints
% AngularVelocity = reshape(qdotfew(i,:,:),[12,1]);
%     for j = 1:size(T(:,i))
%         [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(T(j,i)), AngularVelocity(j)*60/(2*pi));
%         if j < 7
%             k = 1;
%         MassAllLin(i,j,k) = FinalMotorMass+FinalHarmonicMass;
%         else
%             k = 2;
%         MassAllLin(i,j-6,k ) = FinalMotorMass+FinalHarmonicMass;
%         end       
%     end
% end
% 
% 
% for k = 1:Narms
%     for j= 1:Njoints
%         MaxMassPerJoint(j,k) =max(MassAllLin(:,j,k ));
%     end
% end
% MassPerArm =zeros(Narms,1);
% for k = 1:Narms
%     for j= 1:Njoints
%         MassPerArm(k)= MaxMassPerJoint(j,k) +MassPerArm(k);
%     end
% end
% MassSystem = MassPerArm(1)+MassPerArm(2);
% 
% 
% 
% ForceOnEachArm_baseframe = F;
% for i = 1:Tpoints
%     for j = 1:Narms
%         Jangles = JointAngles_traj(i,:,j);
%         TEE=  bot_all(j).A(1:length(Jangles),Jangles);
%         ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_baseframe(:,j,i));
%     end
% end
% 

%%
%PREVIOUSLY COMMENTED OUT STUFF

    % end
    %
    % %      T
    % %      Torg
    % %      term1a = reshape(term1,[Njoints*Narms 1]);
    % %      term2a = reshape(term2,[Njoints*Narms 1]);
    % %
    % %      %D'Ftask = T-(Term1a+term2a)
    % %      DFtask = T(:,i)-term1a-term2a;
    % %      D = Di_all0ArmthenPlanar_inversetranspose(:,:,:,i);
    % %      Dall = [D(:,:,1) D(:,:,2)]
    % %      Ftask =DFtask
    %
    %
    %
    %
    %
    % if size(Njoints) < Narms
    %     NjointsAll = Njoints*ones(Narms,1);
    % end
    %
    % for i = 1:Tpoints
    %     PreviousEnd = 0;
    %     for j = 1:Narms
    %         StartPoint = PreviousEnd+1;
    %         TotalPointsPerArm = NjointsAll(j);
    %         PreviousEnd = TotalPointsPerArm+StartPoint-1;
    %         Tcalc(:,i,j) = T(StartPoint:PreviousEnd,i);
    %     end
    % end
    %
    %
    %
    % for i = 1:Tpoints
    %     if IsPlanar == 1
    %         DT1 = Di_all0ArmthenPlanar_inversetranspose(:,:,1,i)*T(1:3,i);
    %         DT2 = Di_all0ArmthenPlanar_inversetranspose(:,:,2,i)*T(4:6,i);
    %         SumDT(:,:,i) = DT1+DT2;
    %     else
    %         DT1 = Di_all0ArmthenPlanar_inversetranspose(:,:,1,i)*T(1:6,i);
    %         DT2 = Di_all0ArmthenPlanar_inversetranspose(:,:,2,i)*T(7:12,i);
    %         SumDT(:,:,i) = DT1+DT2;
    %     end
    % end
    % DTorqueLin = zeros(Njoints,Tpoints);
    % linprogcounterStart =-5;
    %
    %
    % for i = 1:Tpoints
    %     Qddot(:,:) = qddot_all(i,:,:);
    %     for j = 1:Narms
    %         Jangles = JointAngles_traj(i,:,j);
    %         TotalPointsPerTime = Narms*6;
    %         CorGrav(:,j) = CorGravall(i,:,j);
    %          %M*qddot+g-J'F = T
    %          %M*qddot+g-T = J'F
    %          JtransposeF = (MassInertia(:,:,j)*Qddot(:,j)+CorGrav(:,j)-Tcalc(:,i,j));
    %          Jrecalc0 = bot_all(j).jacob0(Jangles);
    %          Jrecalc0T = Jrecalc0';
    %          ForceOnTip(:,i,j)= (Jrecalc0T^(-1))*JtransposeF;
    %          if j == 1
    %              ForceOnTip_all(1:6,i)=ForceOnTip(:,i,j);
    %          else
    %              ForceOnTip_all(7:12,i)=ForceOnTip(:,i,j);
    %          end
    %
    %     end
    %
    %      verifyFtask(:,i) = -ForceTransformationToGlobalFrame_all(:,:,i)* ForceOnTip_all(:,i); %should be the same as ForceOnpayloadInGlobalFrame
    % end
    %
    % verifyFtask
    %
    %
    %
    %
    % for i = 1:Tpoints
    %     TotalPointsPerTime = Narms*6;
    %     Timestep = i;
    %     StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
    %     EndPoint = StartPoint+6*Narms-1;
    %     %    xpertime = x(StartPoint:EndPoint);
    %
    %
    %     Qddot(:,:) = qddot_all(i,:,:);
    %     %  Qdot(:,:) = qdot_all(i,:,:);
    %     % Jtemp0(:,:,:)= Ji0(:,:,i,:);
    %     % JtempEE(:,:,:)= JiEE(:,:,i,:);
    %
    %     for j = 1:Narms
    %         Jangles = JointAngles_traj(i,:,j);
    %         TotalPointsPerTime = Narms*6;
    %         CorGrav(:,j) = CorGravall(i,:,j);
    %         % Timestep = i;
    %         %M*qddot+g-J'F = T %so I'min whatever frame teh Mass Inertia and
    %         %Qddot is in... though mostly whatever the J term is in so this is
    %         %in the base frame
    %
    %         %xterm is the force on the tip but should be in the global frame
    %         %  Tcalc0(:,i,j) =MassInertia(:,:,j)*Qddot(:,j)+CorGrav(:,j)-Jrecalc0'*xterm;%LinkMasses(:,:,j)*Qddot(:,j)+g_all(:,:,j)-Jtemp(:,:,j)'*x_all((j-1)*6+1:6*j);
    %         %M*qddot+g-J'F = T
    %         T_time = T(1+(j-1)*Njoints:Njoints+(j-1)*Njoints,i);
    %         Jrecalc0 = bot_all(j).jacob0(Jangles);
    %         Jrecalc0T = Jrecalc0';
    %         Jrecalc0TXterm_lin(:,i,j) = (MassInertia(:,:,j)*Qddot(:,j)+CorGrav(:,j)-T_time); %should be force at the tool
    %         if IsPlanar == 1
    %             Jrecalc0T_planar  =[Jrecalc0T(:,1) Jrecalc0T(:,2) Jrecalc0T(:,6)];
    %             Xterm_lin(:,i,j)= (Jrecalc0T_planar^(-1))* Jrecalc0TXterm_lin(:,i,j) ;
    %             Xterm_linAll(:,i,j) =[ Xterm_lin(1,i,j)  Xterm_lin(2,i,j) 0 0 0  Xterm_lin(3,i,j)];
    %             linprogcounterStart = linprogcounterStart+6;
    %             linprogcounterEnd= linprogcounterStart+5;
    %             Xterm_linall(linprogcounterStart:linprogcounterEnd) = [ Xterm_lin(1,i,j)  Xterm_lin(2,i,j) 0 0 0  Xterm_lin(3,i,j)];
    %         else
    %             Xterm_lin(:,i,j)= (Jrecalc0T^(-1))* Jrecalc0TXterm_lin(:,i,j) ;
    %             Xterm_linAll(:,i,j) = Xterm_lin(:,i,j);
    %             linprogcounterStart = linprogcounterStart+6;
    %             linprogcounterEnd= linprogcounterStart+5;
    %             Xterm_linall(linprogcounterStart:linprogcounterEnd) = [ Xterm_lin(:,i,j)];
    %         end
    %         DTorqueLin(:,i)=DTorqueLin(:,i)+Di_all0ArmthenPlanar_inversetranspose(:,:,j,i)*T_time;
    %
    %      %   Xterm_linAll(:,i,j) =[ Xterm_lin(1,i,j)  Xterm_lin(2,i,j) 0 0 0  Xterm_lin(3,i,j)];
    %         %    SHOULDBEFTASKLIN = ForceTransformationToGlobalFrame_all(:,:,i)* Xterm_linAll(:,i,j)
    %
    %       %  linprogcounterStart = linprogcounterStart+6;
    %       %  linprogcounterEnd= linprogcounterStart+5;
    %       %  Xterm_linall(linprogcounterStart:linprogcounterEnd) = [ Xterm_lin(1,i,j)  Xterm_lin(2,i,j) 0 0 0  Xterm_lin(3,i,j)];
    %         %        DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
    %         %I need to figure out what I'm getting different results from these
    %         %too torque calculations...
    %         %      ForceOnEachArm_WorldFrame(:,i,j)= xterm;
    %     end
    % end
    % Xterm_linallT = Xterm_linall';
    % for i = 1:Tpoints
    %     TotalPointsPerTime = Narms*6;
    %     Timestep = i;
    %     StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
    %     EndPoint = StartPoint+6*Narms-1;
    %     %This is the torque from linprog
    %     Xterm_lin_pertime =  Xterm_linallT(StartPoint:EndPoint);
    %     veriftyFtask_linprog(:,i) =   -ForceTransformationToGlobalFrame_all(:,:,i)*Xterm_lin_pertime;
    %     DTorqueLinVerify(:,i)=[Di_all0ArmthenPlanar_inversetranspose(:,:,1,i) Di_all0ArmthenPlanar_inversetranspose(:,:,2,i)]*T(:,i);
    % end
    % veriftyFtask_linprog
    % DTorqueLinVerify
    % ForceOnEachArm_baseframe = Xterm_linAll;
    % for i = 1:Tpoints
    %     for j = 1:Narms
    %         Jangles = JointAngles_traj(i,:,j);
    %         TEE=  bot_all(j).A(1:length(Jangles),Jangles);
    %         %TEE
    %         %ForceOnEachArm_baseframe
    %         %ForceOnEachArm_baseframe(:,i,j)
    %         ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_baseframe(:,i,j));
    %         %ForceOnEachArm_ArmEEframe
    %     end
    % end
    %
    % MassPerArmLin = zeros(Narms,Tpoints);
    % MassPerTimeLin = zeros(Tpoints,1);
    %
    % for i = 1:Tpoints
    %     tempi = 1;
    %     for j = 1:Narms
    %         for k = 1:Njoints
    %             RPM(tempi,i) = qdotfew(i,k, j);
    %             tempi= tempi+1;
    %         end
    %     end
    % end
    %
    % MassAllLin = zeros(2,Tpoints,length(T(:,1)));
    % MassPerArmLin = zeros(2,Tpoints);
    % MassPerTimeLin = zeros(Tpoints,1);
    % for i = 1:Tpoints
    %     for j = 1:size(T(:,i))
    %         [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(T(j,i)),RPM(j,i));
    %         if j < 7
    %             k = 1;
    %         else
    %             k = 2;
    %         end
    %
    %         MassAllLin(k,i,j) = FinalMotorMass+FinalHarmonicMass;
    %         MassPerArmLin(k,i) = MassPerArmLin(k,i)+MassAllLin(k,i,j);
    %         MassPerTimeLin(i) = MassPerTimeLin(i)+MassAllLin(k,i,j);
    %     end
    % end