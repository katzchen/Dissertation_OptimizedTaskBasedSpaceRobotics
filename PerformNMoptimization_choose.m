function [Tcalc0,MaxMassPerJoint,MassPerArm,MassSystem,ForceOnEachArm_payload,ForceOnEachArm_ArmEEframe]= PerformNMoptimization_choose(x0,choose,Narms,Tpoints,Njoints,Ji0,qddot_all,CorGravall,MassInertia,GearRatio,...
    ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,IsPlanar,JointAngles_traj,bot_all,Di_all0,Di_all0ArmthenPlanar_inversetranspose,qdotfew)
%set up the optimization
%X is the force on each arm in the global frame AT THE EE
PlanarConstraint =0;

 maxvel_joint = max(abs(qdotfew)) %max velocity per joint per arm
 for i = 1:Narms
     for j = 1:Njoints
         maxvel_joint_RPM(j,i) = maxvel_joint(1,j,i)*60/(2*pi);
     end
 end
 
for j = 1:Narms
    if IsPlanar(j) == 1
        PlanarConstraint(1,(j-1)*6+3) = 1;
        PlanarConstraint(1,(j-1)*6+4) = 1;
        PlanarConstraint(1,(j-1)*6+5) = 1;
    end
end

A= [];
b = [];
lb = [];
ub = [];
Aeq = [];
beq = [];
isGA = 0;
if isinf(max(x0)>0)
    clear x0
    x0=0;
end

if length(x0) < Narms*6*Tpoints
    x0 = rand*ones(Narms*6*Tpoints,1)*100;
end


%in a planar case there should be nothing in the Fz
x0(3) = 0;
x0(4)=0;
x0(5)=0;
options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',200000,'StepTolerance',1e-12,'TolCon', 1e-8, 'TolFun', 1e-3,'MaxIterations',1200);

%
% Qddot(:,:) = qddot_all(i,:,:);
% Qdot(:,:) = qdot_all(i,:,:);
% Jtemp0(:,:,:)= Ji0(:,:,i,:);
% JtempEE(:,:,:)= JiEE(:,:,i,:);


clear f_mincon nonlcon_mincon

f_mincon = @(x)narm_objectivefunctionForce_PieceWise_trajAllPoints_Choose(x,Ji0,qddot_all,CorGravall,MassInertia,Narms,Njoints,Tpoints,GearRatio,choose,qdotfew*60/(2*pi));
nonlcon_mincon=  @(x)narm_constraintfunction_PieceWise_trajAllPoints(x,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,PlanarConstraint,Narms,Njoints,Tpoints);
[x,fval, exitflag, output] = fmincon(f_mincon,x0,A,b,Aeq,beq,lb,ub,nonlcon_mincon,options);
[c,ceq] = narm_constraintfunction_PieceWise_trajAllPoints(x,ForceTransformationToGlobalFrame_all,ForceOnpayloadInGlobalFrame,PlanarConstraint,Narms,Njoints,Tpoints); % Check the constraint values at x
output
exitflag
%x = force in base frame for each arm

xDTorque = zeros(Njoints,Tpoints);
for i = 1:Tpoints
    TotalPointsPerTime = Narms*6;
    Timestep = i;
    StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
    EndPoint = StartPoint+6*Narms-1;
    xpertime = x(StartPoint:EndPoint);    
    Qddot(:,:) = qddot_all(i,:,:);
    Jtemp0(:,:,:)= Ji0(:,:,i,:);
    
    for j = 1:Narms
        TotalPointsPerTime = Narms*6;
        Timestep = i;
        StartPoint = (j-1)*6+(i-1)*TotalPointsPerTime+1;
        EndPoint = StartPoint+6-1;
        Qdd1=Qddot(:,j);
        grav = [0 0 0];
        xterm = x(StartPoint:EndPoint);
        ForceOnEachArm(:,j,i) = xpertime; %In PAYLOAD FRAME
        Jangles = JointAngles_traj(i,:,j);
        
        %So this works when the jacobian is in the tool frame, X should be in tool frame
        Jrecalc0 = bot_all(j).jacobn(Jangles);
        %Ji0(:,:,i,j)
        %WhY is my Ji reporting the same as Jacobn, it only does when the
        %angle is 0 between EE
        
        term1 =  MassInertia(:,:,j)*Qddot(:,j);
        term2 = CorGravall(i,:,j)';
        term3 = Jrecalc0'*xterm;
        %M*qddot+g-J'F = T
        %xterm is the force on the tip but should be in the global frame
        CorGrav(:,j) = CorGravall(i,:,j);
        Tcalc0(:,i,j) =MassInertia(:,:,j)*Qddot(:,j)+CorGrav(:,j)-Jrecalc0'*xterm;%LinkMasses(:,:,j)*Qddot(:,j)+g_all(:,:,j)-Jtemp(:,:,j)'*x_all((j-1)*6+1:6*j);
%        T_time = T(1+(j-1)*Njoints:Njoints+(j-1)*Njoints,i);
        %M*qddot+g-J'F = T
 %       Jrecalc0T = Jrecalc0';
 %       Jrecalc0T_planar  =[Jrecalc0T(:,1) Jrecalc0T(:,2) Jrecalc0T(:,6)];
        
        DiTi0(:,i,j) =  Di_all0(:,:,i,j)*Tcalc0(:,i,j);
        %I need to figure out what I'm getting different results from these
        %too torque calculations...
        if j == 1
            ForceOnEachArm_WorldFrame(:,i,j)= ForceTransformationToGlobalFrame_all(:,1:6,i)*xterm;
        else
            ForceOnEachArm_WorldFrame(:,i,j)= ForceTransformationToGlobalFrame_all(:,7:12,i)*xterm;
        end
    end
end
if choose == 'e' || choose =='p'
    choose
DiTi0
'line 113'
end

ForceOnEachArm_WorldFrame
for i = 1:Tpoints
    for j = 1:Narms
        Jangles = JointAngles_traj(i,:,j);
        TEE=  bot_all(j).A(1:length(Jangles),Jangles);
        ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_WorldFrame(:,i,j));
    end
end



for i = 1:Tpoints
    TotalPointsPerTime = Narms*6;
    Timestep = i;
    StartPoint = (1-1)*6+(i-1)*TotalPointsPerTime+1;
    EndPoint = StartPoint+6*Narms-1;
    xpertime = x(StartPoint:EndPoint);
    verifyFtask(:,i) = -ForceTransformationToGlobalFrame_all(:,:,i)*xpertime; %should be the same as ForceOnpayloadInGlobalFrame
    ForceOnEachArm_payload(:,i,1) =  -ForceTransformationToGlobalFrame_all(1:6,1:6,i)*xpertime(1:6);
    ForceOnEachArm_payload(:,i,2) =  -ForceTransformationToGlobalFrame_all(1:6,7:12,i)*xpertime(7:12);
%    DTorqueVerify(:,i)=[Di_all0ArmthenPlanar_inversetranspose(:,:,1,i) Di_all0ArmthenPlanar_inversetranspose(:,:,2,i)]*[Tcalc0(:,i,1); Tcalc0(:,i,2)];
end

ForceOnEachArm_payload;
verifyFtask;

MassTotal = 0;
MassPerTime = zeros(Tpoints,1);
MassPerArm = zeros(Narms,Tpoints);
TorquePerTime = zeros(Tpoints,1);
for i = 1:Tpoints
    for j = 1:Narms
        for k = 1:Njoints
            RPM = maxvel_joint_RPM(k,j)*GearRatio(j);
            [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(Tcalc0(k,i,j)),maxvel_joint_RPM(k,j));
            MassAll(i,k,j) = FinalMotorMass+FinalHarmonicMass;
         end
    end
end


for k = 1:Narms
    for j= 1:Njoints
        MaxMassPerJoint(j,k) =max(MassAll(:,j,k ));
    end
end
MassPerArm =zeros(Narms,1);
for k = 1:Narms
    for j= 1:Njoints
        MassPerArm(k)= MaxMassPerJoint(j,k) +MassPerArm(k);
    end
end
MassSystem = MassPerArm(1)+MassPerArm(2);



