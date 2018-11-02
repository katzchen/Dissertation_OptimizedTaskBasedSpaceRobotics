function [Tcalc,MaxMassPerJoint,MassPerArm,MassSystem,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformSingleoptimization_choose(Narms,Tpoints,Njoints,Di_all0,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdotfew,PayloadXYZ,PayloadRPY)
%Divide Task force in half. Apply to arms

Ftask  = ForceOnpayloadInGlobalFrame; %in global frame
Farm = Ftask; %what if I put this in 
%'starting Simple Optimization single arm'


for i = 1:Tpoints
    for j = Narms
        ForceOnEachArm_ArmEEframe(:,j,i)=(ForceTransformationToGlobalFrame_all(:,(j-1)*6+1:(j-1)*6+6,i))^(-1)*Ftask(:,i,1);
    end
end

%ForceOnEachArm_ArmEEframe
%MassInertia
%F_final_ArmFrame

for i = 1:Tpoints
    for j = Narms  
        
      %  [TotalTorque_optimize(i,:,j)] = bot.rne(Kinematics(j).Arm.Angle(i,:),  Kinematics(j).Arm.Velocity(i,:),  Kinematics(j).Arm.Acc(i,:),ArmVariables(j).gravity',F_final_ArmFrame(:,j,i)); %Need to recalculate since this changes with arm mass
        
        Dtemp = Di_all0(:,:,i,j);
        term1 = MassInertia(:,:,j,i)*qddot_all(i,:,j)';
        term2 = CorGravall(i,:,j)';
        Jangles = JointAngles_traj(i,:,j);
        velocity = qdotfew(i,:,j) ;
        acc = qddot_all(i,:,j);
        Forceapplied = Farm(:,i);
        Jrecalcn = bot_all(j).jacobn(Jangles);%THis kinda assumes the base is at zero
        DTFa = Dtemp'*Farm(:,i);
       % Tcalc(:,i) = (DTFa-(term1+term2));
        Tcalc2(:,i) = Jrecalcn'*ForceOnEachArm_ArmEEframe(:,j,i)+(term1+term2); %this gives me the same result as rne- I want EE frame. 
        Tcalc(:,i) =  (DTFa-(term1+term2));
        [Tcalc_rne(:,i)] = bot_all(j).rne(Jangles, velocity, acc,[0 0 0]',ForceOnEachArm_ArmEEframe(:,j,i)) ;
    end
end 
% 
% Tcalc
% Tcalc_rne
% Tcalc2

MassPerArm = zeros(2,1);
MassPerTime = zeros(Tpoints,1);
MassAll = 0;

 T = [Tcalc(:,:,1)];
 [A,B,C] = size(qdotfew);
for i = 1:Tpoints
AngularVelocity = reshape(qdotfew(i,:,:),[B*C,1]);
    for j = 1:size(T(:,i))
        [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(T(j,i)), AngularVelocity(j)*60/(2*pi));
        if j < B+1
            k = 1;
        MassAllLin(i,j) = FinalMotorMass+FinalHarmonicMass;
        else
            k = 2;
        MassAllLin(i,j-B) = FinalMotorMass+FinalHarmonicMass;
        end       
    end
end

for j= 1:Njoints
    MaxMassPerJoint(j) =max(MassAllLin(:,j));
end

for j= 1:Njoints
    MassPerArm(Narms)= MaxMassPerJoint(j) +MassPerArm(Narms);
end
MassSystem = MassPerArm(Narms);

ForceOnEachArm_baseframe(:,:,1) = Farm;
for i = 1:Tpoints
    for j = Narms
        Jangles = JointAngles_traj(i,:,j);
        TEE=  bot_all(j).A(1:length(Jangles),Jangles);
        ForceOnEachArm_ArmEEframe(:,1,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_baseframe(:,i,1));
    end
end