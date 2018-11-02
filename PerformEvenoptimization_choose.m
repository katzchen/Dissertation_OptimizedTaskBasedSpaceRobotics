function [Tcalc,MaxMassPerJoint,MassPerArm,MassSystem,ForceOnEachArm_baseframe,ForceOnEachArm_ArmEEframe]=PerformEvenoptimization_choose(Narms,Tpoints,Njoints,Di_all0,Di_all0ArmthenPlanar_inversetranspose,...
    ForceOnpayloadInGlobalFrame,MassInertia,qddot_all,CorGravall,IsPlanar,bot_all,JointAngles_traj,ForceTransformationToGlobalFrame_all,GearRatio,qdotfew,PayloadXYZ)
%Divide Task force in half. Apply to arms
'Even Distribution'
Ftask  = ForceOnpayloadInGlobalFrame;
Farm = Ftask/Narms;

for i = 1:Tpoints
    for j = 1:Narms
        Jangles = JointAngles_traj(i,:,j);
        TEE=  bot_all(j).A(1:length(Jangles),Jangles);
        ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),(TEE(1:3,4)+bot_all(j).base(1:3,4))-PayloadXYZ(i,:)', Farm(:,i,1));
    end
end

ForceOnEachArm_ArmEEframe



for i = 1:Tpoints
    for j = 1:Narms
        Dtemp = Di_all0(:,:,i,j);
        term1 = MassInertia(:,:,j,i)*qddot_all(i,:,j)';
        term2 = CorGravall(i,:,j)';
        Jangles = JointAngles_traj(i,:,j);
        Jrecalc0 = bot_all(j).jacob0(Jangles);
        DTFa = Dtemp'*Farm(:,i);
        Tcalc(:,i,j) = (DTFa-(term1+term2));
    end
end

MassPerArm = zeros(Narms,Tpoints);
MassPerTime = zeros(Tpoints,1);
MassAll = zeros(Tpoints,Njoints,Narms);

 T = [Tcalc(:,:,1); Tcalc(:,:,2)];
 
 [A,B,C] = size(qdotfew);
for i = 1:Tpoints
AngularVelocity = reshape(qdotfew(i,:,:),[Njoints*Narms,1]);
    for j = 1:size(T(:,i))
        [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(abs(T(j,i)), AngularVelocity(j)*60/(2*pi));
        if j < Njoints+1
            k = 1;
        MassAllLin(i,j,k) = FinalMotorMass+FinalHarmonicMass;
        else
            k = 2;
        MassAllLin(i,j-Njoints,k ) = FinalMotorMass+FinalHarmonicMass;
        end       
    end
end


for k = 1:Narms
    for j= 1:Njoints
        MaxMassPerJoint(j,k) =max(MassAllLin(:,j,k ));
    end
end

for k = 1:Narms
    for j= 1:Njoints
        MassPerArm(k)= MaxMassPerJoint(j,k) +MassPerArm(k);
    end
end
MassSystem = MassPerArm(1)+MassPerArm(2);

ForceOnEachArm_baseframe(:,:,1) = Farm;
ForceOnEachArm_baseframe(:,:,2) = Farm;
for i = 1:Tpoints
    for j = 1:Narms
        Jangles = JointAngles_traj(i,:,j);
        TEE=  bot_all(j).A(1:length(Jangles),Jangles);
        ForceOnEachArm_ArmEEframe(:,j,i)=ForceTransformation_UsingRotationMatrix(TEE(1:3,1:3),TEE(1:3,4), ForceOnEachArm_baseframe(:,i,j));
    end
end
