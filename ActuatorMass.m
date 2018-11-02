function [FinalMotorMass,FinalHarmonicMass,FinalGearRatio] = ActuatorMass(T,RPM)

%this is the output torque of harmonic
%RPM is the output speed of harmonic
[HarmonicMass,GearRatio] =  HarmonicDataCurve(abs(T));

GearRatio

for i = 1:length(GearRatio)
    Torque_motor(i) = abs(T)/GearRatio(i);
    RPM_Motor(i) = RPM*GearRatio(i);
    MotorMass(i) = MotorMassEquation(Torque_motor(i),RPM_Motor(i));
    JointMass(i) = MotorMass(i)+HarmonicMass(i);
end
Torque_motor;
HarmonicMass;
MotorMass;
[FinalMotorMass,idx] = min(JointMass);
FinalHarmonicMass =HarmonicMass(idx);
FinalGearRatio = GearRatio(idx);