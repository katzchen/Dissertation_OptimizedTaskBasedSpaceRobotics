function [MotorMass,HarmonicMass, MotorInertia,Diameter,GearRatio ] = JointMass(torque,RPM,GearRatio,ploton)
%Goal: Find the motor and harmonic mass as well as the min frame diameter

%Input: Torque - for each joint at the output of the gearbox
%RPM - speed of output of the gearbox
%GearRatio - given

%Output: 
%MotorMass- based on Kollmorgen RBE series brushless, frameless motors,
%uses the RPM to pull corresponding torque values from the motor curves,
%then, for that specific RPM, creates a torque vs mass plot for the motors,
%which has a linear line fit to them with the coefficients being mavg (slope)
%and bavg(y-intercept). Which is then used to calculate the MotorMass

%%
%Must take into account the gearratio, which reduces the torque the motor
%experiences and decreases the speed the harmonic sees
%Assume GEAR REDUCTION
%%
%Motor based on torque
%[a b]  =MotorCurve_kollmorganTest(RPM);
ploton = 1;
%using analysis from all motors, just use the equation instead of going
%through the entire databaes
% a = -0.01539*10^-6*RPM^2+0.000266*RPM+0.528851;
% b = 0.014184*10^-6*RPM^2+RPM*0.000079+1.100324;
% c = 0.004513*10^-6*RPM^2-RPM*0.000022+0.641026;
% %[a b c ] = MotorCurve_AllData(RPM,ploton);
% 
% CAi = 5.248*RPM^(-.2882);
% Cki = 1.067*10^(-8)*RPM^(1.832)+1.036;
% Cbi = 0;

[MotorMass,HarmonicMass,GearRatio] = ActuatorMass(torque,RPM);
MotorInertia = 0.00003*MotorMass^1.8458; %excel from kollmorgen rbe data
torque_motor = torque/GearRatio;

D_harmonic = 0.0245*torque^0.3153;
D_motor_mm = 70.497*torque_motor^0.3034;
D_motor = D_motor_mm/1000;
Diameter = max(D_harmonic,D_motor);
%[MotorMass HarmonicMass Diameter GearRatio torque RPM]

% 
% 
% %[miavg biavg]  =MotorCurveInertia_kollmorgan(RPM);
% [HarmonicMass,GearRatio] = HarmonicDataCurve(abs(torque))
% torque_motor = abs(torque)/GearRatio;
% Speed_harmonic = RPM/GearRatio; %output of the harmonic
% 
% MotorMass = MotorMassEquation(torque_motor,RPM)
% % 
% torque
% HarmonicMass
%   HarmonicMass100 =0.0086*(abs(torque))^.9492
%   HarmonicMass120 =0.0049*(abs(torque))^1.0258
%   HarmonicMass160 =0.005*(abs(torque))^1.0258
%   'endcode'
% if GearRatio < 110
%     HarmonicMass =0.0086*(abs(torque))^.9492; %based on just the 100gear ratio and the limit peak torque
% elseif GearRatio < 140
%     HarmonicMass =0.0049*(abs(torque))^1.0258;%based on just the 120gear ratio and the limit peak torque
% else
%      HarmonicMass =0.005*(abs(torque))^1.0258; %based on just the 160gear ratio and the limit peak torque
% end
% if HarmonicMass < 0
%      HarmonicMass =0;
% end
% 
% 
% %ChrisMass = ((abs(torque)/(0.035))^(1/1.0822))/1000; %based on Chris's with just a high gear ratio
% %if ChrisMass > MotorMass
% %    MotorMass = ChrisMass;
% %end
% %Prot = torque_motor*RPM;
% %MotorMass_excel = .0979*torque_motor+.7805;
% %MotorMass_excelStall = .718*torque_motor^.7714;
% %MotorMass_excelPower = .0009*Prot^1.1683;
% 
% %IDDARM_gearRatio = [8137 8137 8137 1538 1538];
% %IDDARM_Torque =[ 45 45 20 9 9]./IDDARM_gearRatio; %Nm
% %IDDARM_Mass_kg =[.590  .480 .405 .380 .350];
% 
% %MotorInertia =  miavg*torque_motor+biavg;
% MotorInertia = 0.0001*MotorMass^1.8835; %excel from kollmorgen rbe data
% 
% 
% %%
% %Harmonic
% %based on heuristic obtained from harmonicnet
% k1 = 0.01196;
% k2 = 0.94343;
% HarmonicMass = k1*torque^k2; %in kg
% %HM = [0.0121*abs(torque)^0.9109 k1*torque^k2 HarmonicMass];
% %MM = [MotorMass MotorMass_excel MotorMass_excelStall MotorMass_excelPower ChrisMass];
% 
% 
% TotalMass = MotorMass+HarmonicMass;
% 
% %%
% %Lets find the Diameter
% D_harmonic = 0.0245*torque^0.3153;
% D_motor_mm = 70.497*torque_motor^0.3034;
% D_motor = D_motor_mm/1000;
% Diameter = max(D_harmonic,D_motor);