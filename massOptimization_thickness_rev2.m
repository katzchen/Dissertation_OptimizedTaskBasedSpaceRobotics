function [FinalMass,Diameter_Out_Joint,Diameter_In_Joint,Diameter_Out_Link,Diameter_In_Link,...
    JointInnerDiameter, MotorMassFraction,exitflag,Motor_harmonicMass,LinkMass,bot,TorqueStatic] = massOptimization_thickness_rev2(MaxDeflection,MinThickness,bot,...
    Torque,AngularVelocity, Density, E,...
    gearratio, GraspingDistance, PayloadAcc, PayloadMom,Mpayload,poisson,Narms)
%given a bot with link lengths, find the thicknesses given it has to
%withstand Pmax at the tip.

%Constraints are each LINK must be less than the bendingstressMax
%and the thickness (d_outer-d_inner) must be greater or equal to the
%minthickness

%Given the torque required, and a gear ratio, lets calculate the motors
%masses and diameters

%Variables: thickness and inner diameter. Going to do that rather than
%inner and outer as that isn't working?

%payloadAcc = linear acceleration of the payload
%Payloadmom = angular acceleration of the payload

PayloadForce = abs(PayloadAcc*Mpayload);

if PayloadForce < 220 %50lbs
    PayloadForce  = 250;
      'We are Using the Default Static Payload Force'
end

PayloadMoment = abs(PayloadMom*Mpayload*GraspingDistance^2);
if PayloadMoment< 20
    PayloadMoment = 20;
      'Using the default static payload moment'
end

[pone,NDOF] = size(bot.links);
%Using the energy method deflection =dU/dP
TotalRobotLength = 0;
for i = 1:NDOF
    Ltotal(i) = bot.a(i)+bot.d(i);
    TotalRobotLength = TotalRobotLength +Ltotal(i);
end

%static torque- does not use ROBOT MOTOR mass
LengthLeft = TotalRobotLength;
UseStatics = zeros(NDOF,1);
Torque_dynamic = Torque;



for i = 1:NDOF
    if i > 1
        LengthLeft = LengthLeft - Ltotal(i-1);
    end
    %    TorqueStatic(i) = LengthLeft*(PayloadForce/Narms)+PayloadMoment/Narms;
    TorqueStatic(i) = LengthLeft*(PayloadForce)+PayloadMoment;
    
    if TorqueStatic(i) > Torque_dynamic(i)
        %   AngularVelocity(i) = 0;
        %   Torque(i) = TorqueStatic(i);
        %   UseStatics(i) = 1;
    end
end
%'I will use the static torque for the following 1 = STATIC, 0 = DYNAMIC'
%
%       AngularVelocity(i) = 0;
%        Torque(i) = TorqueStatic(i);
Ltotal(NDOF+1) =  GraspingDistance;

TotalMotorMass = 0;
for i =1:NDOF
    %Going to use the angularvelocity limit to size the motor  max RPM
    JointRPM(i) = max(abs(AngularVelocity(i)))*60/(2*pi); %rad/sec*60sec/min
    if JointRPM(i) < 2
        JointRPM(i) = 2;
    end
    
    [MotorMass(i), HarmonicMass(i),MotorInertia(i),JointInnerDiameter(i),GearRatio(i) ] = JointMass(abs(Torque(i)),JointRPM(i),gearratio);
    TotalMotorMass = TotalMotorMass+MotorMass(i)+HarmonicMass(i);
    if JointInnerDiameter(i) < 0.005
        JointInnerDiameter(i) = 0.005;
    end
    ChrisMass(i) = ((abs(Torque(i))/(0.035))^(1/1.0822))/1000; %Chris estimate based on HD with mass in kg
end
torqueinput = Torque;
Motor_harmonicMass = MotorMass+HarmonicMass;

%Now we need to combine the joints with the link
L_link_joint = zeros(NDOF,1);
iter = 0;

%set up the lower and upper bounds

iter_final = 1;
for i =1:NDOF
    L_link_joint(i) = Ltotal(i);
    if Ltotal(i) > 1
        InnerDiameterMin(i) = MinThickness*4;
    else
        InnerDiameterMin(i) =  MinThickness;
    end
end

%x is the inner diameters and the outer diameters
%the objective function is the mass
A =[];
b=[];
Aeq = [];
beq = [];

%Lets see. I don't need this to be very constrained. I mean MinThickness is
%0.0016 so if I put A and b in mm, I really only need it to like the 1st or
%2nd decimal


lb = min(InnerDiameterMin)*ones(1,NDOF);
lb(1,1+NDOF:2*NDOF) = MinThickness*ones(1,NDOF);
ub = [Ltotal(1:NDOF) Ltotal(1:NDOF)/10];

x0 = [InnerDiameterMin MinThickness*ones(1,NDOF)];
f_mincon = @(x)mass_objectiveFunction_thickness(x,L_link_joint,Density);
nonlcon_mincon=  @(x)mass_constraintFunction_thickness_rev2(x,L_link_joint,MaxDeflection,E,MinThickness,PayloadForce, PayloadMoment,poisson);
options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',9000,'StepTolerance',1e-10,'TolCon', 1e-6, 'TolFun', 1e-5,'MaxIterations',1000,'Display','none');
[x,fval,exitflag,output]  = fmincon(f_mincon,x0',A,b,Aeq,beq,lb,ub,nonlcon_mincon,options);
[c,ceq,Dcalc,StressLink] = mass_constraintFunction_thickness_rev2(x,L_link_joint,MaxDeflection,E,MinThickness,PayloadAcc, PayloadMom,poisson); % Check the constraint values at x


%A*x
%b
DiameterOut_In = x;
Mass_calc_link = fval;

iter = 1;
for i = 1:NDOF
    Diameter_Out(iter) =x(i)+x(i+NDOF);
    Diameter_In(iter) = x(i);
    iter = iter+1;
end

iter = 1;
TotalLinkMass = 0;
for i = 1:NDOF
    Diameter_Out_Joint(i) = 0;%Diameter_Out(2*(i-1)+1);
    Diameter_In_Joint(i) = 0;%Diameter_In(2*(i-1)+1);
    Diameter_Out_Link(i) = Diameter_Out(i);
    Diameter_In_Link(i) = Diameter_In(i);
    LinkMass(i) = pi*((Diameter_Out_Link(i)^2-Diameter_In_Link(i)^2)/4)*Ltotal(i)*Density;
    %   MotorStructuralMass(iter) = pi*(Diameter_Out_Joint(iter)^2-Diameter_In_Joint(iter)^2)/4*JointLength(i)*Density;
    MotorStructuralMass(i) = 0;
    h = bot.links(i).a;
    r = Diameter_In_Link(i)/2;
    Ixy = 1/12*LinkMass(i)*h^2+1/4*LinkMass(i)*r^2;
    Iz = 1/2*LinkMass(i)*r^2; 
    bot.links(i).m = LinkMass(i);
    bot.links(i).I =[Ixy 0 0; 0 Ixy 0; 0 0 Iz];%moment of inertia of link
    bot.links(i).G = 0;
    bot.links(i).Jm = (MotorInertia(i));
    TotalLinkMass = TotalLinkMass+LinkMass(i);
    iter = iter+1;
    
end
FinalThickness = x(NDOF+1:2*NDOF);
FinalMass = TotalMotorMass+TotalLinkMass;
MotorMassFraction = TotalMotorMass/FinalMass;

