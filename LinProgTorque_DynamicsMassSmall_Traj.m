function [T,total,minValue] = LinProgTorque_DynamicsMassSmall_Traj(Tpoints,Narms, Njoints,qdarm,D,Ftask,Marm, Qddarm, CorGravarm, IsPlanar,friction)
%D is actually the inverse(D)


%LETS ASSUME SMALL TORQUE COEFFICIENT FOR RIGHT NOW. Not sure how to deal
%with that. May just write a single equation
GearRatio = 160;
%Coefficients_Motor = [0.0676   1.3788   .0010]; %small
Coefficients_Motor = [2.5067 .6071 -.2796]; %large
Coefficient_Harmonic = [-.3681 .0066];

%Minimize: Acutator Mass
%Mass= SUM{ C1+(CM/gearratio+CHarmonic)*T+C3*gearratio*S}
%Such that dynamics work
%
%M(q)qdd + cg +D'Ftask= T PER ARM or combine
%constraint: %T = D'Ftask+M*qddot+g
Tcoefficient = [Coefficients_Motor(2)/GearRatio+Coefficient_Harmonic(2)];
Scoefficient = [Coefficients_Motor(3)*GearRatio]; %I KNOW THIS, NOT A VARIABLE
Nothingcoefficient = [Coefficients_Motor(1)];%NOT A VARIABLE
%x = [T]; %[torque P Penalty] . Penalty should be 1
MotorMassPenalty = 0;
iter = 1;

    iter = 1;
for k = 1:Tpoints
     MotorMassPenalty(k) =0;
    for i = 1:Narms
        for j = 1:Njoints
            MotorMassPenalty(k) = MotorMassPenalty(k)+Nothingcoefficient+Scoefficient*qdarm(k,j,i)/1000; %THIS GETS ADDED TO THE MASS
            PenaltyCoefficient(iter) = Nothingcoefficient+Scoefficient*qdarm(k,j,i)/1000;
            iter = iter+1;
        end
    end
end

%Minimize SUM(C*abs(T))
%Minimize Sum(C*P)
%P >= T
%P >= -T

%according to matlab my x = [T P]
f = [zeros(Njoints*Narms*Tpoints,1);Tcoefficient*ones(Njoints*Narms*Tpoints,1)];%;ones(Njoints*Narms,1)];
%F'*x = sum (c*P)

%Now lets build constraints
%T <= P
%-T <= P
%Lets deal with the first 0 <= P-T, P+T <= 0
A1 = [eye(Njoints*Narms*Tpoints);-eye(Njoints*Narms*Tpoints)];%;zeros(Njoints*Narms)];
b1 = zeros(Njoints*Narms*Tpoints,1);
%Lets deal with the 2nd  -T <= P, 0 <= P+T, 0>= -P-T
A2 = [-eye(Njoints*Narms*Tpoints);-eye(Njoints*Narms*Tpoints)];%;zeros(Njoints*Narms)];
b2 =  zeros(Njoints*Narms*Tpoints,1);

Amin = [zeros(Njoints*Narms*Tpoints);-eye(Njoints*Narms*Tpoints)];%;zeros(Njoints*Narms)];
bmin = -friction*160;

%EQUALITY CONSTRAINTS
%Linear penalty coefficient. PenaltyCoefficient*1 = Penaltycoefficient

%Lets deal with the 3rd constraint sum(D*T) = Ftaskd
%M*qddot+g = T-D'Ftask
%T = D'Ftask+M*qddot+g

%Lets deal with the 3rd constraint sum(D*T) = Ftaskd
%Add in dynamics SUM(DT+D*(M*qdd+gi)) = Ftask +Me*Xedd+ge
%Add in dynamics SUM(DT)+sum(D*(M*qdd+gi)) = Ftask +Me*Xedd+ge
%Add in dynamics SUM(DT) = (Ftask +Me*Xedd+ge)-sum(D*(M*qdd+gi))
%Here D is the inverse transpose of the J_ei matrix or hte grasph mastrix
clear term1 term2
DynamicsArmTotal=0;


for j = 1:Tpoints    
    DynamicsArmTotal = 0;
    for i = 1:Narms
        clear Qtemp
        Qtemp=Qddarm(j,:,i);
        term1(:,i) = Marm(:,:,i,j)*Qtemp';
        term2(:,i) = CorGravarm(j,:,i)';
        term12(:,i)= term1(:,i)+term2(:,i);
        DynamicsArm(:,i) =D(:,:,j,i)*(term12(:,i));
        DynamicsArmTotal = DynamicsArmTotal+DynamicsArm(:,i);   
    end
    Ndof = length(D(1,:,j,i));
    DynamicsTotal((j-1)*Ndof+1:(j-1)*Ndof+Ndof,1) = DynamicsArmTotal;
end
%DynamicsTotal = [DynamicsArm(:,1)+DynamicsArm(:,2)];
%term1a = reshape(term1,[Njoints*Narms 1]);
%term2a = reshape(term2,[Njoints*Narms 1]);
%syms T1 T2 T3 T4 T5 T6 real
%syms P1 P2 P3 P4 P5 P6 real
%Xtemp = [T1 T2 T3 T4 T5 T6  P1 P2 P3 P4 P5 P6]'
% clear Aeq_individual
% %Lets see... all the T are in a row. So I ant all the Ds to be in a row
% for j = 1:Tpoints
%    D_all(:,:,j) = [D(:,:,1,j) D(:,:,2,j)];
%    Aeq_individual((j-1)*Njoints+1:(j-1)*Njoints+6,:,j) = [D_all(:,:,j) 0*D_all(:,:,j)];
% end

clear Dz
Dz = zeros(Ndof*Tpoints,Tpoints*Njoints*Narms); %I'm trying to make a matrix with a diagonal of Ds
for i = 1:Tpoints    
   D_all(:,:,i) = [D(:,:,i,1) D(:,:,i,2)];
   start = 1+(i-1)*Ndof;
   endrow = start+Ndof-1;
   startcolumn = 1+(i-1)*Ndof*2;
   endcolumn = startcolumn+Ndof*2-1;
   Dz(start:endrow,startcolumn:endcolumn) = D_all(:,:,i);
end

Z = 0*D_all(:,:,1);

%size of D1 is Tpoints nubmer of D_alls in a row. 
[R,C] = size(D_all(:,:,1));
D1 = zeros(R*Tpoints,C*Tpoints);
for i = 1:Tpoints
    startrow = (i-1)*Ndof+1;
    endrow = startrow+Ndof-1;
    startcolumn = (i-1)*2*Ndof+1;
    endcolumn = startcolumn+2*Ndof-1;
    D1(startrow:endrow,startcolumn:endcolumn) = D_all(:,:,i);
end
                                             
%D1 = [D_all(:,:,1) Z Z Z; Z D_all(:,:,2) Z Z; Z Z D_all(:,:,3) Z; Z Z Z D_all(:,:,4)];
D2 = [D1 0*D1];
clear Aeq
Aeq = D2;
if Ndof < 6
    FtaskPlanar = [Ftask(1,:); Ftask(2,:); Ftask(6,:)];
FtaskRe = reshape(FtaskPlanar,[Tpoints*Njoints 1]);
else
FtaskRe=reshape(Ftask,[Tpoints*Njoints 1]);
beq = FtaskRe-DynamicsTotal';
end

beq =FtaskRe-DynamicsTotal;
%D1 = rand(Njoints,Njoints);
%D2 = -rand(Njoints,Njoints);

%If this were the static case than the sum(D*T) = Ftask
%Ti =J(q)'(Je^-T)Fi per arm
%sum((Je^-T)*Fi)=Ftask 
%Ti = D'Ftask
%DT = Ftask

%Aeq = [D_allNew 0*D_allNew];% 0*D_allNew];
%beq = reshape([-Ftask+DynamicsArmTotal],[6*Tpoints 1]);


%T = D'*Ftask+ynamics
%Aeq = [D_all zeros(Njoints*Narms) zeros(Njoints*Narms)];
%beq = [D_allNew'*Ftask+term1a+term2a]; 

%BOUNDS
lb = [-inf*ones(Njoints*Narms*Tpoints,1); zeros(Njoints*Narms*Tpoints,1)];
ub =inf*ones(Njoints*Narms*Tpoints*2,1);


CheckIfWorked = (max(abs(FtaskRe))/max(abs(DynamicsTotal)))^-1;
%DynamicsTotal
%CheckIfWorked
exitflag = 0;
if max(max(abs(term1))) >  1.0e+9
CheckIfWorked = 1*10^9;
end

if CheckIfWorked > 1*10^8 && isinf(CheckIfWorked) == 0
    'dynamics maycause an issue so give inf as an answer'
    x = inf*ones(Njoints*Narms*Tpoints,1) ;
else
    A = [A1 A2 Amin]';
    options.MaxIterations = 50*(length(A)+length(Aeq));
    options.Display = 'off';
   [x,fval,exitflag,output] = linprog(f,[A1 A2 Amin]',[b1 b2 bmin],Aeq,beq,lb,ub,options);
end


if exitflag == 1
   
    T = x(1:Njoints*Narms*Tpoints); %this is the torque
    P = x(Tpoints*Njoints*Narms+1:2*Njoints*Narms*Tpoints);
    total = 0;
    for i = 1:length(T)
        total = abs(T(i))+total;
    end
    
    %Sum(D-'*T)+sum(D-'(MarmQddotarm+garm)) = Ftask-Mpaylaod-gpaylaod
    %Sum(D-'*T)+sum(D-'(MarmQddotarm+garm)) =Ftask (Ftask shoudl include the paylaod dynamics)
    
    minValue = f'*x+MotorMassPenalty;
else
    'cannot find linear optimal'
    x = inf*ones(2*Tpoints*Njoints*Narms,1) ;
    T = x(1:Njoints*Narms*Tpoints); %this is the torque
    P = x(Tpoints*Njoints*Narms+1:2*Njoints*Narms*Tpoints);
    minValue = inf;
    total = inf;
end


%minValue

% beq
%
% T
% P
