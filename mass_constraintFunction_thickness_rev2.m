function [c,ceq,Dcalc,StressLink] = mass_constraintFunction_thickness_rev2(x,Ltotal,Dmax,E,MinThickness,PayloadForce, PayloadMoment,poisson)

%objective function for minimizing total mass in the links
%constraint is to ensure deflection isn't exceeded
%also need to make sure the stresses aren't exceeded

%INPUTS
%x = [OuterDiameter InnerDiameter]
%Ltotal = [L1 L2 L3 ...Ln] etc
%Dmax is the maxdefleciton allowed at the tip
%Pload is the force load on the end
%modulus of elasticiy fo rthe material


%OUTPUTS
%C the costing function

%poissons ratio, needs to be moved out
%nu = 0.33;
nu = poisson;

dtotal = 0;
I = 0;
term1 = 0;
term2 = 0;

dforce_total = 0;
dmoment_total = 0;

for i = 1:length(Ltotal)  
    index1 = i;
    index2 = i+length(Ltotal);
    
    InnerDiameter = x(index1);
    Thickness = x(index2);
    OuterDiameter = InnerDiameter+Thickness*2;
    r =  OuterDiameter/2;
      
    I(i) = (OuterDiameter^4-InnerDiameter^4)*pi/64;
    term1 = 0;
    for j = i:length(Ltotal)
        term1 = term1+Ltotal(j);
    end
    
    term2 = 0;
    for j = (i+1):length(Ltotal)
        term2 = term2+Ltotal(j);
    end
    
    d_force(i) = (1/I(i))*(term1^3-term2^3);
    d_moment(i) = (1/I(i))*(term1^2-term2^2);
    dforce_total = dforce_total+d_force(i);
    dmoment_total = dmoment_total+d_moment(i);

    %critical buckling, treats each segment seperate
    phi = (1/16)*sqrt(r/Thickness);
    gamma = 1-.731*(1-exp(-phi));
    StressLink(i) = (PayloadForce+PayloadMoment)*term1*r/I(i);
    bucklingstressratio_axial(i) = 1/(gamma*E/(sqrt(3*(1-nu^2)))*(Thickness/r)); %should be negative
    bucklingstressratio_torsion(i) = 1/((.747*0.67*E)/((r/Thickness)^(5/4)*(Ltotal(i)/r)^.5));
    criticalbucklingstress(i) =  1/(bucklingstressratio_axial(i)+ bucklingstressratio_torsion(i));
    bucklingstressratio(i) = (StressLink(i)/criticalbucklingstress(i))-1; %should be greater than 1, so negative
    rt_criteria(i) = r/Thickness-1500; %r/thickness <  1500 so should be negative
end 
% criticalbucklingstress
% bucklingstressratio
% bucklingstressratio_axial
% bucklingstressratio_torsion

%combine the deflections
deflection_force_total = (PayloadForce/(3*E))*dforce_total;
deflection_moment_total = (PayloadMoment/(2*E))*dmoment_total;
Dcalc = deflection_force_total +deflection_moment_total;

%AllowableStress_matrix = ones(1,length(Ltotal))*AllowableStress;

%c=[StressLink-AllowableStress_matrix'];
%ceq = [(Dmax-Dcalc)]; %Ceq(x) = 0
%ceq=Dfraction;
%c = Cfraction; %Stress/Allowable-1 <= 0
%ceq = [Dfraction];
%c = [(StressLink-AllowableStress_matrix)/AllowableStress];
%c1=[Dcalc-Dmax]/Dmax*100;
c1 = (Dcalc-Dmax);
%ceq = [];
%Cfraction
%AllowableStress
%c = (Cfraction);
%Cfraction

c2 = bucklingstressratio;
c3 = rt_criteria;
ceq=[];
c = [c1 c2 c3];% Cfraction];



%c=[];

% Nonlinear inequality constraints
%c = -x(1)*x(2) - 10;
% Nonlinear equality constraints
%ceq = x(1)^2 + x(2) - 1;

