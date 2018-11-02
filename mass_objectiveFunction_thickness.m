function [C,Mass] = mass_objectiveFunction_thickness(x,Ltotal,Density)

%objective function for minimizing total mass in the links

%INPUTS
%x = [InnerDiameter thickness]
%Ltotal = [L1 L2 L3 ...Ln  GraspingDistance] etc
%Density is the density of the link length material

%OUTPUTS
%C the costing function

MassTotal = 0; 
for i = 1:length(Ltotal)
    InnerDiameter = x(i);
    Thickness(i) = x(i+length(Ltotal));
    OuterDiameter = InnerDiameter+Thickness(i)*2;
    
    %         if (x1-x2) <MinThickness;
    %              x(2*i-1) = x(2*i)+MinThickness;
    %              x1 = x(2*i)+MinThickness;
    %         end
    Mass(i) = Density*Ltotal(i)*((OuterDiameter/2)^2-(InnerDiameter/2)^2)*pi;
    MassTotal=MassTotal+Mass(i);
end
% Mass
% MassTotal
% Ltotal
% OuterDiameter
% Thickness
C = MassTotal; 