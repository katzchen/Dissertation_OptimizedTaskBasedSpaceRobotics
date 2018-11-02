function AllVariables = ChangeArmLength_single(x,AllVariables,armnumber)

%Only optimizing a single arm
if armnumber == 1
    otherarmnumber = 2;
else
    %sometimes my code is messy and I don't specify what arm2 shouldbe
    otherarmnumber = 1;
end

if armnumber == 0
    armnumber = 1; %means I need to deal with 2 arms
end

%sometimes my code is messy and I don't specify what arm2 shouldbe
if length(AllVariables.Arm(armnumber).ArmLength)<1
    AllVariables.Arm(armnumber).ArmLength = AllVariables.Arm(otherarmnumber).ArmLength;
end

if length(x) == 1
    % 'Case: Change The TotalLength for Arm2'
    
    %first calculate the length of the arm
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    %normalize the arm and mulitple by x
    AllVariables.Arm(armnumber).ArmLength = x*(AllVariables.Arm(armnumber).ArmLength/L1length);
    %recalculate the total length
    L1length = 0;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
    
else
    % 'Case: Change ALL the variables for Arm armnumber'
    L1length = 0;
    AllVariables.Arm(armnumber).ArmLength=x;
    for i = 1:length(AllVariables.Arm(armnumber).ArmLength)
        L1length = L1length+AllVariables.Arm(armnumber).ArmLength(i);
    end
    AllVariables.Arm(armnumber).TotalArmLength = L1length;
end
%x
SumX =AllVariables.Arm(armnumber).TotalArmLength;

AllVariables.Arm(otherarmnumber).ArmLength=0.01*ones(1,length(AllVariables.Arm(armnumber).ArmLength));
AllVariables.Arm(otherarmnumber).TotalArmLength = .01*6;