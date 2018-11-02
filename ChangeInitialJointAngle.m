function AllVariables = ChangeInitialJointAngle(AllVariables,armnumber,NewJointAngle);

for i= 1:length(NewJointAngle)
    iterationlimit = 0;
    while NewJointAngle(i)<0 && iterationlimit < 10
        NewJointAngle(i)=NewJointAngle(i)+2*pi;
    end
    iterationlimit = 0;
    while NewJointAngle(i)>2*pi && iterationlimit < 10
        NewJointAngle(i)=NewJointAngle(i)-2*pi;
    end
end

initialLength = length(AllVariables.Arm(armnumber).TrajGenAngle);
if initialLength == length(NewJointAngle)
    AllVariables.Arm(armnumber).TrajGenAngle =NewJointAngle ;
else
    'Keeping original joint angle because size of NewJointAngle is wrong'
end
