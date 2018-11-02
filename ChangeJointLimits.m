function AllVariables = ChangeJointLimits(AllVariables,armnumber,NewJointAngle)
[njoints,shouldbe2] = size(NewJointAngle)
for j = 1:shouldbe2
    for i= 1:njoints
        iterationlimit = 0;
        while NewJointAngle(i,j)<-2*pi && iterationlimit < 10
            NewJointAngle(i,j)=NewJointAngle(i,j)+2*pi;
        end
        iterationlimit = 0;
        while NewJointAngle(i,j)>2*pi && iterationlimit < 10
            NewJointAngle(i,j)=NewJointAngle(i,j)-2*pi;
        end
    end
end

[initialLength, shoudlbe2] = size(AllVariables.Arm(armnumber).JointLimits);
if initialLength == length(NewJointAngle) &&shouldbe2==2
    AllVariables.Arm(armnumber).JointLimits =NewJointAngle ;
else
    'Keeping original joint angle because size of NewJointAngle is wrong'
end


for i = armnumber
    teach = 0;
    AllVariables.bot(i)= GenerateBot_singleArm(AllVariables.Arm(i).ArmLength,AllVariables.Arm(i).IsPlanar,AllVariables.Arm(i).armtype,AllVariables.Arm(i).Armoffset,teach);
    AllVariables.bot(i).base = [eye(3,3) AllVariables.Arm(i).xyz_base';0 0 0 1];
    AllVariables.bot(i).qlim =AllVariables.Arm(i).JointLimits;
end

