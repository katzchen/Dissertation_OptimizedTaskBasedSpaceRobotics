
function [DynamicResults,MassResults]=Narm_Ndof_dynamic_Structure_NotIterative(Kinematics, Dynamics, TaskVariables, ArmVariables,ploton,bot)

%I have the relationship for optimal torque distribution where DiTi = DjTj for all i and j
%want to just have a single program for a system given Ftask
%Is Planar - if its planar than I don't want the optimizer to put in any
%forces in the Fz, Tx, Tz places, 1 = YES

%Task Variables
%maxtotaltime = Kinematics.finaltime;
Aworld(:,:) = [Kinematics(1).Payload.A_xyz Kinematics(1).Payload.A_rpy]; %should be the same for all arms since its the payload
Tworld(:,:) = [Kinematics(1).Payload.T_xyz Kinematics(1).Payload.T_rpy];
%Vworld(:,:) = [Kinematics(1).Payload.V_xyz Kinematics(1).Payload.V_rpy];


%Arm Variables
for i = 1:length(bot)
    qfew(:,:,i) = Kinematics(i).Arm.Angle;
    qdotfew(:,:,i) = Kinematics(i).Arm.Velocity;
    qddotfew(:,:,i) = Kinematics(i).Arm.Acc;
    Torque_final(:,:,i) = Dynamics(i).Arm.Torque;
    Time_traj = Kinematics(i).time_all;
    MaxDeflection(i) = ArmVariables(i).MaxDeflectionPerLength*ArmVariables(i).TotalArmLength;
    Velocity_angularlimit(i,:) = TaskVariables.Velocity_angularlimit*ones(length(ArmVariables(i).ArmLength),1);
    XGripAlongPayload(i)=ArmVariables(i).PayloadGrip(1);
    YGripAlongPayload(i)=ArmVariables(i).PayloadGrip(2);
    ZGripAlongPayload(i)=ArmVariables(i).PayloadGrip(3);
    theta_grasp(i,:)= ArmVariables(i).theta_grasp_start;
    IsPlanar(i) = ArmVariables(i).IsPlanar;
end

%Task Variables
xyz_task_start = TaskVariables.task_start;
xyz_task_end = TaskVariables.task_end;
tpoints =  TaskVariables.npoints; %I use it so often, Lets make it shorter
DesiredTime = TaskVariables.DesiredTime;
theta_task = TaskVariables.theta_task_start;
theta_task_end =TaskVariables.theta_task_end;
Mpayload = TaskVariables.Mpayload;
Lpayload = TaskVariables.Lpayload;
Ftask_cont = TaskVariables.ForceContinuous;
Pmax = TaskVariables.maxAcceleration_xyz*TaskVariables.Mpayload;
PayloadAcc =TaskVariables.maxAcceleration_xyz;
PayloadMoment =TaskVariables.maxAcceleration_rpy; %actually a torque
%%
Narms = length(bot);
NDOF = length(bot(1).links);
tpoints = 2*TaskVariables.npoints;

TorqueGoingtoMass_all = zeros(Narms,NDOF);
for i = 1:Narms
    for j = 1:tpoints
        Ji(:,:,j,i) = bot(i).jacob0(Kinematics(i).Arm.Angle(j,:)); %world frame
    end
end

%M = [1 1 1 1 1 1];

MassTotalAll = 9E99;
mass_err = 100;
dynamiciteration = 1;

ForceRequiredOnPayload =Mpayload*Aworld'; %yes. tihs is the forcerequired at the end-effector.
MinCount = 0;

GraspingDistance = 0;
for j = 1:Narms
    GraspingDistance(j) = sqrt(ArmVariables(j).PayloadGrip(1)^2+ ArmVariables(j).PayloadGrip(2)^2+ArmVariables(j).PayloadGrip(3)^2);
end

%'Lets do a static run first- this gives me a minimum mass'
MassTotalAll = 0;
for i = 1:Narms
    TorqueGoingtoMass = zeros(1,length(bot(i).links));
    [MassTotal_arm(i),Diameter_Out_Joint,Diameter_In_Joint,Diameter_Out_Link,Diameter_In_Link,...
        JointInnerDiameter, MotorMassFraction(i),exitflag(i),MotorMass_temp,LinkMass(i,:),bot_new(i),TorqueStatic] = massOptimization_thickness_rev2(MaxDeflection(i),...
        ArmVariables(i).material.MinThickness, bot(i),TorqueGoingtoMass, 0*max(abs(qdotfew(:,:,i))),...
        ArmVariables(i).material.Density, ArmVariables(i).material.E,...
        ArmVariables(i).gearratio, GraspingDistance(i),PayloadAcc,PayloadMoment,Mpayload,ArmVariables(i).material.poisson,Narms);
    if Narms == 1
        MassTotalAll = MassTotal_arm(i);
    else
        MassTotalAll = MassTotalAll+MassTotal_arm(i);
    end    
end
bot=bot_new;
bot_original = bot;
MinCount = 0;
MinAllowableCount = 2;
dynamiciteration=0;
%mass_err is in percentage so 1% difference is good enough
bot_start = bot;
mindynamiciteration = 4;

while (mass_err > 1 && dynamiciteration <9)
    MassTotalBefore = MassTotalAll;

    clear MaxTotalTorque_optimize MaxTotalTorque_optimize_stop
    [MaxTotalTorque_optimize,DynamicResults]=Narm_Ndof_dynamic_GetTorqueToPayload(Kinematics, Dynamics, TaskVariables, ArmVariables,ploton,bot);
    [MaxTotalTorque_optimize_stop,DynamicResults_stop]=Narm_Ndof_dynamic_GetTorqueToPayloadStopping(Kinematics, Dynamics, TaskVariables, ArmVariables,ploton,bot);
DynamicResults.Stop = DynamicResults_stop;
    
    
    MaxTotalTorqueOptimize_iter(MinCount+1,:,:) = MaxTotalTorque_optimize;
    MaxTotalTorqueOptimizeStop_iter(MinCount+1,:,:) = MaxTotalTorque_optimize_stop;
    MaxTotalTorque_optimize   = DynamicResults.Optimize.MaxJointTorquePerJoint;
    MaxTotalTorque_optimize_stop   = DynamicResults_stop.Optimize.MaxJointTorquePerJoint;
    MassTotalAll = 0;
    for i = 1:Narms
        ArmNumber  = i;
        clear TorqueGoingtoMass
        for k = 1:length(MaxTotalTorque_optimize(i,:))
            TorqueGoingtoMass(k) =  max(abs(MaxTotalTorque_optimize(i,k)),abs(MaxTotalTorque_optimize_stop(i,k)));
            if MaxTotalTorque_optimize_stop(i,k)>MaxTotalTorque_optimize(i,k)
                UseStopping(i,k) = 1;
            else
                UseStopping(i,k) = 0;
            end      
        end
        DynamicResults.UseStopping = UseStopping;
      %  TorqueGoingtoMass=MaxTotalTorque_optimize(i,:);
        TorqueGoingToArm_iter(MinCount+1,:,i) = TorqueGoingtoMass;
        %lets get the angular joint rate
        %I want to have the returning bot be under a different name because
        %I want to run all arms at the same iteration- Maybe this doesn't
        %matter since at this point the torques are already calculated and
        %one arm doesn't influence the other
        
        [MassTotal_arm(i),Diameter_Out_Joint,Diameter_In_Joint,Diameter_Out_Link,Diameter_In_Link,...
            JointInnerDiameter, MotorMassFraction(i),exitflag(i),MotorMass_temp,LinkMass(i,:),bot_new(i),TorqueStatic] = massOptimization_thickness_rev2(MaxDeflection(i),...
            ArmVariables(i).material.MinThickness, bot(i),TorqueGoingtoMass, max(abs(qdotfew(:,:,i))),...
            ArmVariables(i).material.Density, ArmVariables(i).material.E,...
            ArmVariables(i).gearratio, GraspingDistance(i),PayloadAcc,PayloadMoment,Mpayload,ArmVariables(i).material.poisson,Narms);
        
        MotorMass(i,:) = MotorMass_temp;
        if Narms == 1
            MassTotalAll = MassTotal_arm(i);
        else
        MassTotalAll = MassTotalAll+MassTotal_arm(i);
        end
        
    end
    %this could diverge if the mass gets too high because it will just keep
    %growing
    
    
        mass_err = (abs(MassTotalAll-MassTotalBefore)/MassTotalBefore)*100;
    mass_err = 100;
    MinCount = MinCount +1;
    if MinCount > MinAllowableCount
        dynamiciteration = dynamiciteration+1;
        mass_err = (abs(MassTotalAll-MassTotalBefore)/MassTotalBefore)*100;
    else
        
    mass_err = 100;
    end
    
    if MassTotalAll > 10^5 && dynamiciteration < mindynamiciteration
        'I have runnaway masss so reset'
        mass_err =100;
        bot = bot_original;
    elseif MassTotalAll > 10^5 && dynamiciteration >2
        'I have runnaway mass'
        mass_err =0;
        MassTotalAll = inf;
    else
        bot=bot_new;
    end
    MassTotalAllIter(MinCount) = MassTotalAll;
    if dynamiciteration < mindynamiciteration;
        mass_err = 100;
    end
    
    %    mass_err
end
% 
% TorqueGoingToArm_iter
% MaxTotalTorqueOptimize_iter
% MaxTotalTorqueOptimizeStop_iter
% MassTotalAllIter


MassResults.SystemTotal = MassTotalAll;
MassResults.LinkMass = LinkMass;
MassResults.MotorMass = MotorMass;
MassResults.MassperArm = MassTotal_arm;
MassResults.Joint.OuterDiameter = Diameter_Out_Joint;
MassResults.Joint.InnerDiameter = Diameter_In_Joint;
MassResults.Link.OuterDiameter = Diameter_Out_Link;
MassResults.Link.InnerDiameter = Diameter_In_Link;


%%
DynamicResults.StaticTorque = TorqueStatic;


















%Plot-Geometry
if ploton > 0
    %     figure
    %     subplot(3,1,1)
    %     hold all
    %
    %     for i = 1:6
    %         plot(Time_traj(:,1),Ae(:,i));
    %     end
    %     xlabel('time (sec)')
    %     ylabel('A Payload')
    %     legend('x','y','z','Rz','Ry','Rx')
    %
    %     subplot(3,1,2)
    %     hold all
    %     for i = 1:6
    %         plot(Time_traj(:,1),Vworld(:,i))
    %     end
    %     xlabel('time (sec)')
    %     ylabel('V Payload')
    %     legend('x','y','z','Rz','Ry','Rx')
    %
    %     subplot(3,1,3)
    %     hold all
    %     for i = 1:6
    %         plot(Time_traj(:,1),Tworld(:,i))
    %     end
    %     xlabel('time (sec)')
    %     ylabel('T Payload')
    %     legend('x','y','z','Rz','Ry','Rx')
    %
    figure
    for i=1:Narms
        Nplots = 8;
        subplot(Narms, Nplots , Nplots*(i-1)+1)
        hold on
        plot(Time_traj(:,1),qfew(:,:,i)*180/pi)
        %        plot([0 max(Time_traj(:,i))],[JointAngles_start(:,i) JointAngles_end(:,i)]*180/pi,'o')
        xlabel('time (sec)')
        ylabel('Angles(degree)')
        title(Nplots*(i-1)+1)
        
        subplot(Narms, Nplots , Nplots *(i-1)+2)
        hold on
        plot(Time_traj(:,1),qdotfew(:,:,i)*180/pi)
        xlabel('time (sec)')
        ylabel('Angles Vworldlocity(degree)')
        title(Nplots*(i-1)+2)
        
        subplot(Narms, Nplots , Nplots *(i-1)+3)
        plot(Time_traj(:,1),qddotfew(:,:,i)*180/pi)
        xlabel('time (sec)')
        ylabel('Angle Acceleraiton(degree)')
        title(Nplots*(i-1)+3)
        
        subplot(Narms, Nplots , Nplots *(i-1)+4)
        hold all
        plot(Time_traj(:,1),Torque(:,:,i))
        %        plot(0, T_start(:,:,i),'o')
        %       plot( max(Time_traj(:,i)), T_end(:,:,i),'s')
        xlabel('time (sec)')
        ylabel('Torque treated as single arms')
        
        subplot(Narms, Nplots , Nplots *(i-1)+5)
        hold all
        plot(Time_traj(:,1),SumTorque_nooptimization)
        xlabel('time (sec)')
        ylabel('Sum of Torque treated as single arms')
        
        
        subplot(Narms, Nplots , Nplots *(i-1)+6)
        hold all
        for j = 1:njoints
            plot(Time_traj(:,1),TotalTorque_optimize(:,j,i))
        end
        xlabel('time (sec)')
        ylabel('Torque Optimization')
        legend('Joint1','Joint2','joint3')
        
        
        
        subplot(Narms, Nplots , Nplots *(i-1)+7)
        hold all
        plot(Time_traj(:,1),SumTorque_optimize)
        xlabel('time (sec)')
        ylabel('Sum of all Joint Torque Optimization')
        
        
        subplot(Narms, Nplots , Nplots *(i-1)+8)
        hold all
        plot(1:njoints,MotorMass(i,:),'blue o')
        plot(1:njoints,LinkMass(i,:),'red o')
        
        xlabel('Joint or Link')
        ylabel('Mass (kg)')
        legend('Motor','Link');
        
    end
    
    %     figure
    %     hold on
    %     for i=1:Narms
    %         plot(Time_traj(:,1),SumTorque_optimize)
    %     end
    %     xlabel('time (sec)')
    %     ylabel('Sum of the Torque with Optimization')
    %
    %
    %
    [p1, p2,p3] = PlotPayload(xyz_task_start,theta_task,Lpayload);
    [p1end, p2end,p3end] = PlotPayload(xyz_task_end,theta_task_end,Lpayload);
    
    for ii = 1:Narms
        %Need to generate ARm A plots
        [ArmGrip(ii,:), Rarm_global]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task,...
            grippingpointalongPayloadLength_x(ii),  grippingpointalongPayloadLength_y(ii),  grippingpointalongPayloadLength_z(ii), theta_grasp(ii,:));
        plotArm_start(:,:,ii) =[xyz_base_all(ii,:); ArmGrip(ii,:)]';
        [ArmGrip(ii,:), Rarm_global]=  FindGlobalGrippingCoordinates(xyz_task_end,theta_task_end,...
            grippingpointalongPayloadLength_x(ii),  grippingpointalongPayloadLength_y(ii),  grippingpointalongPayloadLength_z(ii), theta_grasp(ii,:));
        plotArm_end(:,:,ii) =[xyz_base_all(ii,:); ArmGrip(ii,:)]';
        
        Angle_final= qfew(:,:,ii);
        
        clear theat_world;
        theta_world(1) = 0;
        [shouldbenarms, njoints] = size(Lall);
        
        istring = num2str(ii);
        nstring = num2str(Narms);
        
        name = strcat(nstring,'StartingPoseArms-',istring);
        name_end = strcat(nstring,'FinalPoseArms-',istring);
        
        if ii == 1
            figure
            %hold on
        end
        subplot(2,1,1)
        ax.Clipping = 'off';
        title('Task Location At START')
        hold all
        plot3(p1(1,:),p1(2,:),p1(3,:),'-+')
        plot3(p2(1,:),p2(2,:),p2(3,:),'-+')
        plot3(p3(1,:),p3(2,:),p3(3,:),'-+')
        xlim([-inf inf])
        ylim([-inf inf])
        
        
        [x_ellipse,y_ellipse,z_ellipse] = PayloadEllipsoid(xyz_task_start, Lpayload, theta_task);
        surf(x_ellipse,y_ellipse,z_ellipse,'FaceColor','none','EdgeAlpha',.25);
        
        %        plot3(plottask_x_start,plottask_y_start,plottask_z_start,'blue-+')
        %        plot3(plottask_x_end,plottask_y_end,plottask_z_end,'red-s')
        %plot3(plotArm_start(1,:,ii),plotArm_start(2,:,ii),plotArm_start(3,:,ii),'blue-o')
        %plot3(plotArm_end(1,:,ii),plotArm_end(2,:,ii),plotArm_end(3,:,ii),'red-p')
        
        subplot(2,1,2)
        ax.Clipping = 'off';
        hold all
        plot3(p1end(1,:),p1end(2,:),p1end(3,:),'-s')
        plot3(p2end(1,:),p2end(2,:),p2end(3,:),'-s')
        plot3(p3end(1,:),p3end(2,:),p3end(3,:),'-s')
        [x_ellipse_end,y_ellipse_end,z_ellipse_end] = PayloadEllipsoid(xyz_task_end, Lpayload, theta_task_end);
        surf(x_ellipse_end,y_ellipse_end,z_ellipse_end,'FaceColor','none','EdgeAlpha',.25);
        xlim([-inf inf])
        ylim([-inf inf])
        
        % [x,y,z] = ellipsoid(xyz_task_end(1),xyz_task_end(2),xyz_task_end(3), Lpayload(1)/2,Lpayload(2)/2,Lpayload(3)/2);
        % surf(x,y,z,'FaceColor','none','EdgeAlpha',.25)
        %   plot3(plot_x(ii,:),plot_y(ii,:),plot_z(ii,:),'-o');
        title('Task Location At End')
        
        
        linkcolor1 = [1 0 0];
        linkcolor2 = [0 1 0];
        linkcolor3 = [0 0 1];
        linkcolor4 = [1 0 1];
        divider = 3;
        if ii == 1
            subplot(2,1,1)
            bot1 = SerialLink(bot(ii),'name',name);
            bot1.plot(qfew(1,:,ii),'jointdiam', .5,'linkcolor', linkcolor1/divider,'nobase','noshadow','noshading','notiles')
            %hold all
            subplot(2,1,2)
            bot1_end = SerialLink(bot(ii),'name',name_end);
            bot1_end.plot(qfew(tpoints,:,ii),'jointdiam', .5,'linkcolor', linkcolor1,'nobase','noshadow','noshading','notiles')
            %'check'
        end
        if ii == 2
            bot2 = SerialLink(bot(ii),'name',name);
            bot2_end = SerialLink(bot(ii),'name',name_end);
            
            subplot(2,1,1)
            bot2.plot(qfew(1,:,2),'linkcolor', linkcolor2/divider,'jointcolor',[.5 .375 0],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles')
            bot1.plot(qfew(1,:,1),'jointdiam', .5,'linkcolor', linkcolor1/divider,'nobase','noshadow','noshading','notiles')
            
            subplot(2,1,2)
            bot2_end.plot(qfew(tpoints,:,2),'jointdiam', .5,'linkcolor',linkcolor2,'nobase','noshadow','noshading','notiles')
            bot1_end.plot(qfew(tpoints,:,1),'jointdiam', .5,'linkcolor', linkcolor1,'nobase','noshadow','noshading','notiles')
            
            %  bot2.plot(qfew(1,:,ii),'jointdiam', .5,'linkcolor', 'b','nobase','noshadow','noshading','notiles','nowrist')
            %for kk = 1:length(qfew(:,1,1))
            %    bot1.plot(qfew(kk,:,1))
            %    bot2.plot(qfew(kk,:,2))
            %end
        end
        if ii == 3
            bot3 = SerialLink(bot(ii),'name',name);
            bot3_end = SerialLink(bot(ii),'name',name_end);
            
            subplot(2,1,1)
            bot3.plot(qfew(1,:,3),'linkcolor',linkcolor3/divider,'jointcolor',[.25 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot2.plot(qfew(1,:,2),'linkcolor', linkcolor2/divider,'jointcolor',[.5 .375 0],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot1.plot(qfew(1,:,1),'jointdiam', .5,'linkcolor', linkcolor1/divider,'nobase','noshadow','noshading','notiles','nowrist')
            
            subplot(2,1,2)
            bot3_end.plot(qfew(tpoints,:,3),'linkcolor',linkcolor3,'jointcolor',[.5 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot2_end.plot(qfew(tpoints,:,2),'jointdiam', .5,'linkcolor',linkcolor2,'nobase','noshadow','noshading','notiles','nowrist')
            bot1_end.plot(qfew(tpoints,:,1),'jointdiam', .5,'linkcolor', linkcolor1,'nobase','noshadow','noshading','notiles','nowrist')
        end
        if ii == 4
            bot4 = SerialLink(bot,'name',name,'base', transl(xyz_base_all(i,:)));
            bot4_end = SerialLink(bot,'name',name_end,'base', transl(xyz_base_all(ii,:)));
            
            subplot(2,1,2)
            bot4.plot(qfew(1,:,4),'linkcolor',linkcolor4/divider,'jointcolor',[.25 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot3.plot(qfew(1,:,3),'linkcolor',linkcolor3/divider,'jointcolor',[.25 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot2.plot(qfew(1,:,2),'linkcolor', linkcolor2/divider,'jointcolor',[.5 .375 0],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot1.plot(qfew(1,:,1),'jointdiam', .5,'linkcolor', linkcolor1/divider,'nobase','noshadow','noshading','notiles','nowrist')
            
            subplot(2,1,2)
            bot4_end.plot(qfew(tpoints,:,4),'linkcolor',linkcolor4/divider,'jointcolor',[.25 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot3_end.plot(qfew(tpoints,:,3),'linkcolor',linkcolor3,'jointcolor',[.5 0 .5],'nobase','jointdiam', .5,'noshadow','noshading' ,'notiles','nowrist')
            bot2_end.plot(qfew(tpoints,:,2),'jointdiam', .5,'linkcolor',linkcolor2,'nobase','noshadow','noshading','notiles','nowrist')
            bot1_end.plot(qfew(tpoints,:,1),'jointdiam', .5,'linkcolor', linkcolor1,'nobase','noshadow','noshading','notiles','nowrist')
            
        end
        
        if ii == 5
            bot5 = SerialLink(bot,'name',name,'base', transl(xyz_base_all(i,:)));
            bot5.plot(qfew(1,:,ii),'linkcolor', [.5 .25 .25],'nobase','jointdiam',.5,'jointcolor',[.25 .125 .125],'noshadow','noshading','notiles','nowrist')
        end
        
        if ii == 6
            bot6 = SerialLink(bot,'name',name,'base', transl(xyz_base_all(i,:)));
            bot6.plot(qfew(1,:,ii),'linkcolor', [0 1 .5],'nobase','jointdiam',.5,'jointcolor',[0 .5 .25],'noshadow','noshading','notiles','nowrist')
        end
        % theta_world(i,j+1) = theta_world(i)+JointAngles(j,i);
        % PlotArmX(i,j+1) = Lall(i,j)*cos(theta_world(i+1))+PlotArmX(i,j);
        % PlotArmY(i,j+1) = Lall(i,j)*sin(theta_world(i+1))+PlotArmY(i,j);
        % PlotArmZ(i,j+1) = Lall(i,j)*sin(theta_world(i+1))+PlotArmZ(i,j);
        
    end
    
    %   PlotArmX_A(i+2) = PlotArmX_A(i+1)+Lpayload/2*cos(theta_world(i+1));
    %   PlotArmY_A(i+2) = PlotArmY_A(i+1)+Lpayload/2*sin(theta_world(i+1))
end