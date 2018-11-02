function [Torque_final, Acc_final, Velocity_final, Angle_final,tall, maxerr_pos,...
    WattHours, Current_calc, totaltime,time_accelerating,A,T,V,payload,J,CorGrav,...
    EndCondition] = TrapezoidalTrajectory_PowerNDOFNarms_Cartesian_rev3(payload,Lpayload,...
    StartingXYZ,FinalXYZ,StartingRPY,FinalRPY,bot,M,tpoints,...
    XYZ_base,ArmStartingRPY,ArmFinalRPY,DesiredTime,GripAlongPayload,armname,...
maxVelocity_xyz,maxVelocity_rpy,maxAcceleration_xyz,maxAcceleration_rpy,IsPlanar)


grippingpointalongPayloadLength_x = GripAlongPayload(1);
grippingpointalongPayloadLength_y = GripAlongPayload(2);
grippingpointalongPayloadLength_z = GripAlongPayload(3);

%
% Given the starting and end coordinates of each arm
% Given teh maxVelocity and maxAcceleration for the payload
% Find the angular accelerations and angular velocities
%make sure its not above the current/power restrictions
% Calculate, the joint angles, joint velocities, joint accelerations and
% TIME

%Inputs:
%Lpayload - length of the payloads, mainly used to figure out the number of
%joints
%StartingXYZ - starting location fo the center of the payload in global
%FinalXYZ- final location of the center of the payload in global
%startingRPY- euler angles for the COM of the payload in global at start
%FinalRPY - euler angles for the COM of the payload in global at end
%Bot - the robot
%M - motor sensitivity values for each joint
%tpoints - number of points to take along the trajectory
%XYZ_base - global location of the base of each robot
%ArmStartingRPY- euler angle for starting RPY relative to the payload
%ArmFinalRPY - euler angle for the endeffector relative to the payload
%DesiredTime - time of the manuever if I can do it
%grippingpointalongPayloadLength - where the arm grips along the payload
%between [-Lpayload/2 Lpayload/2], assume only different along x

%assume both arms move at the same time
%assume all joints move at once
%Assume all joints take the same amount of time to move so that the joint
%that must move the furthest moves the fastest, and the other joints move
%slower

%put these in a loop until both are donw with the same time. So yeah...
DesiredTime_new = DesiredTime; %given incase there are multiple arms
iter = 0;
%payload = 14515; %what SRMS can do

Error = 100;
ta = 500;
ploton = 0;
plot_movie = 0;
currentlimited = 0;
CurrentError = 5;
PowerError = 5;
TotalWattHours_before = 0;
V1max_before = 100;
DeltaPower_before = 0;
Error_power_before = inf;
powercounter = 0;
gravity = [0 0 0];
voltage = 28;
GivenWattHours = inf;
maxCurrent = inf;
% maxVelocity_xyz = .06; %SRMS
% maxVelocity_rpy = .06; %SRMS
% maxAcceleration_xyz = .1
% maxAcceleration_rpy = .1
Error_current_limit = 1;
Error_power_limit = 1;
MaxNumberIterations = 100;

bot.gravity= gravity;

[pone,NDOF] = size(bot.links);
for j = 1:NDOF
    torquesensitivity(j) = M(j); %Nm/AMP
end

iterpower = 0;
while ((CurrentError > Error_current_limit || PowerError > Error_power_limit) && iterpower <MaxNumberIterations);
    iterpower = iterpower+1;
    timeerror = 100;
    maxiter = 100;
    clear tall tall_xyz tall_rpy
    
    %theese are in the world frame??
    
    [A,T,V,A_xyz,A_rpy,T_xyz,T_rpy,V_xyz,V_rpy,finaltime,tall] = TrapezoidalTrajectory_Payload(payload,...
        StartingXYZ,FinalXYZ,StartingRPY,FinalRPY,bot,tpoints,DesiredTime,maxVelocity_xyz,maxVelocity_rpy,maxAcceleration_xyz,maxAcceleration_rpy);
    
    %Figure out the delta angle between the payload and the arm
    theta_graspPayload(:,:) = T(:,4:6);
    theta_arm_traj(:,1) = linspace(ArmStartingRPY(1),ArmFinalRPY(1),2*tpoints);
    theta_arm_traj(:,2) = linspace(ArmStartingRPY(2),ArmFinalRPY(2),2*tpoints);
    theta_arm_traj(:,3) = linspace(ArmStartingRPY(3),ArmFinalRPY(3),2*tpoints);
    %theta_Arm_Global(:,:) = theta_graspPayload(:,:)+theta_arm_traj;
    
    for i = 1:2*tpoints
        [Tarm_global(i,:), Rarm_global(:,:,i)]=  FindGlobalGrippingCoordinates(T_xyz(i,:),T_rpy(i,:),...
            grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z,theta_arm_traj(i,:));
    end
    
    
    %'Check Singularities'
    if (GripAlongPayload(1))<0
        armnegative = 1;
    else
        armnegative = 0;
    end
    
    
    
    [Manip,q_all]= CheckSingularities(bot,XYZ_base,GripAlongPayload,T_xyz,T_rpy,2*tpoints,theta_arm_traj,V,armnegative,IsPlanar);
    q0 = q_all(1,:);
    clear tau
    for i = 1:2*tpoints
        
        %I have the location and acceleration of the center of the paylaod
        %so I need to convert that to the robot end point
        
        %I have the angle of the Task (or center of the payload)
        [Tarm_global(i,:), Rarm_global(:,:,i)]=  FindGlobalGrippingCoordinates(T_xyz(i,:),T_rpy(i,:),...
            grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z,theta_arm_traj(i,:));
        
        RotationTask_arm_global = Rarm_global;
        [Tmatrix]=  FindArmToPayloadCoordinates(grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z, theta_arm_traj(i,:));
        
        Rarm =  Tmatrix(1:3,1:3)^(-1);        %invert to get Payload to Arm
        Rtotal = [Rarm zeros(3,3);zeros(3,3) Rarm];
        armrelativetopayload = theta_arm_traj(i,:);
        
        %get velocity and acceleration of payload in terms of EE
        deltaXYZ_original = [grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z]; %location of EE relative to original (payload)
        deltaRPY_original = theta_arm_traj(i,:); %rotation of EE frame in the world frame?
        Vee(:,i) =VelocityTransformation(deltaXYZ_original,deltaRPY_original,V(i,:)');
        Aee(:,i) =AccelerationTransformation(deltaXYZ_original,deltaRPY_original,A(i,:)');
        
        [J(:,:,i),Theta(i,:),err,qdot(i,:),qddot(i,:),Coriolis(i,:),GravLoad(i,:),bot]  = singlearm_robottoolbox_NDOF_Dynamic_bot(bot,Rtotal,RotationTask_arm_global(:,:,i),Tarm_global(i,:),XYZ_base,Vee(:,i),Aee(:,i),q_all(i,:));
        Fpayload_InArmCoordinates(i,:) = PayloadinArmCoordinates(bot,payload,A(i,:), theta_arm_traj(i,:),grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z );
        [tau(:,i)] = bot.rne(Theta(i,:), qdot(i,:), qddot(i,:),gravity,Fpayload_InArmCoordinates(i,:)); %I don't know that this take into account a paylaod
        err_pos(i,:) = err;
    end

    maxerr_pos = max(err_pos);
    [Narms,shouldbethree] = size(StartingXYZ);
    

    %clear bot
    njoints = NDOF;
    
%         figure
%         armname
%         time = linspace(0,67,2*tpoints);
%         subplot(3,2,3)
%         plot(time, Aee)
%         
%         ylabel('Acceleration of Payload (m/s^2)')
%         legend('x','y','z','angular x','angular y','angular z','Location','eastoutside')
%       %  title(armname)
%     
%         subplot(3,2,5)
%         hold on
%        plot(time,tau')
%         ylabel('Joint Torques')
%         legend('joint 1','2','3','4','5','6','Location','eastoutside')
%     %    title(armname)
%     
%         subplot(3,2,2)
%          plot(time,Theta*180/pi)
%         ylabel('Theta (degrees)')
%         legend('joint 1','2','3','4','5','6','Location','eastoutside')
%     
%         subplot(3,2,4)
%          plot(time,qdot*180/pi)
%         ylabel('Qdot (deg/sec)')
%         legend('joint 1','2','3','4','5','6','Location','eastoutside')
%     
%         subplot(3,2,6)
%        plot(time,qddot*180/pi)
%         ylabel('Qddot (deg/sec^2)')
%         legend('joint 1','2','3','4','5','6','Location','eastoutside')
%     
%         subplot(3,2,1)
%         plot(time,Vee)
%         ylabel('Velocity of the Payload (m/s)')
%         legend('x','y','z','angular x','angular y','angular z','Location','eastoutside')
%     %
    %Ta = MaQddot+Ga-Ja'(-Jea^(-T)*(MeXeddot+ge))
    
    torque(iterpower,:,:) = tau;
    Tall_all(iterpower,:) = tall;
    Velocity(iterpower,:,:) = qdot;
    Angle(iterpower,:,:) = Theta;
    Acc(iterpower,:,:) = qddot;
    
    TotalPower(iterpower,:) = zeros(1,2*tpoints);
    TotalCurrent(iterpower,:) = zeros(1,2*tpoints)';
    
    for j = 1:2*tpoints
        for i= 1:njoints
            Power(iterpower,i,j) =abs(torque(iterpower,i,j)/torquesensitivity(i)*voltage);
            TotalPower(iterpower,j) =  TotalPower(iterpower,j)+Power(iterpower,i,j);
            
            Current(iterpower,i,j) =abs( torque(iterpower,i,j)/torquesensitivity(i));
            TotalCurrent(iterpower,j) =  TotalCurrent(iterpower,j)+ Current(iterpower,i,j);
        end
    end
    
    deltat =  ( finaltime /(2*tpoints-1))/3600; %assume evenly divided between time segments in hours
    PowerHours(iterpower,:) = TotalPower(iterpower,:)*deltat;
    
    TotalWattHours(iterpower) = 0;
    FinalTotalPower =0;
    for i = 1:2*tpoints
        TotalWattHours(iterpower) = TotalWattHours(iterpower)+PowerHours(iterpower,i,1);
    end
    %    finalTime = 2*ta+tc;
    maxCurrent_calc = max(TotalCurrent(iterpower,:));
    TotalWattHourstemp(iterpower) = TotalWattHours(iterpower);
    maxCurrentFinalAnglesmp(iterpower) = maxCurrent_calc;
    PowerWH = TotalWattHours(iterpower);
    
    %NOW LETS JUST CHECK TO MAKE SURE I"M UNDER CURRENT AND POWER LIMTIS
    didIalreadyadjust = 0;
    deltaPower = abs(GivenWattHours/TotalWattHours(iterpower));
    powererror_calc =(1-deltaPower)*100;
    if  powererror_calc <Error_power_limit
%        'Good Power'
        PowerError = 0;
        Error_power(iterpower) = 0;
    else
        %WattHours is too high which means the time is too slow, add in
        %constant velocity, decrease Vmax
        'bad power'
        PowerWH
        GivenWattHours
        deltaPower = abs(GivenWattHours/TotalWattHours(iterpower))
        if didthisslowdown_xyz == 0
            maxAcceleration_xyz
            maxAcceleration_xyz =  maxAcceleration_xyz*deltaPower*.9;
            maxVelocity_xyz =  maxVelocity_xyz*deltaPower*.9;
            maxAcceleration_xyz
            maxVelocity_xyz
        end
        if didthisslowdown_rpy == 0
            deltaPower
            maxVelocity_rpy
            maxAcceleration_rpy
            %  maxAcceleration_rpy =  maxAcceleration_rpy*deltaPower*.9;
            maxVelocity_rpy =  maxVelocity_rpy*deltaPower*.9;
            maxAcceleration_rpy
            maxAcceleration_rpy
        end
        
        didIalreadyadjust = 1;
        
        %Vmax_allow = Vmax_calc*deltaPower;
        
        if deltaPower < 1
            PowerError  = (1-deltaPower)*100;
        else
            PowerError = deltaPower;
            'Error - my GivenWattHours is less than my TotalWatt Hours. How did I get here'
        end
    end
    Error_power(iterpower) = PowerError;
    CurrentError =10;
    Error_current(iterpower) = CurrentError;
    if maxCurrent>=maxCurrent_calc
%        'Good Current'
        CurrentError = 0;
        Error_current(iterpower) = 0;
    elseif didIalreadyadjust == 0
        'Bad Current'
        deltaCurrent = abs(maxCurrent/maxCurrent_calc);
        deltaCurrent
        
        if didthisslowdown_xyz == 0
            maxAcceleration_xyz =  maxAcceleration_xyz*deltaCurrent*.9;
            maxVelocity_xyz =  maxVelocity_xyz*deltaCurrent*.9;
        elseif didthisslowdown_rpy == 0
            'Line 238'
            %   maxAcceleration_rpy =  maxAcceleration_rpy*deltaCurrent*.9;
            deltaCurrent
            maxVelocity_rpy =  maxVelocity_rpy*deltaCurrent*.9;
            maxAcceleration_rpy =  maxAcceleration_rpy*deltaCurrent*.9;
            maxAcceleration_rpy
            maxVelocity_rpy
        end
        
        
        CurrentError = (1-deltaCurrent)*100;
        Error_current(iterpower) = CurrentError;
    end
    
    %
    Error(iterpower) = sqrt(Error_current(iterpower)^2+Error_power(iterpower)^2);
    [MinError,index]  = min(Error);
    
    %  CurrentError = Error_current(index);
    %  PowerError = Error_power(index);
end
[MinError,index]  = min(Error);
CurrentError = Error_current(index);

% 'Analysis Done'
CorGrav = GravLoad(:,:)+Coriolis(:,:);
Acc_final(:,:) = Acc(index,:,:);
Velocity_final(:,:) = Velocity(index,:,:);

Angle_final(:,:) = Angle(index,:,:);
tall =    Tall_all(index,:);
Angle1=Angle(:,:,1);
Vel1 = Velocity(:,:,1);
Acc1=Acc(:,:,1); 

Torque_final(:,:) = torque(index,:,:);
Torque_final = Torque_final';

totaltime =finaltime;

time_accelerating = ta;
WattHours = TotalWattHours(index);
Atemp1 = Angle(index,:,1);
Add1 = Acc(index,:,1);
Current_calc = max(abs(TotalCurrent(index,:)));

if ((Current_calc > maxCurrent*(1+Error_current_limit/100)) && (WattHours > GivenWattHours*(1+Error_power_limit/100)))
    EndCondition = 1; %breaks both
    % 'Breaks all'
elseif (WattHours > GivenWattHours*(1+Error_power_limit/100))
    EndCondition = 3; %breaks power
    % 'Breaks Power'
elseif (Current_calc > maxCurrent*(1+Error_current_limit/100))
    EndCondition = 2; %breaks current
    % 'Breaks Current'
else
    EndCondition = 0;
end

if plot_movie == 1
    figure
    qi(:,:) =  Velocity(index,:,:)*180/pi;
    bot.plot((qi))
end

if ploton == 1
    figure
    subplot(3,1,1)
    hold all
    for i = 1:njoints
        plot(tall,Acc(index,:,i))
    end
    xlabel('time (s)')
    ylabel('Acceleration m/s')
    subplot(3,1,2)
    hold all
    for i = 1:njoints
        plot(tall,Velocity(index, :,i))
    end
    xlabel('time (s)')
    ylabel('Velocity m/s')
    subplot(3,1,3)
    hold all
    for i = 1:njoints
        plot(tall,Angle(index,:,i)*180/pi)
    end
    xlabel('time (s)')
    ylabel('Angle (degree)')
    %
    %     figure
    %     subplot(5,1,1)
    %     hold all
    %     for i = 1:njoints
    %         plot(tall,torque(index,:,i))
    %     end
    %     xlabel('time (s)')
    %     ylabel('Torque')
    %
    %     subplot(5,1,2)
    %     hold all
    %     for i = 1:njoints
    %         plot(tall,Power(index,:,i))
    %     end
    %     xlabel('time (s)')
    %     ylabel('Power')
    %
    %     subplot(5,1,3)
    %     plot(tall, PowerHours(index,:));
    %     xlabel('time (s)')
    %     ylabel('Power Hours')
    %
    %     subplot(5,1,4)
    %     plot(tall,TotalCurrent(index,:));
    %     xlabel('time (s)')
    %     ylabel('TotalCurrent')
    %
    %     subplot(5,1,5)
    %     hold all
    %     for i = 1:njoints
    %         plot(tall, Current(index,:,i));
    %     end
    %     xlabel('time (s)')
    %     ylabel('Current (Amps)')
end

%'LEAVING TRAPEZOIDAL'