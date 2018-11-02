function [Torque_final, Acc_final, Velocity_final, Angle_final,tall, MinError,...
    WattHours, Current_calc, totaltime,time_accelerating,A,T,payload,J,CorGrav,EndCondition] = TrapezoidalTrajectory_PowerNDOFNarms_Cartesian(deltaGraspPayload_xyz,...
    Lpayload,StartingXYZ,FinalXYZ,StartingRPY,FinalRPY,bot,M,tpoints,RotationTask,XYZ_base,ArmStartingRPY,ArmFinalRPY,DesiredTime, grippingpointalongPayloadLength)
%
% Given the starting and end coordinates of each arm
% Given teh maxVelocity and maxAcceleration for the payload
% Find the angular accelerations and angular velocities
%make sure its not above the current/power restrictions
% Calculate, the joint angles, joint velocities, joint accelerations and
% TIME

%assume both arms move at the same time
%assume all joints move at once
%Assume all joints take the same amount of time to move so that the joint
%that must move the furthest moves the fastest, and the other joints move
%slower

%put these in a loop until both are donw with the same time. So yeah...
DesiredTime_new = DesiredTime; %given incase there are multiple arms
%maxVelocity = .02;
%nmaxAcceleration = .005;
iter = 0;
payload = 100000;
payload_offset = [0 0 0 ]; %I think this should be based on grasping locataion vs task

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
GivenWattHours = inf
maxCurrent = inf
maxVelocity_xyz = .01;
maxVelocity_rpy = .02;
maxAcceleration_xyz = .001
maxAcceleration_rpy = .001;
Error_current_limit = 1;
Error_power_limit = 1;
MaxNumberIterations = 100;

%bot.payload(payload,payload_offset);
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
    while timeerror > .1 && iter < maxiter
        iter = iter+1;
        DesiredTime = DesiredTime_new;
        %   [A_xyz, V_xyz, T_xyz,tall_xyz,didthisslowdown_xyz,ta_xyz,A_allow_xyz,V_allow_xyz ] = TrapezoidalTrajectory_XYZ(StartingXYZ,FinalXYZ,tpoints,maxVelocity_xyz,maxAcceleration_xyz,DesiredTime);
        [A_xyz, V_xyz, T_xyz,tall_xyz,didthisslowdown_xyz,ta_xyz,A_allow_xyz,V_allow_xyz ] = TrapezoidalTrajectory_absXYZ(StartingXYZ,FinalXYZ,tpoints,maxVelocity_xyz,maxAcceleration_xyz,DesiredTime);
        finaltime_xyz = max(tall_xyz);
        
        'Now moving over to the ANGLES'
        [A_rpy,V_rpy, T_rpy,tall_rpy,didthisslowdown_rpy,ta_rpy,A_allow_rpy,V_allow_rpy ] = TrapezoidalTrajectory_absXYZ(StartingRPY,FinalRPY,tpoints,maxVelocity_rpy,maxAcceleration_rpy,DesiredTime);
        finaltime_rpy = max(tall_rpy);
        DesiredTime_new = max(finaltime_xyz,finaltime_rpy);
   
        if (finaltime_xyz == 0)
            'Not moving in XYZ'  
          %  timeerror = 0;
        elseif (finaltime_rpy == 0)
            'Not moving in RPY'
            T_rpy = 0*T_rpy;
         %   timeerror = 0;
        else
            %Only have 1 time, so I don't need to iterate to make sure all
            %joints are moving at the same time
            timeerror = abs(DesiredTime-DesiredTime_new)/DesiredTime;
        end        
        
            timeerror = abs(DesiredTime-DesiredTime_new)/DesiredTime;
        finaltime = DesiredTime_new;
        
        A =[A_xyz A_rpy];
        V =[V_xyz V_rpy];
        T =[T_xyz T_rpy];
        T_rpy
        
        %From here lets calculate ge, a combintation of coriollis and
        %gravity
        %my frame isn't rotating... the payload might be? 
        
        if finaltime == max(tall_xyz)
            tall = tall_xyz;
            ta = ta_xyz;
            'Final Time defined by XYZ'
            ta
        end
        if finaltime == max(tall_rpy)
            tall = tall_rpy;
            ta = ta_rpy;
            'Final Time defined by RPY'
        end
        if (ta > 9E50)
            'Error with Ta'
            timeerror = 1000;
            DesiredTime_new = 100;
        end
        iter
    end
    iter
    iter = 0;
    tpoints
    figure 
    hold all
    
    %Figure out the delta angle between the payload and the arm
    
        theta_graspPayload(:,:) = T(:,4:6); 
        theta_arm_traj(:,1) = linspace(ArmStartingRPY(1),ArmFinalRPY(1),2*tpoints);
        theta_arm_traj(:,2) = linspace(ArmStartingRPY(2),ArmFinalRPY(2),2*tpoints);
        theta_arm_traj(:,3) = linspace(ArmStartingRPY(3),ArmFinalRPY(3),2*tpoints);
        theta_Arm_Global(:,:) = theta_graspPayload(:,:)+theta_arm_traj;
        
        
    for i = 1:2*tpoints
        %I have the location and acceleration of the center of the paylaod
        %so I need to convert that to the robot end point   
        
        %I have the angle of the Task (or center of the payload) 
        State = T(i,:);
        theta_payload = T(i,4:6);
        theta_temp = theta_payload; %RPY of the grasping point relative to the payload
        Rz = [cos(theta_temp(1)) -sin(theta_temp(1)) 0; sin(theta_temp(1)) cos(theta_temp(1)) 0; 0 0 1];
        Ry = [cos(theta_temp(2)) 0 sin(theta_temp(2)); 0 1 0; -sin(theta_temp(2)) 0 cos(theta_temp(2))];
        Rx = [1 0 0; 0 cos(theta_temp(3)) -sin(theta_temp(3)); 0 sin(theta_temp(3)) cos(theta_temp(3))];
        RotationTask_payload_global= Rz*Ry*Rx; %NEEDS TO BE IN BASE FRAME, angle of arm relative to teh payload in task frame to arm        
        Tmatrix_payload_global = [ RotationTask_payload_global T(i,1:3)'; 0 0 0 1];
        
        %Now to find the end of the payload- not the arm and how it graps,
        %just straight down
        theta_temp = [0 0 0]; %RPY of the grasping point relative to the payload
        Rz = [cos(theta_temp(1)) -sin(theta_temp(1)) 0; sin(theta_temp(1)) cos(theta_temp(1)) 0; 0 0 1];
        Ry = [cos(theta_temp(2)) 0 sin(theta_temp(2)); 0 1 0; -sin(theta_temp(2)) 0 cos(theta_temp(2))];
        Rx = [1 0 0; 0 cos(theta_temp(3)) -sin(theta_temp(3)); 0 sin(theta_temp(3)) cos(theta_temp(3))];
        RotationTask_end_payload= Rz*Ry*Rx; %NEEDS TO BE IN BASE FRAME, angle of arm relative to teh payload in task frame to arm        
        
        
        Tmatrix_end_payload = [ RotationTask_end_payload [grippingpointalongPayloadLength 0 0]'; 0 0 0 1];
        Tmatrix_end_global =  Tmatrix_payload_global* Tmatrix_end_payload;
        
        %Now to find the grasp of the arm relative to the end of the payload
        theta_temp =  theta_arm_traj(i,:) %RPY of the grasping point relative to the payload
        Rz = [cos(theta_temp(1)) -sin(theta_temp(1)) 0; sin(theta_temp(1)) cos(theta_temp(1)) 0; 0 0 1];
        Ry = [cos(theta_temp(2)) 0 sin(theta_temp(2)); 0 1 0; -sin(theta_temp(2)) 0 cos(theta_temp(2))];
        Rx = [1 0 0; 0 cos(theta_temp(3)) -sin(theta_temp(3)); 0 sin(theta_temp(3)) cos(theta_temp(3))];
        RotationTask_end_payload= Rz*Ry*Rx; %NEEDS TO BE IN BASE FRAME, angle of arm relative to teh payload in task frame to arm
        Tmatrix_arm_end = [ RotationTask_end_payload [0 0 0]'; 0 0 0 1];
        
        Tmatrix_arm_global = Tmatrix_end_global*Tmatrix_arm_end     ;
        Tarm(i,:) = [Tmatrix_arm_global(1,4) Tmatrix_arm_global(2,4) Tmatrix_arm_global(3,4)] ; 
        
        deltaGraspPayload_arm = T(i,1:3)-Tarm(i,:)
        plot3(Tarm(i,1),Tarm(i,2),Tarm(i,3),'o')
        plot3(T(i,1),T(i,2),T(i,3),'s')
        
        plot3(XYZ_base(1),XYZ_base(2),XYZ_base(3),'+')
        ShouldbeZero_Distancebetweenpoints =sqrt((Tarm(i,1)-T(i,1))^2+(Tarm(i,2)-T(i,2))^2+(Tarm(i,3)-T(i,3))^2)-Lpayload/2
        RotationTask_arm_global = Tmatrix_arm_global(1:3,1:3)
        
        if i > 1
            [J(:,:,i),Theta(i,:),err,qdot(i,:),qddot(i,:),Coriolis(i,:),GravLoad(i,:),bot]  = singlearm_robottoolbox_NDOF_Dynamic_bot(bot,RotationTask_arm_global,Tarm(i,1:3),XYZ_base,V(i,:)',A(i,:)',Theta(i-1,:));
        else            
            q0 = pi/4*ones(1,NDOF);
            [J(:,:,i),Theta(i,:),err,qdot(i,:),qddot(i,:),Coriolis(i,:),GravLoad(i,:),bot]  = singlearm_robottoolbox_NDOF_Dynamic_bot(bot,RotationTask_arm_global,Tarm(i,1:3),XYZ_base,V(i,:)',A(i,:)',q0);            
        end
        'lets verify'
        err
        Tbot = bot.fkine(Theta(i,:)) %THIS global
        Tmatrix_arm_global
        GraspingLocation_global = [Tbot(1,4) Tbot(2,4) Tbot(3,4)]+XYZ_base
        
        PayloadCenter =  T(i,1:3);
        PayloadAngle = T(i,4:6);
        
        clear theta_grasp
        theta_grasp(i,:) = PayloadAngle; %RPY of the grasping point relative to the payload
        Rz = [cos(theta_grasp(i,1)) -sin(theta_grasp(i,1)) 0; sin(theta_grasp(i,1)) cos(theta_grasp(i,1)) 0; 0 0 1];
        Ry = [cos(theta_grasp(i,2)) 0 sin(theta_grasp(i,2)); 0 1 0; -sin(theta_grasp(i,2)) 0 cos(theta_grasp(i,2))];
        Rx = [1 0 0; 0 cos(theta_grasp(i,3)) -sin(theta_grasp(i,3)); 0 sin(theta_grasp(i,3)) cos(theta_grasp(i,3))];
        RotationTask_payload= Rz*Ry*Rx; %NEEDS TO BE IN BASE FRAME, angle of arm relative to teh payload in task frame to arm
        Tpayload = [ RotationTask_payload PayloadCenter'; 0 0 0 1];
      
        EdgeofPayload_temp =   Tpayload *[-Lpayload/2 0 0 1]';
        EdgeofPayload= EdgeofPayload_temp(1:3)'
%        deltaGraspPayload_arm
        
        test =  sqrt((EdgeofPayload(1)-GraspingLocation_global(1))^2+ (EdgeofPayload(2)-GraspingLocation_global(2))^2+ (EdgeofPayload(3)-GraspingLocation_global(3))^2)
        x = deltaGraspPayload_arm(1);
        y= deltaGraspPayload_arm(2);
        z = deltaGraspPayload_arm(3);
        
        
        Jei(:,:,i,j) = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 -z y 1 0 0;z 0 -x 0 1 0;y -x 0 0 0 1]';        
        Farm(:,i) = dynamics_withPayload(payload,A(i,:)',Jei(:,:,i,j),Lpayload/2);
        Marm(:,i) = cross(Farm(1:3,i),[x y z]);
         %Now I SHOULD add in power constraints but I'm not going to do that now
    end
    
    Theta
    
   
    [Narms,shouldbethree] = size(StartingXYZ);
    Delta = StartingXYZ - FinalXYZ;
    maxDifference = max(max(Delta));
    
   
    %clear bot
    njoints = NDOF;
       bot2 = bot.nofriction();
    bot2.gravity = [0 0 0];
    

    for i = 1:2*tpoints
     %   [Tarm_calculated(:,i),inertia] = dynamics_withPayloadTorque(Farm(:,i),Coriolis(i,:),GravLoad(i,:),Theta(i,:),qdot(i,:),qddot(i,:),J(:,:,i),bot2);
        [tau(:,i)] = bot2.rne(Theta(i,:), qdot(i,:), qddot(i,:),[0 0 0],Farm(:,i)); %I don't know that this take into account a paylaod
   
    end
    
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
    powererror_calc =(1-deltaPower)*100
    if  powererror_calc <Error_power_limit
        'Good Power'
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
    didIalreadyadjust
    Error_power(iterpower) = PowerError;
    CurrentError =10;
     Error_current(iterpower) = CurrentError;
    if maxCurrent>=maxCurrent_calc
        'Good Current'
        CurrentError = 0;
        Error_current(iterpower) = 0;
    elseif didIalreadyadjust == 0
        'Bad Current'
        deltaCurrent = abs(maxCurrent/maxCurrent_calc);
        deltaCurrent
        
        if didthisslowdown_xyz == 0
            maxAcceleration_xyz
            maxAcceleration_xyz =  maxAcceleration_xyz*deltaCurrent*.9;
            maxVelocity_xyz =  maxVelocity_xyz*deltaCurrent*.9;
            maxVelocity_xyz
              maxAcceleration_xyz
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
% J
% Theta
% err
% qdot
% qddot
% Coriolis
% GravLoad
% 
% iterpower
% Error
[MinError,index]  = min(Error);
CurrentError = Error_current(index);
% PowerError = Error_power(index);
% PowerError
% CurrentError
% index

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
    
    figure
    subplot(5,1,1)
    hold all
    for i = 1:njoints
        plot(tall,torque(index,:,i))
    end
    xlabel('time (s)')
    ylabel('Torque')
    
    subplot(5,1,2)
    hold all
    for i = 1:njoints
        plot(tall,Power(index,:,i))
    end
    xlabel('time (s)')
    ylabel('Power')
    
    subplot(5,1,3)
    plot(tall, PowerHours(index,:));
    xlabel('time (s)')
    ylabel('Power Hours')
    
    subplot(5,1,4)
    plot(tall,TotalCurrent(index,:));
    xlabel('time (s)')
    ylabel('TotalCurrent')
    
    subplot(5,1,5)
    hold all
    for i = 1:njoints
        plot(tall, Current(index,:,i));
    end
    xlabel('time (s)')
    ylabel('Current (Amps)')
end


%
%     eq2= vpa(int(torque2,t,0,ta))
%     eq1= vpa(int(torque1,t,0,ta))
%     eq1n= vpa(int(torque1n,t,ta,2*ta))
%     eq2n= vpa(int(torque2n,t,ta,2*ta))
%     %Power = Motor*(eq1+eq2)
%     %eq1 = Power/Motor
%
%     for i = 1:length(P)
%         alleqn_p = abs(eq1)+abs(eq2);
%         alleqn_n = abs(eq1n)+abs(eq2n);
%         alleqn = alleqn_p +alleqn_n;
%         A_finalp(i) = vpa(solve(alleqn_p-P(i)/M));
%         A_finaln(i) = vpa(solve(alleqn_n-P(i)/M));
%         A_final(i,j) = vpa(solve(alleqn-P(i)/M));
%
%         ta_1 = (sqrt(2*(T1e-T1s)/A_final(i,j)))/2; %end acceleration time
%         ta_2 = (sqrt(2*(T2e-T2s)/A_final(i,j)))/2; %end acceleration time
%         if ta_1 > ta_2
%             ta_final(i,j) = vpa(ta_1);
%         else
%             ta_final(i,j) = vpa(ta_2);
%         end
%         A1_final(i,j) = vpa(2*(T1e-T1s)/ta_final(i,j)^2);
%         A2_final(i,j) = vpa(2*(T2e-T2s)/ta_final(i,j)^2);
%         V1_max_final(i) = vpa(A1_final(i,j)*ta_final(i,j));
%         V2_max_final(i) = vpa(A2_final(i,j)*ta_final(i,j));
%         ta_hours(i,j) = ta_final(i,j)/3600;
%         watthours(i,j) = P(i)*ta_hours(i,j);
%
%         clear ta
%         A1 = A1_final(i,j);
%         A2 = A2_final(i,j);
%         ta = ta_final(i,j);
%         npoints = 200;
%         t = linspace(0,2*ta,npoints);
%         for i= 1:npoints
%             if t(i) < ta
%                 T1(i) =  T1s+.5*A1*t(i)^2;
%             else
%                 V1max = A1*ta;
%                 V2max = A2*ta;
%                 V1n = V1max - A1*(t-ta);
%                 V1n = 2*A1*ta-A1*t; %2*ta = t;
%                 A1n = -A1;
%                 T1(i) = (T1s+.5*A1*ta^2)+V1max*(t(i)-ta)+.5*A1n*(t(i)-ta)^2;
%             end
%         end
%         figure
%         plot(t,T1)
%
%
%
%     end
% end
%
% figure
% hold all
% for i = 1:length(T2all)
%     a = rem(i,2)
%     if T2all(i) > pi/2
%         plot(P,2*ta_final(:,i),'.-')
%     else
%         plot(P,2*ta_final(:,i),'-o')
%     end
% end
% legend('Joint 2 = 0','0.3927','0.7854','1.1781','1.5708','1.9635','2.3562','2.7489','3.1416')
% xlabel('Power (Watt)')
% ylabel('Time (s)')
%
% figure
% hold all
% for i = 1:length(T2all)
%     if T2all(i) > pi/2
%         plot(P,A1_final(:,i),'.-')
%     else
%         plot(P,A1_final(:,i),'-o')
%     end
%
% end
% legend('Joint 2 = 0','0.3927','0.7854','1.1781','1.5708','1.9635','2.3562','2.7489','3.1416')
% xlabel('Power (Watt)')
% ylabel('Acceleration rad^2/s')
%
% figure
% hold all
% for i = 1:length(T2all)
%     if T2all(i) > pi/2
%         plot(P,A2_final(:,i),'.-')
%     else
%         plot(P,A2_final(:,i),'-o')
%     end
%
% end
% legend('Joint 2 = 0','0.3927','0.7854','1.1781','1.5708','1.9635','2.3562','2.7489','3.1416')
% xlabel('Power (Watt)')
% ylabel('Acceleration2 rad^2/s')
%
%
% figure
% hold all
% for i = 1:length(T2all)
%     if T2all(i) > pi/2
%         plot(P,watthours(:,i),'.-')
%     else
%         plot(P,watthours(:,i),'-o')
%     end
%
% end
% legend('Joint 2 = 0','0.3927','0.7854','1.1781','1.5708','1.9635','2.3562','2.7489','3.1416')
% xlabel('Power (Watt)')
% ylabel('Watt Hours W-hours')
% watthours
