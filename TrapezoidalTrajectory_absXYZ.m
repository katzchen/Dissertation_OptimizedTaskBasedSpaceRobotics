function [A,V,T,tall,didthisslowdown,Final_time,calc_acceleration_i,calc_velocity_i] = TrapezoidalTrajectory_absXYZ(StartingXYZ,FinalXYZ,tpoints,maxVelocity,maxAcceleration,DesiredTime)
%
% Given the starting and end coordinates of each arm
% Given teh maxVelocity and maxAcceleration for the payload
% Find the angular accelerations and angular velocities
%make sure its not above the current/power restrictions
% Calculate, the joint angles, joint velocities, joint accelerations and
% TIME

%didthisslowdown  = 0 means NO. It did not. Which meant that it defined the
%time. So if didthisslowdown == 0 then it could not make the desiredtime so
%the desired time is re-defined


%assume both arms move at the same time
%assume all joints move at once
%Assume all joints take the same amount of time to move so that the joint
%that must move the furthest moves the fastest, and the other joints move
%slower

[Narms,shouldbethree] = size(StartingXYZ);

if shouldbethree < 3
    StartingXYZ = StartingXYZ';
    FinalXYZ = FinalXYZ';
    [Narms,shouldbethree] = size(StartingXYZ);
end


if shouldbethree < 3
    'ERROR IN DESIRED TRAJECTORY STARTING LOCATION- TrapexoidalTrajectory_absXYZ line 31'
end

Delta = FinalXYZ-StartingXYZ;
%use distance formula to find absolute delta
distance = sqrt(Delta(1)^2+Delta(2)^2+Delta(3)^2);
%calculated maxAcceleration if being the desired time
%start at zero velocity and at acceleration
%X = Xo+.5*A*t^2
%distance = X-xo = .5*A*t^2
%distance = .5*A*t^2
calc_acceleration = 2*(distance/2)/(DesiredTime/2)^2;
%Velocity at the half way point
%V = Vo+A*t
calc_velocity = calc_acceleration*(DesiredTime/2);
distanceaccelerating = .5*calc_acceleration*(DesiredTime/2);
distancedecelerating = calc_velocity*(DesiredTime/2)-.5*calc_acceleration*(DesiredTime/2);
totaldistance = distanceaccelerating+distancedecelerating;
 
if distance == 0
%    'Not moving'
    timetoaccelerate = 0;
    calc_velocity = 0;
    Final_velocity = 0;
    Final_acc = 0;
    Final_time =  DesiredTime;
    Final_accelerationtime = 0;
    Final_acc = 0;
    didthisslowdown = 0;
else
    
    
    if calc_acceleration > maxAcceleration
       % 'Unable to accomplish task with given time, exceeds acc limit'
        %so in order to accomplish it in the necessary time, I need to
        %acceleration faster than allowed
        didtthisslowdown = 1;
        %Find the time to acceleration, want to acceleration using the
        %maxAcceleration
        timetoaccelerate= sqrt(2*(distance/2)/maxAcceleration);
        calc_velocity = maxAcceleration*(timetoaccelerate);
        if calc_velocity > maxVelocity
    %        'need to slow down maxVelocity in addition to max Acceleration'
            didthisslowdown = 1;
            %V = Vo+A*t, Vo = 0, V=maxVelocity,
            timetoaccelerate = maxVelocity/maxAcceleration;
            calc_velocity = maxAcceleration*(timetoaccelerate);
            distanceafteraccelerating = .5*maxAcceleration*timetoaccelerate^2;
            distanceofconstantvelocity = distance-2*distanceafteraccelerating;
            timeofconstantvelocity = abs(distanceofconstantvelocity/calc_velocity);
            Final_velocity = calc_velocity;
            Final_acc = maxAcceleration;
            Final_time =  timeofconstantvelocity+2*timetoaccelerate;
            Final_accelerationtime = timetoaccelerate;
        else
     %       'Just need to reduce time to make sure I do not exceed acceleration'
            didthisslowdown = 1;
            Final_acc = maxAcceleration;
            Final_velocity = calc_velocity;
            Final_time = 2*timetoaccelerate;
            Final_accelerationtime = timetoaccelerate;
        end
    elseif calc_velocity > maxVelocity
    %    'Unable to accomplish task with given time, exceeds velocity limit'
        didthisslowdown = 1;
        timetoaccelerate = maxVelocity/maxAcceleration;
        calc_velocity = maxAcceleration*(timetoaccelerate);
        distanceafteraccelerating = .5*maxAcceleration*timetoaccelerate^2;
        distanceofconstantvelocity = distance-2*distanceafteraccelerating;
        timeofconstantvelocity = abs(distanceofconstantvelocity/calc_velocity);
        Final_velocity = calc_velocity;
        Final_acc = maxAcceleration;
        Final_time =  timeofconstantvelocity+2*timetoaccelerate;
        Final_accelerationtime = timetoaccelerate;
    else
  %      'can meet the desired time'
        %so I'm under the time and my velocity and my accelerations are within
        %spect. So the question is do I need to slow down
        
        %Just need to slow down to accomplish the task, lets do this by
        %reducing the acceleration to what I've already calculated
        time_withMaxAcceleration = 2*sqrt(2*(distance/2)/calc_acceleration);
        deltatime_shouldbe1 = time_withMaxAcceleration/DesiredTime; %this will always be less than 1
        calc_Maxvelocity =  calc_acceleration*(DesiredTime/2);
        Final_velocity = calc_Maxvelocity;
        Final_acc = calc_acceleration;
        Final_time = time_withMaxAcceleration;
        Final_accelerationtime = time_withMaxAcceleration /2;
        didthisslowdown = 0;
    end
end
%A_rel^2 = Ax^2+Ay^2+Az^2
%V_rel^2 = Vx^2+Vy^2+Vz^2
%T_rel^2 = deltaX^2+deltaY^2+deltaZ^2
%Trel^2/Distance^2 = (deltaX^2+deltaY^2+deltaZ^2)/Distance^2 = 1;

%assume all axis finish at the same time- which is defined by
%Final_time
totalacc = 0;
totalvel = 0;
totalacc_new = 0;
totalvel_new = 0;
distanceafteraccelerating = .5*Final_acc*Final_accelerationtime^2;
deltadistance = distance/distanceafteraccelerating ; %this is the first half

for i = 1:shouldbethree
    calc_acceleration_i(i) = 2*(Delta(i)/deltadistance)/(Final_accelerationtime)^2;
    if Final_accelerationtime== 0
         calc_acceleration_i(i) = 0;
    end
    calc_velocity_i(i) = calc_acceleration_i(i)*(Final_accelerationtime);
    totalacc = totalacc+calc_acceleration_i(i)^2;
    totalvel= totalvel+calc_velocity_i(i)^2;
end
totalacc =sqrt(totalacc);
totalvel = sqrt(totalvel);


% Final_acc
% Final_velocity
% calc_acceleration_i
% calc_velocity_i

tall = linspace(0,Final_time,tpoints*2);
deltat = (tall(2)-tall(1))/3600; %time between steps in hours
Final_constantvelocitytime = Final_time-2*Final_accelerationtime;
timestartconstantvelocity =Final_accelerationtime;
timeafterconstantvelocity= Final_accelerationtime+Final_constantvelocitytime; %time at the end of the constant velocity portion
%I have the final_time, total acceleration and total velocity for the
%maneuver, not per axis.
%
A = zeros(2*tpoints,3);
T = zeros(2*tpoints,3);
V = zeros(2*tpoints,3);

if isnan(max(tall)) == 1
    ' ERROR NO TIME'
end


for j = 1:shouldbethree
    for i = 1:2*tpoints
        t(i) =tall(i);
        if t(i) <= Final_accelerationtime
            A(i,j) = calc_acceleration_i(j); %norm of all the accelerations
            V(i,j) = A(i,j)*t(i);
            T(i,j) = StartingXYZ(j)+.5*A(i,j)*t(i)^2;
        elseif (t(i) > Final_accelerationtime && t(i) <= timeafterconstantvelocity)
            %constant velocity
            %'Constant Velocity'
            TafterAcceleration(j) = (StartingXYZ(j)+.5*calc_acceleration_i(j)*Final_accelerationtime^2);
            V(i,j) = calc_velocity_i(j);
            A(i,j) = 0;
            T(i,j) = TafterAcceleration(j)+calc_velocity_i(j)*(t(i)-Final_accelerationtime);
        elseif t(i) > timeafterconstantvelocity
            
            %'Decelerate'
            %decelerate
            TafterAcceleration(j) = (StartingXYZ(j)+.5*calc_acceleration_i(j)*Final_accelerationtime^2);
            TafterConstantVelocity(j) = TafterAcceleration(j)+calc_velocity_i(j)*Final_constantvelocitytime;
            time_aftervelocity = t(i)-(timeafterconstantvelocity);
            A(i,j)= -calc_acceleration_i(j);
            V(i,j) = calc_velocity_i(j)-calc_acceleration_i(j)*(time_aftervelocity);
            T(i,j) = TafterConstantVelocity(j)+calc_velocity_i(j)*(time_aftervelocity)-.5*calc_acceleration_i(j)*(time_aftervelocity)^2;
        else
            'ERROR IN TRAJECOTY. DONT KNOW WHERE I AM'
        end
    end
end

