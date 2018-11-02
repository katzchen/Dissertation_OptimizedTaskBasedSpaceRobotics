function [J,Theta,err,qdot,qddot,Coriolis,GravLoad,bot] = singlearm_robottoolbox_NDOF_Dynamic_bot(bot,RtoEE,RotationTask,xyz_task,xyz_start,V_ee, Vdot_ee,q0)
%use robotics toolbox to find inverse kinematics and Jacobian
%step 1: put into DH parameters
%step 2: calculate joint angles
%Now with taking into account the velocity and acceleration of the
%end-effector. Note: V_ee and vdot_ee are given at the END-EFFECTOR not the
%payload center 

NDOF = length(bot.links);
for i = 1:NDOF
    Lengths(i) = bot.links(i).a;
end

m = zeros(length(Lengths),1);
if length(Lengths < 6);
    m = zeros(6,1);
end

xyz_task_rel = xyz_task;%-xyz_start; %grasping point - base because I need to shift everything over in to calculate the angles
fillin = [0 0 0 1];
T = [RotationTask  xyz_task_rel'; fillin];

%T = [cos(theta_task(1)) -sin(theta_task(1)) 0 xyz_task(1); sin(theta_task(1)) cos(theta_task(1)) 0 xyz_task(2); 0 0 1 xyz_task(3); 0 0 0 1];

%Ideally for 2DOF I would have it calculate but... lets just move on to
%4DOF

q=q0;
Cartesian_final = bot.fkine(q);

% Cartesian_final
% xyz_task_rel
% xyz_task
% xyz_start
% err = sqrt((Cartesian_final(1,4)-xyz_task_rel(1))^2+(Cartesian_final(2,4)-xyz_task_rel(2))^2+(Cartesian_final(3,4)-xyz_task_rel(3))^2);
% err
% 
% if err > max(Lengths)/1000;       
%     'change initial conditions'
%     [q,err] = bot.ikunc(T,.25*q0);
%     if err > max(Lengths)/1000    
%         'try chancing IC again'
%         [q,err] = bot.ikunc(T,-.25*q0);
%         if err > max(Lengths)/1000           
%             'Error is high, trying more brute force IK'
%             
%             %mask cannot be longer than 6 but if I have more than 6 joints, I have
%             %a longer mask HOWEVER I also don't need the mask.
%             if length(m) > 6
%                 m = ones(6,1);
%             end
%             
%             [q] = bot.ikine(T,q0,m);
%             %I don't have a new error
%             T_calc = bot.fkine(q);
%             x = T_calc(1,4);
%             y = T_calc(2,4);
%             z = T_calc(3,4);
%             err = sqrt((x-xyz_task_rel(1))^2+(y-xyz_task_rel(2))^2+(z-xyz_task_rel(3))^2);
%             
%             if err > max(Lengths)/1000
%                 'Trying one more time'
%                 [q] = bot.ikine(T,-1*q0,m);
%                 %I don't have a new error
%                 T_calc = bot.fkine(q);
%                 x = T_calc(1,4);
%                 y = T_calc(2,4);
%                 z = T_calc(3,4);
%                 err = sqrt((x-xyz_task_rel(1))^2+(y-xyz_task_rel(2))^2+(z-xyz_task_rel(3))^2);
%                 if err > max(Lengths)/1000
%                     'Very last one more time'
%                     [q] = bot.ikine(T,2*q0,m);
%                     %I don't have a new error
%                     T_calc = bot.fkine(q);
%                     x = T_calc(1,4);
%                     y = T_calc(2,4);
%                     z = T_calc(3,4);
%                     err = sqrt((x-xyz_task_rel(1))^2+(y-xyz_task_rel(2))^2+(z-xyz_task_rel(3))^2);
%                 end
%             end
%             
%         end
%         
%     end
% end

%Jacobn is in end-effector frame
j_trans0 = bot.jacobn(q, 'trans');
j_rot0 = bot.jacobn(q, 'rot');
%bot.plot(q)
j_total = [j_trans0; j_rot0];


clear J
J = j_total;
Theta = q';
qdot = pinv(J)*V_ee;
Jdq = bot.jacob_dot(q,qdot); %product of the derivative of the Jacobian and hte joint rates IN THE WORLD FRAME. I want it in the End-effector frame

Jdq_ee = RtoEE*Jdq;
qddot = pinv(J)*(Vdot_ee-Jdq_ee);
%bot.payload(0,[0 0 0]);
bot.coriolis(q,qdot');

Coriolis = bot.coriolis(q,qdot')*qdot;
GravLoad = bot.gravload(q);


Cartesian_final = bot.fkine(q); 
err = sqrt((Cartesian_final(1,4)-xyz_task_rel(1))^2+(Cartesian_final(2,4)-xyz_task_rel(2))^2+(Cartesian_final(3,4)-xyz_task_rel(3))^2);
if err > max(Lengths)/1000
  %  printf('ERROR IS HIGH %d',err)
end
