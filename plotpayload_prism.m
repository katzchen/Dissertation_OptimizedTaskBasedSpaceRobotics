function Prism = PlotPayload_prism(xyz_task_start,theta_task, Lpayload,ifplot)
%IFPLOT == 0 than PLOT

% close all
% clear all
% figure
% hold on
%
% Lpayload = [8 4 2];
% Mpayload = 116000; %this is what SSRMS CAN DO, kg
% x_task = 0;
% y_task = 4;
% z_task = 0;
%
% xyz_task_start= [x_task,y_task,z_task];
% theta_task =[45*pi/180 0*pi/180 0*pi/180]; %of the main payload relative to global

x_task = xyz_task_start(1);
y_task = xyz_task_start(2);
z_task = xyz_task_start(3);
[EndPayloadNeg1, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, -Lpayload(1)/2,-Lpayload(2)/2,-Lpayload(3)/2, [0 0 0]); %x, y, z coordinate
[EndPayloadNeg2, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, -Lpayload(1)/2,Lpayload(2)/2,-Lpayload(3)/2, [0 0 0]); %x, y, z coordinate
[EndPayloadPos1, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, Lpayload(1)/2,-Lpayload(2)/2,-Lpayload(3)/2, [0 0 0]);
[EndPayloadPos2, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, Lpayload(1)/2,Lpayload(2)/2,-Lpayload(3)/2, [0 0 0]);

[EndPayloadNeg1b, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, -Lpayload(1)/2,-Lpayload(2)/2,Lpayload(3)/2, [0 0 0]); %x, y, z coordinate
[EndPayloadNeg2b, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, -Lpayload(1)/2,Lpayload(2)/2,Lpayload(3)/2, [0 0 0]); %x, y, z coordinate

[EndPayloadPos1b, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, Lpayload(1)/2,-Lpayload(2)/2,Lpayload(3)/2, [0 0 0]);
[EndPayloadPos2b, Rarm_global1]=  FindGlobalGrippingCoordinates(xyz_task_start,theta_task, Lpayload(1)/2,Lpayload(2)/2,Lpayload(3)/2, [0 0 0]);

Xall_1= [EndPayloadNeg1(1) EndPayloadNeg2(1) EndPayloadPos2(1) EndPayloadPos1(1) EndPayloadNeg1(1)];
Yall_1= [EndPayloadNeg1(2) EndPayloadNeg2(2) EndPayloadPos2(2) EndPayloadPos1(2) EndPayloadNeg1(2) ];
Zall_1= [EndPayloadNeg1(3) EndPayloadNeg2(3) EndPayloadPos2(3) EndPayloadPos1(3) EndPayloadNeg1(3)];
if ifplot == 0
    fill3(Xall_1,Yall_1, Zall_1,'b')
end

Xall_1b= [EndPayloadNeg1b(1) EndPayloadNeg2b(1) EndPayloadPos2b(1) EndPayloadPos1b(1) EndPayloadNeg1b(1)];
Yall_1b= [EndPayloadNeg1b(2) EndPayloadNeg2b(2) EndPayloadPos2b(2) EndPayloadPos1b(2) EndPayloadNeg1b(2) ];
Zall_1b= [EndPayloadNeg1b(3) EndPayloadNeg2b(3) EndPayloadPos2b(3) EndPayloadPos1b(3) EndPayloadNeg1b(3)];
if ifplot == 0
    fill3(Xall_1b,Yall_1b, Zall_1b,'c')
end

Xall_1c= [EndPayloadNeg1b(1) EndPayloadNeg2b(1) EndPayloadNeg2(1) EndPayloadNeg1(1) EndPayloadNeg1b(1)];
Yall_1c= [EndPayloadNeg1b(2) EndPayloadNeg2b(2) EndPayloadNeg2(2) EndPayloadNeg1(2) EndPayloadNeg1b(2) ];
Zall_1c= [EndPayloadNeg1b(3) EndPayloadNeg2b(3) EndPayloadNeg2(3) EndPayloadNeg1(3) EndPayloadNeg1b(3)];
if ifplot == 0
    fill3(Xall_1c,Yall_1c, Zall_1c,'y')
end
Xall_1d= [EndPayloadPos1b(1) EndPayloadPos2b(1) EndPayloadPos2(1) EndPayloadPos1(1) EndPayloadPos1b(1)];
Yall_1d= [EndPayloadPos1b(2) EndPayloadPos2b(2) EndPayloadPos2(2) EndPayloadPos1(2) EndPayloadPos1b(2) ];
Zall_1d= [EndPayloadPos1b(3) EndPayloadPos2b(3) EndPayloadPos2(3) EndPayloadPos1(3) EndPayloadPos1b(3)];
if ifplot == 0
    fill3(Xall_1d,Yall_1d, Zall_1d,'g')
end

Xall_1e= [EndPayloadNeg1(1) EndPayloadPos1(1) EndPayloadPos1b(1) EndPayloadNeg1b(1)];
Yall_1e= [EndPayloadNeg1(2) EndPayloadPos1(2) EndPayloadPos1b(2) EndPayloadNeg1b(2)];
Zall_1e= [EndPayloadNeg1(3) EndPayloadPos1(3) EndPayloadPos1b(3) EndPayloadNeg1b(3)];
if ifplot == 0
    fill3(Xall_1e,Yall_1e, Zall_1e,'m')
end

Xall_1f= [EndPayloadNeg2(1) EndPayloadPos2(1) EndPayloadPos2b(1) EndPayloadNeg2b(1)];
Yall_1f= [EndPayloadNeg2(2) EndPayloadPos2(2) EndPayloadPos2b(2) EndPayloadNeg2b(2)];
Zall_1f= [EndPayloadNeg2(3) EndPayloadPos2(3) EndPayloadPos2b(3) EndPayloadNeg2b(3)];
if ifplot == 0
    fill3(Xall_1f,Yall_1f, Zall_1f,'r')
end

Prism.side(1).X = Xall_1;
Prism.side(1).Y = Yall_1;
Prism.side(1).Z = Zall_1;

Prism.side(2).X = Xall_1b;
Prism.side(2).Y = Yall_1b;
Prism.side(2).Z = Zall_1b;

Prism.side(3).X = Xall_1c;
Prism.side(3).Y = Yall_1c;
Prism.side(3).Z = Zall_1c;

Prism.side(4).X = Xall_1d;
Prism.side(4).Y = Yall_1d;
Prism.side(4).Z = Zall_1d;

Prism.side(5).X = Xall_1e;
Prism.side(5).Y = Yall_1e;
Prism.side(5).Z = Zall_1e;

Prism.side(6).X = Xall_1f;
Prism.side(6).Y= Yall_1f;
Prism.side(6).Z = Zall_1f;

if ifplot == 0
    xlabel('x')
    ylabel('y')
    zlabel('z')
end