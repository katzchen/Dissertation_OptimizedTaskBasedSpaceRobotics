function Tpayload_grasping = FindJe_rev2(grippingpointalongPayloadLength_x,grippingpointalongPayloadLength_y,grippingpointalongPayloadLength_z, Arm_RPY_relativetoPayload,Payload_RPY,Payload_XYZ)

%OUTPUT: This should give the force transformation matrices for an arm in
%GLOBAL frame

%OKay. Here's the deal
%I want to put everything in the GLOBAL frame for the constraint function
%Notes: The rotation between the GRASPING frame and the PAYLOAD frame are not
%necessarily the same, it depends on the ARM_RPY_relativetoPayload


%Tfinal_original = [Rfinal_original zeros(3); skew(Pfinal_original)*Rfinal_original Rfinal_original]

%first do Grasping point ot the Payload: THIS IS IN WHAT FRAME? IN EE FRAME
%Arm_RPY_relativetoPayload is to rotate the Payload into Arm coordinates
x = grippingpointalongPayloadLength_x;
y = grippingpointalongPayloadLength_y;
z = grippingpointalongPayloadLength_z;

%We have the r vector. Its the [x y z] in the payload frame
%Porg_b x r

Ppayload_grasping = [0 -z y;z 0 -x;-y x 0]; %IN PAYLAOD FRAME
[Rpayload_grasping]= (RotatiomMatrix(Arm_RPY_relativetoPayload));
Tpayload_grasping = [Rpayload_grasping zeros(3); Ppayload_grasping*Rpayload_grasping Rpayload_grasping];

%Now from payload to GLOBAL
%I can make this actually the base frame... is that what I want? Lets try
%it? 

%3/8/18 I think I want this all relative to the Center of the PAYLOAD NOT
%GLOBAL? I want it rotated to global but the translation shouldn't impact
%my moments because that should ONLY be relative to the 
x = Payload_XYZ(1)*0;
y = Payload_XYZ(2)*0;
z = Payload_XYZ(3)*0;
Pglobal_payload = [0 -z y;z 0 -x;-y x 0];
[Rglobal_payload]= (RotatiomMatrix(Payload_RPY));
Tglobal_payload = [Rglobal_payload zeros(3); Pglobal_payload*Rglobal_payload Rglobal_payload];
Tglobal_grasping = (Tglobal_payload*Tpayload_grasping);
