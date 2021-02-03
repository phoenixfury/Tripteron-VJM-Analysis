function [q,d] = IK_Tripton_PF_UP(Tbase,Ttool, p_global, leg_no)
%IK_TRIPTON_PF Summary of this function goes here
%% Constants
L1 = 1;
L2 = 1;
T = [eye(3) p_global' ; 0 0 0 1];
%% Transformation from global to local
if leg_no ==1
   Tplat = inv(Ty(0.1*cos(pi/6)))*inv(Tx(-0.1*sin(pi/6)));
   d = p_global(3);
elseif leg_no == 2
    Tplat = inv(Tx(0.1));
    d = p_global(2) ;
else
    Tplat = inv(Ty(-0.1*cos(pi/6)))*inv(Tx(-0.1*sin(pi/6)));
    d = p_global(1)+0.1*sin(pi/6);
end

Tloc = (Tbase\T)*Tplat/Ttool;
%Tloc = inv(Tbase)*T*inv(Ttool)*inv(Tplat);
x = Tloc(1,4);

y = Tloc(2,4);



%% Inverse kinematics

cos_q2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
sin_q2 = sqrt(1 - cos_q2^2);

q2 = atan2(sin_q2, cos_q2);

q1 = atan2(y,x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

q3 = - (q1+q2);

q = [q1, q2, q3];
end

