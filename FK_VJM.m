function [T_leg, R1, R2, T1, T2] = FK_VJM(d,q,theta,Tbase,Ttool,leg_no)

%% Constants
L1 = 1;
L2 = 1;

%% Getting the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);

%% Transformation
if leg_no ==1
    Tplat = Tx(-0.1*sin(pi/6))*Ty(0.1*cos(pi/6));
elseif leg_no ==2
    Tplat = Tx(0.1) ;
else
    Tplat =Tz(-0.1*sin(pi/6))*Ty(0.1*cos(pi/6));
end

  T_leg = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Tplat*Ttool ;

T1 = Tbase * Tz(d) * Rz(q1);

T2 = T1 * Tx(L1) * Rz(q2);

R1 = T1(1:3,1:3);

R2 = T2(1:3,1:3);


end

