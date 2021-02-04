function [T_leg, R1, R2, T1, T2, T3] = FK_Tripton_PF(Tbase, Ttool, d, q, leg_no)
%FK_TRIPTON_PF Summary of this function goes here
 
%FK_TRIPTON Summary of this function goes here

%% Constants
L1 = 1;
L2 = 1;

%% Getting the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);

%% Transformation
if leg_no ==1
    T_leg = Tbase * Tz(d) * Rz(q1) * Tx(L1) * Rz(q2) * Tx(L2) * Rz(q3) *Ttool*Tx(-0.1*sin(pi/6))*Ty(0.1*cos(pi/6))  ;
elseif leg_no ==2
    T_leg = Tbase * Tz(d) * Rz(q1) * Tx(L1) * Rz(q2) * Tx(L2) * Rz(q3)* Ttool *Tx(0.1) ;
else
    T_leg = Tbase * Tz(d) * Rz(q1) * Tx(L1) * Rz(q2) * Tx(L2) * Rz(q3)*Ttool*Tx(-0.1*sin(pi/6))*Ty(-0.1*cos(pi/6)) ;
end

T1 = Tbase * Tz(d) * Rz(q1);

T2 = T1 * Tx(L1) * Rz(q2);

T3 = T2 * Tx(L2) * Rz(q3);

R1 = T1(1:3,1:3);

R2 = T2(1:3,1:3);

end

