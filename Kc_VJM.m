function [Kc] = Kc_VJM(Jq, Jt)

d = 0.15; % Cylinder diameter
k0 = 1000000; % actuator stiffness
E = 7.0000e+10; % Young's modulus
G = 2.5500e+10; % shear modulus
L = 1; %Link length

%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;

% Let's get the stiffness matrixes of the robot  
[k11, k12, k21, k22] = k_cylinder(E, G, d, L, S, Iy, Iz);

% K theta matrix
Kt = [k0 zeros(1,12)
    zeros(6,1) k22 zeros(6,6)
    zeros(6,1) zeros(6,6) k22];

% Numerical solution
% SM=inv([(Jt/Kt)*Jt' Jq;  Jq' zeros(3,3)]);
% Kc=SM(1:6,1:6);

Kc = inv((Jt/Kt)*Jt') -  ((Jt/Kt)*Jt')\Jq/(Jq'/((Jt/Kt)*Jt')*Jq)*Jq'/((Jt/Kt)*Jt');
% Analytical solution
% Kc0=inv(Jt*inv(Kt)*Jt');
% % Kcq = inv(Jq'*inv(Kc0)*Jq)*Jq'*inv(Kc0);
% % Kc = Kc0-Kc0*Jq*Kcq;
% 
% Kc = Kc0 - Kc0*Jq*inv(Jq'*Kc0*Jq)*Jq'*Kc0;
end