 function [delta_T, Kc] = ComputeDeflection( p_global, Force)
%COMPUTEDEFLECTION Summary of this function goes here
%% Constant stuff
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

%% Workspace limits
limit_x = 1.5;
limit_y = 1.5;
limit_z = 1.5;

%% Defining the Transformations of the bases and Tools
Tbase1 = eye(4);
Tbase2 = Tz(limit_z) * Rx( - pi/2 );
Tbase3 = Ty(limit_y) * Ry( pi/2) * Rz( pi) ;

Ttool1 = eye(4);
Ttool2 = Rx( - pi/2 )';
Ttool3 =  Rz( pi)' * Ry( pi/2)' ;

%% Displacement of each prismatic joint

dx = p_global(1);
dy = p_global(2);
dz = p_global(3);

%% Getting the Rotation matrices
    
[q1, dz] = IK_Tripton_PF_UP(Tbase1, Ttool1, p_global, 1);
[T_leg1, R11, R12] = FK_Tripton_PF(Tbase1, Ttool1, dz, q1, 1);

[q2, dy] = IK_Tripton_PF_UP(Tbase2,Ttool2, p_global, 2);
[T_leg2, R21, R22] = FK_Tripton_PF(Tbase2, Ttool2, dy, q2, 2);

[q3, dx] = IK_Tripton_PF_UP(Tbase3, Ttool3, p_global, 3);
[T_leg3, R31, R32] = FK_Tripton_PF(Tbase3, Ttool3, dx, q3, 3);

%% Transforming the q from local frame to the global frame

Qz_45 = [R11, zeros(3,3);
           zeros(3,3), R11];

Qx_45 = [R31, zeros(3,3);
           zeros(3,3), R31];
Qy_45 = [R21, zeros(3,3);
           zeros(3,3), R21];

Qz_67 = [R12, zeros(3,3);
           zeros(3,3), R12];

Qx_67 = [R32, zeros(3,3);
           zeros(3,3), R32];

Qy_67 = [R22, zeros(3,3);
           zeros(3,3), R22];

%% Global stiffness matrices for the 1st link 

K11_45z = Qz_45*k11*Qz_45';
K12_45z = Qz_45*k12*Qz_45';
K21_45z = Qz_45*k21'*Qz_45';
K22_45z = Qz_45*k22*Qz_45';

K11_45x = Qx_45*k11*Qx_45';
K12_45x = Qx_45*k12*Qx_45';
K21_45x = Qx_45*k21'*Qx_45';
K22_45x = Qx_45*k22*Qx_45';

K11_45y = Qy_45*k11*Qy_45';
K12_45y = Qy_45*k12*Qy_45';
K21_45y = Qy_45*k21'*Qy_45';
K22_45y = Qy_45*k22*Qy_45';

%% Global stiffness matrices for the 2nd link 

K11_67z = Qz_67*k11*Qz_67';
K12_67z = Qz_67*k12*Qz_67';
K21_67z = Qz_67*k21'*Qz_67';
K22_67z = Qz_67*k22*Qz_67';

K11_67x = Qx_67*k11*Qx_67';
K12_67x = Qx_67*k12*Qx_67';
K21_67x = Qx_67*k21'*Qx_67';
K22_67x = Qx_67*k22*Qx_67';

K11_67y = Qy_67*k11*Qy_67';
K12_67y = Qy_67*k12*Qy_67';
K21_67y = Qy_67*k21'*Qy_67';
K22_67y = Qy_67*k22*Qy_67';

%% Calculate Lamdas

[lambdaez, lambdaey, lambdaex] = lambda_e();
[lambdap34z, lambdap56z, lambdap78z,lambdap34y, lambdap56y, ...
lambdap78y,lambdap34x, lambdap56x, lambdap78x] = lambda_p();

[lambdar12z, lambdar34z, lambdar56z, lambdar78z, lambdar12y, lambdar34y, lambdar56y, lambdar78y, ...
lambdar12x, lambdar34x, lambdar56x, lambdar78x] = lambda_r();

%% Aggregation for Chain Z
d9_10 = skew([-0.1*sin(pi/6), 0.1*cos(pi/6), 0]);
D9_10 = [ eye(3), d9_10'; zeros(3), eye(3)];

Full_Matrix_Zleg = [ 
  zeros(6, 60), eye(6,6), zeros(6, 54); %eq1 rigid base 1
  
  zeros(5, 60), lambdar12z, -lambdar12z, zeros(5, 48); %eq2 elastic joint 1,2
  
  eye(6,6), eye(6,6), zeros(6,48), zeros(6,60);
  
  lambdaez, zeros(1, 54), lambdaez*k0, -lambdaez*k0, zeros(1, 48);
  
  zeros(6, 66), eye(6,6), - eye(6,6), zeros(6,42); %rigid link 2,3
  
  zeros(6,6), eye(6,6), eye(6,6), zeros(6, 42), zeros(6,60);
  
  zeros(5,60), zeros(5, 12), lambdar34z, -lambdar34z, zeros(5,36); % passive joint 3,4
  
  zeros(5,12), lambdar34z, lambdar34z, zeros(5, 36), zeros(5,60);
  
  zeros(1,12), lambdap34z, zeros(1, 17*6);
  
  zeros(1,18), lambdap34z, zeros(1, 16*6);
  
  zeros(6,18), -eye(6,6), zeros(6,36), zeros(6,18),  K11_45z, K12_45z, zeros(6,30); % Flexible link 4,5
  
  zeros(6,24), -eye(6,6), zeros(6,48),  K21_45z, K22_45z, zeros(6,30);
  
  zeros(5,60), zeros(5, 24), lambdar56z, -lambdar56z, zeros(5,24); % passive joint 5,6
  
  zeros(5,24), lambdar56z, lambdar56z, zeros(5, 24), zeros(5,60);
  
  zeros(1,24), lambdap56z, zeros(1, 15*6);
  
  zeros(1,30), lambdap56z, zeros(1, 14*6);
  
  zeros(6,30), -eye(6,6), zeros(6,24), zeros(6,30),  K11_67z, K12_67z, zeros(6,18); % Flexible link 6,7
  
  zeros(6,36), -eye(6,6), zeros(6,48),  K21_67z, K22_67z, zeros(6,18);
  
  zeros(5,60), zeros(5, 36), lambdar78z, -lambdar78z, zeros(5,12); % passive joint 7,8
  
  zeros(5,36), lambdar78z, lambdar78z, zeros(5, 12), zeros(5,60);
  
  zeros(1,36), lambdap78z, zeros(1, 13*6);
  
  zeros(1,42), lambdap78z, zeros(1, 12*6);
  
  zeros(6, 102), eye(6,6), - eye(6,6), zeros(6,6); %rigid link 8,9
  
  zeros(6,42), eye(6,6), eye(6,6), zeros(6, 6), zeros(6,60);
  
  zeros(6, 108),  D9_10, - eye(6,6); %rigid link 9,10
  
  zeros(6,48), eye(6,6),D9_10', zeros(6,60);
  
  zeros(6,54), - eye(6,6), zeros(6,60); ];% external force 
  
Az = Full_Matrix_Zleg(1:114, 1:114);

Bz = Full_Matrix_Zleg(1:114, 115:120);

Cz = Full_Matrix_Zleg(115:120, 1:114);

Dz = Full_Matrix_Zleg(115:120, 115:120);

Kcz =  Dz - Cz*(Az\Bz);
%% Aggregation for Chain Y
d9_10y = skew([0.1, 0, 0]);
D9_10y = [ eye(3), d9_10y'; zeros(3), eye(3)];

Full_Matrix_yleg = [ 
  zeros(6, 60), eye(6,6), zeros(6, 54); %eq1 rigid base 1
  
  zeros(5, 60), lambdar12y, -lambdar12y, zeros(5, 48); %eq2 elastic joint 1,2
  
  eye(6,6), eye(6,6), zeros(6,48), zeros(6,60);
  
  lambdaey, zeros(1, 54), lambdaey*k0, -lambdaey*k0, zeros(1, 48);
  
  zeros(6, 66), eye(6,6), - eye(6,6), zeros(6,42); %rigid link 2,3
  
  zeros(6,6), eye(6,6), eye(6,6), zeros(6, 42), zeros(6,60);
  
  zeros(5,60), zeros(5, 12), lambdar34y, -lambdar34y, zeros(5,36); % passive joint 3,4
  
  zeros(5,12), lambdar34y, lambdar34y, zeros(5, 36), zeros(5,60);
  
  zeros(1,12), lambdap34y, zeros(1, 17*6);
  
  zeros(1,18), lambdap34y, zeros(1, 16*6);
  
  zeros(6,18), -eye(6,6), zeros(6,36), zeros(6,18),  K11_45y, K12_45y, zeros(6,30); % Flexible link 4,5
  
  zeros(6,24), -eye(6,6), zeros(6,48),  K21_45y, K22_45y, zeros(6,30);
  
  zeros(5,60), zeros(5, 24), lambdar56y, -lambdar56y, zeros(5,24); % passive joint 5,6
  
  zeros(5,24), lambdar56y, lambdar56y, zeros(5, 24), zeros(5,60);
  
  zeros(1,24), lambdap56y, zeros(1, 15*6);
  
  zeros(1,30), lambdap56y, zeros(1, 14*6);
  
  zeros(6,30), -eye(6,6), zeros(6,24), zeros(6,30),  K11_67y, K12_67y, zeros(6,18); % Flexible link 6,7
  
  zeros(6,36), -eye(6,6), zeros(6,48),  K21_67y, K22_67y, zeros(6,18);
  
  zeros(5,60), zeros(5, 36), lambdar78y, -lambdar78y, zeros(5,12); % passive joint 7,8
  
  zeros(5,36), lambdar78y, lambdar78y, zeros(5, 12), zeros(5,60);
  
  zeros(1,36), lambdap78y, zeros(1, 13*6);
  
  zeros(1,42), lambdap78y, zeros(1, 12*6);
  
  zeros(6, 102), eye(6,6), - eye(6,6), zeros(6,6); %rigid link 8,9
  
  zeros(6,42), eye(6,6), eye(6,6), zeros(6, 6), zeros(6,60);
  
  zeros(6, 108),  D9_10y, - eye(6,6); %rigid link 9,10
  
  zeros(6,48), eye(6,6),D9_10y', zeros(6,60);
  
  zeros(6,54), - eye(6,6), zeros(6,60); ];% external force 
  
Ay = Full_Matrix_yleg(1:114, 1:114);

By = Full_Matrix_yleg(1:114, 115:120);

Cy = Full_Matrix_yleg(115:120, 1:114);

Dy = Full_Matrix_yleg(115:120, 115:120);


Kcy =  Dy - Cy*(Ay\By);
%% Aggregation for Chain X
d9_10x = skew([-0.1*sin(pi/6), -0.1*cos(pi/6), 0]);

D9_10x = [ eye(3), d9_10x'; zeros(3), eye(3)];

Full_Matrix_xleg = [ 
  zeros(6, 60), eye(6,6), zeros(6, 54); %eq1 rigid base 1
  
  zeros(5, 60), lambdar12x, -lambdar12x, zeros(5, 48); %eq2 elastic joint 1,2
  
  eye(6,6), eye(6,6), zeros(6,48), zeros(6,60);
  
  lambdaex, zeros(1, 54), lambdaex*k0, -lambdaex*k0, zeros(1, 48);
  
  zeros(6, 66), eye(6,6), - eye(6,6), zeros(6,42); %rigid link 2,3
  
  zeros(6,6), eye(6,6), eye(6,6), zeros(6, 42), zeros(6,60);
  
  zeros(5,60), zeros(5, 12), lambdar34x, -lambdar34x, zeros(5,36); % passive joint 3,4
  
  zeros(5,12), lambdar34x, lambdar34x, zeros(5, 36), zeros(5,60);
  
  zeros(1,12), lambdap34x, zeros(1, 17*6);
  
  zeros(1,18), lambdap34x, zeros(1, 16*6);
  
  zeros(6,18), -eye(6,6), zeros(6,36), zeros(6,18),  K11_45x, K12_45x, zeros(6,30); % Flexible link 4,5
  
  zeros(6,24), -eye(6,6), zeros(6,48),  K21_45x, K22_45x, zeros(6,30);
  
  zeros(5,60), zeros(5, 24), lambdar56x, -lambdar56x, zeros(5,24); % passive joint 5,6
  
  zeros(5,24), lambdar56x, lambdar56x, zeros(5, 24), zeros(5,60);
  
  zeros(1,24), lambdap56x, zeros(1, 15*6);
  
  zeros(1,30), lambdap56x, zeros(1, 14*6);
  
  zeros(6,30), -eye(6,6), zeros(6,24), zeros(6,30),  K11_67x, K12_67x, zeros(6,18); % Flexible link 6,7
  
  zeros(6,36), -eye(6,6), zeros(6,48),  K21_67x, K22_67x, zeros(6,18);
  
  zeros(5,60), zeros(5, 36), lambdar78x, -lambdar78x, zeros(5,12); % passive joint 7,8
  
  zeros(5,36), lambdar78x, lambdar78x, zeros(5, 12), zeros(5,60);
  
  zeros(1,36), lambdap78x, zeros(1, 13*6);
  
  zeros(1,42), lambdap78x, zeros(1, 12*6);
  
  zeros(6, 102), eye(6,6), - eye(6,6), zeros(6,6); %rigid link 8,9
  
  zeros(6,42), eye(6,6), eye(6,6), zeros(6, 6), zeros(6,60);
  
  zeros(6, 108),  D9_10x, - eye(6,6); %rigid link 9,10
  
  zeros(6,48), eye(6,6),D9_10x', zeros(6,60);
  
  zeros(6,54), - eye(6,6), zeros(6,60); ];% external force 
  
Ax = Full_Matrix_xleg(1:114, 1:114);

Bx = Full_Matrix_xleg(1:114, 115:120);

Cx = Full_Matrix_xleg(115:120, 1:114);

Dx = Full_Matrix_xleg(115:120, 115:120);
Kcx =  Dx - Cx*(Ax\Bx);
%% Calculate the whole stiffness matrix
Kc = Kcx+Kcy+Kcz;

%% delta_T
delta_T = Kc\ Force; 
end

