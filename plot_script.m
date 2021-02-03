clc;
limit_x = 1.3;
limit_y = 1.3;
limit_z = 1.3;

%% Defining the Transformations of the bases and Tools
Tbase1 = eye(4);
Tbase2 = Tz(limit_z) * Rx( - pi/2 );
Tbase3 = Ty(limit_y) * Ry( pi/2) * Rz( pi) ;

Ttool1 = eye(4);
Ttool2 = inv(Rx( - pi/2 ));
Ttool3 = inv(Ry( pi/2)) * inv(Rz( pi)) ;

%% forward and inverse kinematics
% 
% p_global = [0.75, 0.12, 0.25];
% q1 = IK_Tripton(Tbase1, p_global);
% 
% q2 = IK_Tripton(Tbase2, p_global);
% 
% q3 = IK_Tripton(Tbase3, p_global);
% 
% [Tleg1, R11, R12, T11, T12] = FK_Tripton(Tbase1, Ttool1, p_global(3), q1);
% 
% [Tleg2, R21, R22, T21, T22] = FK_Tripton(Tbase2, Ttool2, p_global(2), q2);
% 
% [Tleg3, R31, R32, T31, T32] = FK_Tripton(Tbase3, Ttool3, p_global(1), q3);


%% forward and inverse kinematics 2

p_global = [.1, 1.3, .1];
[q1,dz] = IK_Tripton_PF_UP(Tbase1,Ttool1, p_global, 1);

[q2,dy] = IK_Tripton_PF_UP(Tbase2,Ttool2, p_global, 2 );

[q3,dx] = IK_Tripton_PF_UP(Tbase3,Ttool3 , p_global, 3);

[Tleg1, R11, R12, T11, T12, T13] = FK_Tripton_PF(Tbase1, Ttool1, dz, q1, 1 );

[Tleg2, R21, R22, T21, T22, T23] = FK_Tripton_PF(Tbase2, Ttool2, dy, q2, 2);

[Tleg3, R31, R32, T31, T32, T33] = FK_Tripton_PF(Tbase3, Ttool3, dx, q3, 3);

%% plotting
PlotRobot(Tleg1,T11, T12, T13, Tbase1, 'blue')
PlotRobot(Tleg2,T21, T22, T23, Tbase2, 'black')
PlotRobot(Tleg3,T31, T32, T33, Tbase3,'green')
