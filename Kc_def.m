function [Kc, Kc1,Kc2,Kc3] = Kc_def(p_global, theta)

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


    %% Let's solve Inverse Kinematics for our Serial Chains of the Robot
     [q1,dz] = IK_Tripton_PF_UP(Tbase1,Ttool1, p_global,1);
     [q2,dy] = IK_Tripton_PF_UP(Tbase2,Ttool2, p_global,2);
     [q3,dx] = IK_Tripton_PF_UP(Tbase3,Ttool3, p_global,3);
    
    theta1 = theta(:,1);
    theta2 = theta(:,2);
    theta3 = theta(:,3);
    %% Let's calculate Jacobians of our Serial Chains
    % Serial chain #1
    Jq1 = Jq(dz,q1,theta1,Tbase1,Ttool1,1);
    Jt1 = Jt(dz,q1,theta1,Tbase1,Ttool1,1);
    
    % Serial chain #2
    Jq2 = Jq(dy,q2,theta2,Tbase2,Ttool2,2);
    Jt2 = Jt(dy,q2,theta2,Tbase2,Ttool2,2);
    
    % Serial chain #3
    Jq3 = Jq(dx,q3,theta3,Tbase3,Ttool3,3);
    Jt3 = Jt(dx,q3,theta3,Tbase3,Ttool3,3);
    %% Let's calculate Stiffness matrixes for our Serial Chains

    Kc1= Kc_VJM(Jq1, Jt1);
    Kc2 = Kc_VJM(Jq2, Jt2);
    Kc3 = Kc_VJM(Jq3, Jt3);

    %% Let's calculate the overall Stiffness Matrix
    Kc = Kc1 + Kc2 + Kc3;
end    


