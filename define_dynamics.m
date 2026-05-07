% Furuta Pendulum Dynamics -- ECEN 5458
% Refer to the following two papers for a full derivation
%   [1] https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
%   [2] https://journals.sagepub.com/doi/epdf/10.1243/PIME_PROC_1992_206_341_02

clear
clc
close all
format compact

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE PHYSICAL PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F_USEREAL = true; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Values from [1]
% if ~F_USEREAL
% J1 = 2.48e-2;
% J2 = 3.86e-3;
% m1 = 0.3;
% m2 = 0.075;
% l1 = 0.150;
% l2 = 0.148;
% L1 = 0.278;
% L2 = 0.3;
% J1hatv = J1 + m1*l1^2;
% J2hatv = J2 + m2*l2^2;
% J0hatv = J1hatv + m2*L1^2 ;
% b1 = 1e-4;
% b2 = 2.80e-4;
% Km = 0.090; % [Nm/A]
% Ke = Km;
% Ke = 1;
% Lm = 0.005; % [H]
% Rm = 7.800; % [Ohm]
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Values from Donavon :)

m1 = 35.27e-3 ;
m2 = 44.73e-3;
% m2 = 70e-3;
% mtotal = 80e-3;
% m1percent = 0.43;
% m1 = mtotal * m1percent;
% m2 = mtotal * (1 - m1percent);
% m1 = 1000e-3;
% m2 = 44.73e-3;
L1 = 192.16e-3 ;
L2 = 243.66e-3 ;
% L2 = 210e-3;

% assume mass is distributed evenly along arms, so COM is in middle
l1 = L1 / 2;
l2 = L2 / 2;
% l2 = L2 / 1.5;

% calculate moment of inertia of rod about COM
J1 = 1/12 * m1 * L1^2;
J2 = 1/12 * m2 * L2^2;

% these inertias are as defined in the paper [1]
Jaddl = 0.1; % need to account for the moment of inertia of the encoder assembly
J1hatv = J1 + m1*l1^2;
J2hatv = J2 + m2*l2^2;
J0hatv = J1hatv + m2*L1^2 + Jaddl ;

% dampening (b1 is a guess, but it should be quite high)
% b1 = 0.5;
% b1 = 10;
% b1 = 0.0122;
% b2 = 4.59e-4;
% b2 = 0.0005;
b1 = .807;
b2 = .00117;

% next, the motor parameters
Lm = 1.6e-3 ;
% Lm = 16e-3 * 1e-3; % scaled for mA
Rm = 1.47; 
% Rm = 1.47 * 1e-3; % scaled for mA

% reduce RL dynamics
% Lm = 1.6e-1;
% Rm = 10;

% Donavon was getting different values for back EMF and torque constants,
% so these are incorporated separately in the model... however, IDEALLY
% they should be the same
Ke = 0.0918 * (60/2/pi);
Km = 1.5 * Ke;
% Km = 1.2 * 0.25;
% Km = 0.25 * 1e-3; % scaled for mA 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE **NONLINEAR** PENDULUM DYNAMICS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Describe nonlinear simplified dynamics symbolically
% NOTE difference from [1]: setting "disturbance torque" tau_2 = 0
syms theta_1 theta_1_dot ... rotor arm angle
     theta_2 theta_2_dot ... pendulum arm angle
     J_0_hat ... moment of intertia (MOI) experience by motor when pendulum in suspended equilib. pos.
     J_1_hat J_2_hat ... rotor and pendulum MOIs
     m_1 m_2 ... rotor and pendulum masses
     L_1 L_2 ... length of rotor and pendulum arms
     l_1 l_2 ... distance from pivot to rotor/pendulum masse
     b_1 b_2 ... rotor and pendulum damping coefficients
     tau_1   ... torque from motor
     tau_2 ...disturbance torque (just set this to zero)
     g % gravitational constant

theta_1_ddot = ( ...
              -J_2_hat*b_1                                           * theta_1_dot ...
             + m_2*L_1*l_2*cos(theta_2)*b_2                          * theta_2_dot ...
             - J_2_hat^2*sin(2*theta_2)                              * theta_1_dot*theta_2_dot ...
             - (1/2)*J_2_hat*m_2*L_1*l_2*cos(theta_2)*sin(2*theta_2) * theta_1_dot^2 ...
             + J_2_hat*m_2*L_1*l_2*sin(theta_2)                      * theta_2_dot^2 ...
             + J_2_hat                                               * tau_1 ...
             - m_2*L_1*l_2*(cos(theta_2))                            * tau_2 ...
             + (1/2)*m_2^2*l_2^2*L_1*sin(2*theta_2)                  * g ...
             )  ...
             / ( J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );


theta_2_ddot = ( ...
               m_2*L_1*l_2*cos(theta_2)*b_1                                      * theta_1_dot ...
             - b_2*(J_0_hat+J_2_hat*sin(theta_2)^2)                              * theta_2_dot ...
             + m_2*L_1*l_2*J_2_hat*cos(theta_2)*sin(2*theta_2)                   * theta_1_dot*theta_2_dot ...
             - (1/2)*sin(2*theta_2)*(J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2) * theta_1_dot^2 ...
             - (1/2)*m_2^2*L_1^2*l_2^2*sin(2*theta_2)                            * theta_2_dot^2 ...
             - m_2*L_1*l_2*cos(theta_2)                                          * tau_1 ...
             + ( J_0_hat + J_2_hat*sin(theta_2)^2 )                              * tau_2 ...
             - m_2*l_2*sin(theta_2)*(J_0_hat + J_2_hat*sin(theta_2)^2)           * g ...
             )  ...
             / ( J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );

% note: commented values below are states!
subs_array = [ ...
              % theta_1      0
              % theta_1_dot  0
              % theta_2      0
              % theta_2_dot  0
              J_0_hat      J0hatv
              J_1_hat      J1hatv
              J_2_hat      J2hatv
              m_1          m1
              m_2          m2
              L_1          L1
              L_2          L2
              l_1          l1
              l_2          l2
              b_1          b1
              b_2          b2
              % tau_1        0
              tau_2        0       % neglect disturbance torque
              g            9.81 
             ];


theta_1_ddot_slk = subs( ...
    theta_1_ddot, subs_array(:,1), subs_array(:,2) );
theta_2_ddot_slk = subs( ...
    theta_2_ddot, subs_array(:,1), subs_array(:,2) );

% NOTE ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1
theta_1_ddot_slk = matlabFunction( theta_1_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 ] );
% NOTE ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1
theta_2_ddot_slk = matlabFunction( theta_2_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 ] );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE MOTOR DYNAMICS 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% describe motor electrical dynamics symbolically
% NOTE: rotor dynamics are already considered using the above dynamics,
%       resulting in torque to be a "state" while voltage is the "new input"
% NOTE: these dynamics require voltage to be the input variable. However,
%       if we are able to characterize what exactly the motor driver is
%       doing, then we can use a PWM voltage signal and then simulate its
%       effect 
syms J_0_hat ... MOI experienced by motor rotor (TODO review assumptions in paper... is this accurate)
     K_m ... torque coefficient
     K_e ... back emf coefficient
     L_m ... inductance
     R_m ... resistance
     V ... voltage (input)
     i ... current 
     theta_1 theta_1_dot ... rotor angle (same as above!)

% current state equation
i_dot = 1 / L_m * ( V - R_m*i - K_e*theta_1_dot );
% torque output equation
tau = K_m*i;

subs_array = [ ...
              % theta_1      0
              % theta_1_dot  0
              % i            0
              % V            0
              J_0_hat        J0hatv            
              K_m            Km % [Nm/A]
              K_e            Ke
              L_m            Lm % [H]
              R_m            Rm % [Ohm]
             ];

i_dot_slk = subs( ...
    i_dot, subs_array(:,1), subs_array(:,2) );
i_dot_slk = matlabFunction( i_dot_slk, 'Vars', [ theta_1_dot, i, V] );

tau_slk = subs( tau, K_m, Km );
tau_slk = matlabFunction( tau_slk, 'Vars', i );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE **LINEARIZED** DYNAMICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define matrix terms
A31 = 0;
A32 = g*m_2^2*l_2^2*L_1 / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
A33 = -b_1*J_2_hat      / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
A34 = -b_2*m_2*l_2*L_1  / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
A41 = 0;
A42 = g*m_2*l_2*J_0_hat / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
A43 = -b_1*m_2*l_2*L_1  / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
A44 = -b_2*J_0_hat      / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);

B31 = J_2_hat     / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
B41 = m_2*L_1*l_2 / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
B32 = m_2*L_1*l_2 / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);
B42 = J_0_hat     / (J_0_hat*J_2_hat - m_2^2*L_1^2*l_2^2);

% write inverted linearization
% ignoring dynamics for "disturbance torque"
A_i = [ 0    0    0        1         0   ;
        0    0    0        0         1   ;
        0    0   -R_m/L_m -K_e/L_m   0   ;
        A31  A32  B31*K_m  A33       A34 ;
        A41  A42  B41*K_m  A43       A44 ];
B_i = [ 0     ; 
        0     ; 
        1/L_m ; 
        0     ; 
        0     ];
C_i = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 1 0 0 ];
D_i = [ 0 ;
        0 ;
        0 ];

A_s = [ 0    0    0        1         0   ;
        0    0    0        0         1   ;
        0    0   -R_m/L_m -K_e/L_m   0   ;
        A31  A32  B31*K_m  A33      -A34 ;
        A41 -A42 -B41*K_m -A43       A44 ];
B_s = [ 0     ; 
        0     ; 
        1/L_m ; 
        0     ; 
        0     ];
C_s = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 1 0 0 ];
D_s = [ 0 ;
        0 ;
        0 ];

% note: commented values below are states and inputs!
subs_array = [ ...
              % theta_1      0
              % theta_1_dot  0
              % theta_2      0
              % theta_2_dot  0
              J_0_hat      J0hatv
              J_1_hat      J1hatv
              J_2_hat      J2hatv
              m_1          m1
              m_2          m2
              L_1          L1
              L_2          L2
              l_1          l1
              l_2          l2
              b_1          b1
              b_2          b2
              % tau_1        0
              tau_2        0       % neglect disturbance torque
              g            9.81 
              % i            0
              % V            0
              K_m            Km % [Nm/A]
              K_e            Ke
              L_m            Lm % [H]
              R_m            Rm % [Ohm]
             ];

A_i = double( subs( A_i, subs_array(:,1), subs_array(:,2) ) );
B_i = double( subs( B_i, subs_array(:,1), subs_array(:,2) ) );
A_s = double( subs( A_s, subs_array(:,1), subs_array(:,2) ) );
B_s = double( subs( B_s, subs_array(:,1), subs_array(:,2) ) );
 
% C and D matrices are the same for both configurations
C = eye( 5, 5 ); % state feedback for now
D = zeros( 5, 1 );

% save dynamics in mat file
save dynamics.mat

lambda = eig( A_s );
fprintf("Eigenvalues of A suspended OL:\n")
disp(lambda)

lambda = eig( A_i );
fprintf("Eigenvalues of A inverted OL:\n")
disp(lambda)

fprintf("Running simulink companion file...\n")
create_suspended_ctrl
% create_inverted_ctrl
