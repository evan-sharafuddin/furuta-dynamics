% Furuta Pendulum Dynamics -- ECEN 5458
% Refer to the following two papers for a full derivation
%   [1] https://onlinelibrary.wiley.com/doi/epdf/10.1155/2011/528341
%   [2] https://journals.sagepub.com/doi/epdf/10.1243/PIME_PROC_1992_206_341_02

clear
clc
close all

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
             / ( J_0_hat*J_2_hat *J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );


theta_2_ddot = ( ...
               m_2*L_1*l_2*cos(theta_2)*b_1                                      * theta_1_dot ...
             - b_2*(J_0_hat+J_2_hat*sin(theta_2)^2)                              * theta_2_dot ...
             + m_2*L_1*l_2*J_2_hat*cos(theta_2)*sin(2*theta_2)                   * theta_1_dot*theta_2_dot ...
             - (1/2)*sin(2*theta_2)*(J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2) * theta_1_dot^2 ...
             - (1/2)*m_2^2*L_1^2*l_2^2*sin(2*theta_2)                            * theta_2_dot^2 ...
             - m_2*L_1*l_2*cos(theta_2)                                          * tau_1 ...
             + J_0_hat + J_2_hat*sin(theta_2)^2                                  * tau_2 ...
             - m_2*l_2*sin(theta_2)*(J_0_hat + J_2_hat*sin(theta_2)^2)           * g ...
             )  ...
             / ( J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );

% substitute in some values 
% looking at autonomous system dynamics, so set both torques equal to zero
% NOTE NEED TO GO THROUGH AND FIGURE OUT WHICH "BASE VARIABLES" WE CAN USE TO SOLVE FOR OTHER VARIABLES (I.E., J0HAT) 
subs_array = [ ...
              theta_1      0
              theta_1_dot  0
              % theta_2      0
              % theta_2_dot  0
              J_0_hat      0.1
              J_1_hat      0.1
              J_2_hat      0.2
              m_1          0.1
              m_2          0.1
              L_1          0.1
              L_2          0.1
              l_1          0.5
              l_2          0.5
              b_1          0.1
              b_2          0.1
              tau_1        0
              tau_2        0
              g            9.81 ...
             ];


theta_1_ddot_auto = subs( ...
    theta_1_ddot, subs_array(:,1), subs_array(:,2));
theta_2_ddot_auto = subs( ...
    theta_2_ddot, subs_array(:,1), subs_array(:,2));

% verify behavior of planar system (i.e., let theta_2 = theta_2_dot = 0)

% x1 := theta_2, x2 := theta_2_dot
t2dd = matlabFunction( theta_2_ddot_auto, 'Vars', [theta_2 theta_2_dot]);

% [X1, X2] = meshgrid( -3*pi/4 : 0.3 : 3*pi/4 );
[X1, X2] = meshgrid( -2*pi : 0.3 : 2*pi );

X1DOT = X2;
X2DOT = t2dd( X1, X2 );

figure
quiver( X1, X2, X1DOT, X2DOT, 1 )

% save workspace for use in live script
save dynamics.mat