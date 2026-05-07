function [dx,y] = furuta_grey(t,x,u,...
    m1,m2,L1,L2,Jaddl,b1,b2,Ke,Km,...
    k1,k2,k3) % these K values are for scaling the output ADC signals

theta1     = x(1);
theta2     = x(2);
i          = x(3);
theta1dot  = x(4);
theta2dot  = x(5);

% constants
Lm = 1.6e-3;
Rm = 1.47;

% derived params
l1 = L1/2;
l2 = L2/2;

J1 = 1/12*m1*L1^2;
J2 = 1/12*m2*L2^2;

J1hat = J1 + m1*l1^2;
J2hat = J2 + m2*l2^2;
J0hat = J1hat + m2*L1^2 + Jaddl;

tau = Km*i;

theta1ddot = theta_1_ddot_func( ...
    theta1,theta1dot,...
    theta2,theta2dot,...
    tau,...
    J0hat,J1hat,J2hat,...
    m1,m2,L1,L2,l1,l2,b1,b2);

theta2ddot = theta_2_ddot_func( ...
    theta1,theta1dot,...
    theta2,theta2dot,...
    tau,...
    J0hat,J1hat,J2hat,...
    m1,m2,L1,L2,l1,l2,b1,b2);

idot = (u - Rm*i - Ke*theta1dot)/Lm;

dx = [
    theta1dot
    theta1ddot
    theta2dot
    theta2ddot
    idot
];

y = [
    k1*theta1
    k2*theta2
    k3*i
];

end

















































































































% function [dx] = furuta_grey( t, x, u, ...
%     m1, ...
%     m2, ...
%     L1, ...
%     L2, ...
%     Jaddl, ...
%     b1, ...
%     b2, ...
%     Ke, ...
%     Km ...
% )
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % DEFINE PHYSICAL PARAMETERS
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %%% define free and fixed parameters
% % these are the starting values
% % m1 = 35.27e-3;
% % m2 = 44.73e-3;
% % L1 = 192.16e-3;
% % L2 = 243.66e-3;
% % Jaddl = 0.1; % need to account for the moment of inertia of the encoder assembly
% % b1 = 0.5;
% % b2 = 4.59e-4;
% % Ke = 0.0918 * (60/2/pi);
% % Km = 1.5 * Ke;
% 
% % leave center of mass at the center of each arm
% l1 = L1 / 2;
% l2 = L2 / 2;
% 
% % the following are well characterized
% Lm = 1.6e-3 ;
% Rm = 1.47; 
% 
% %%% calculate values from the free parameters
% % calculate moment of inertia of rod about COM
% J1 = 1/12 * m1 * L1^2;
% J2 = 1/12 * m2 * L2^2;
% 
% % these inertias are as defined in the paper [1]
% J1hatv = J1 + m1*l1^2;
% J2hatv = J2 + m2*l2^2;
% J0hatv = J1hatv + m2*L1^2 + Jaddl ;
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % DEFINE **NONLINEAR** PENDULUM DYNAMICS 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Describe nonlinear simplified dynamics symbolically
% % NOTE difference from [1]: setting "disturbance torque" tau_2 = 0
% syms theta_1 theta_1_dot ... rotor arm angle
%      theta_2 theta_2_dot ... pendulum arm angle
%      J_0_hat ... moment of intertia (MOI) experience by motor when pendulum in suspended equilib. pos.
%      J_1_hat J_2_hat ... rotor and pendulum MOIs
%      m_1 m_2 ... rotor and pendulum masses
%      L_1 L_2 ... length of rotor and pendulum arms
%      l_1 l_2 ... distance from pivot to rotor/pendulum masse
%      b_1 b_2 ... rotor and pendulum damping coefficients
%      tau_1   ... torque from motor
%      tau_2 ...disturbance torque (just set this to zero)
%      g % gravitational constant
% 
% theta_1_ddot = ( ...
%               -J_2_hat*b_1                                           * theta_1_dot ...
%              + m_2*L_1*l_2*cos(theta_2)*b_2                          * theta_2_dot ...
%              - J_2_hat^2*sin(2*theta_2)                              * theta_1_dot*theta_2_dot ...
%              - (1/2)*J_2_hat*m_2*L_1*l_2*cos(theta_2)*sin(2*theta_2) * theta_1_dot^2 ...
%              + J_2_hat*m_2*L_1*l_2*sin(theta_2)                      * theta_2_dot^2 ...
%              + J_2_hat                                               * tau_1 ...
%              - m_2*L_1*l_2*(cos(theta_2))                            * tau_2 ...
%              + (1/2)*m_2^2*l_2^2*L_1*sin(2*theta_2)                  * g ...
%              )  ...
%              / ( J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );
% 
% 
% theta_2_ddot = ( ...
%                m_2*L_1*l_2*cos(theta_2)*b_1                                      * theta_1_dot ...
%              - b_2*(J_0_hat+J_2_hat*sin(theta_2)^2)                              * theta_2_dot ...
%              + m_2*L_1*l_2*J_2_hat*cos(theta_2)*sin(2*theta_2)                   * theta_1_dot*theta_2_dot ...
%              - (1/2)*sin(2*theta_2)*(J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2) * theta_1_dot^2 ...
%              - (1/2)*m_2^2*L_1^2*l_2^2*sin(2*theta_2)                            * theta_2_dot^2 ...
%              - m_2*L_1*l_2*cos(theta_2)                                          * tau_1 ...
%              + ( J_0_hat + J_2_hat*sin(theta_2)^2 )                              * tau_2 ...
%              - m_2*l_2*sin(theta_2)*(J_0_hat + J_2_hat*sin(theta_2)^2)           * g ...
%              )  ...
%              / ( J_0_hat*J_2_hat + J_2_hat^2*sin(theta_2)^2 - m_2^2*L_1^2*l_2^2*cos(theta_2)^2 );
% 
% % note: commented values below are states!
% subs_array = [ ...
%               % theta_1      0
%               % theta_1_dot  0
%               % theta_2      0
%               % theta_2_dot  0
%               J_0_hat      J0hatv
%               J_1_hat      J1hatv
%               J_2_hat      J2hatv
%               m_1          m1
%               m_2          m2
%               L_1          L1
%               L_2          L2
%               l_1          l1
%               l_2          l2
%               b_1          b1
%               b_2          b2
%               % tau_1        0
%               tau_2        0       % neglect disturbance torque
%               g            9.81 
%              ];
% 
% 
% theta_1_ddot_slk = subs( ...
%     theta_1_ddot, subs_array(:,1), subs_array(:,2) );
% theta_2_ddot_slk = subs( ...
%     theta_2_ddot, subs_array(:,1), subs_array(:,2) );
% 
% % NOTE ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1
% theta_1_ddot_slk = matlabFunction( theta_1_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 ] );
% % NOTE ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1
% theta_2_ddot_slk = matlabFunction( theta_2_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 ] );
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % DEFINE MOTOR DYNAMICS 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % describe motor electrical dynamics symbolically
% % NOTE: rotor dynamics are already considered using the above dynamics,
% %       resulting in torque to be a "state" while voltage is the "new input"
% % NOTE: these dynamics require voltage to be the input variable. However,
% %       if we are able to characterize what exactly the motor driver is
% %       doing, then we can use a PWM voltage signal and then simulate its
% %       effect 
% syms J_0_hat ... MOI experienced by motor rotor (TODO review assumptions in paper... is this accurate)
%      K_m ... torque coefficient
%      K_e ... back emf coefficient
%      L_m ... inductance
%      R_m ... resistance
%      V ... voltage (input)
%      i ... current 
%      theta_1 theta_1_dot ... rotor angle (same as above!)
% 
% % current state equation
% i_dot = 1 / L_m * ( V - R_m*i - K_e*theta_1_dot );
% % torque output equation
% tau = K_m*i;
% 
% subs_array = [ ...
%               % theta_1      0
%               % theta_1_dot  0
%               % i            0
%               % V            0
%               J_0_hat        J0hatv            
%               K_m            Km % [Nm/A]
%               K_e            Ke
%               L_m            Lm % [H]
%               R_m            Rm % [Ohm]
%              ];
% 
% i_dot_slk = subs( ...
%     i_dot, subs_array(:,1), subs_array(:,2) );
% i_dot_slk = matlabFunction( i_dot_slk, 'Vars', [ theta_1_dot, i, V] );
% 
% tau_slk = subs( tau, K_m, Km );
% tau_slk = matlabFunction( tau_slk, 'Vars', i );
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % SOLVE DYNAMICS
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% Calculate state variables
% % takes current ( x(3) ) as input)
% u_tau = tau_slk( x(3) );
% 
% dx = zeros( 5, 1 );
% 
% % theta_1_dot
% dx(1) = x(4);
% % theta_2_dot
% dx(2) = x(5);
% % i_dot ( ORDERING: theta_1_dot, i, V )
% dx(3) = i_dot_slk( x(4), x(3), u );
% % theta_1_ddot ( ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 )
% dx(4) = theta_1_ddot_slk( x(1), x(4), x(2), x(5), u_tau );
% % theta_2_ddot ( ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 )
% dx(5) = theta_2_ddot_slk( x(1), x(4), x(2), x(5), u_tau );
% 
% end