
clear
clc
% close all

load dynamics.mat

subs_array = [ ...
              % theta_1      0
              % theta_1_dot  0
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
              % tau_1        0
              tau_2        0
              g            9.81 ...
             ];


theta_1_ddot_slk = subs( ...
    theta_1_ddot, subs_array(:,1), subs_array(:,2));
theta_2_ddot_slk = subs( ...
    theta_2_ddot, subs_array(:,1), subs_array(:,2));

theta_1_ddot_slk = matlabFunction( theta_1_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1]);
theta_2_ddot_slk = matlabFunction( theta_2_ddot_slk, 'Vars', [ theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1]);

odefun = @(t, x) simulate_dynamics_func(x, 0);

[t, x] = ode45( odefun, [0 50], [0 0 1 1] );

figure
subplot(4,1,1)
plot(t, x(:,1))
subplot(4,1,2)
plot(t, x(:,2))
subplot(4,1,3)
plot(t, x(:,3))
subplot(4,1,4)
plot(t, x(:,4))


