function [dx] = simulate_dynamics_func(x, V)
%SIMULATE_DYNAMICS_FUNC For use in furuta.slx and as odefun
arguments (Input)
    x % states (5x1)
    V % voltage input
end

arguments (Output)
    dx
end

% OLD
% x1: theta_1
% x2: theta_1_dot
% x3: theta_2
% x4: theta_2_dot
% x5: i

% NEW
% x1: theta_1
% x2: theta_2
% x3: i
% x4: theta_1_dot
% x5: theta_2_dot

coder.extrinsic('evalin')
theta_1_ddot_slk = evalin('base','theta_1_ddot_slk');
theta_2_ddot_slk = evalin('base','theta_2_ddot_slk');
i_dot_slk        = evalin('base', 'i_dot_slk');
tau_slk          = evalin('base', 'tau_slk');

% takes current ( x(3) ) as input)
u_tau = tau_slk( x(3) );

% dx = zeros(5, 1);
% % calculate theta_1_dot
% dx(1) = x(2);
% % calculate theta_1_ddot
% dx(2) = theta_1_ddot_slk( x(1), x(2), x(3), x(4), u_tau );
% % calculate theta_2_dot
% dx(3) = x(4);
% % calculate theta_2_ddot
% dx(4) = theta_2_ddot_slk( x(1), x(2), x(3), x(4), u_tau );
% % calculate i_dot
% dx(5) = i_dot_slk( x(2), x(5), V );

dx = zeros( 5, 1 );

% theta_1_dot
dx(1) = x(4);
% theta_2_dot
dx(2) = x(5);
% i_dot ( ORDERING: theta_1_dot, i, V )
dx(3) = i_dot_slk( x(4), x(3), V );
% theta_1_ddot ( ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 )
dx(4) = theta_1_ddot_slk( x(1), x(4), x(2), x(5), u_tau );
% theta_2_ddot ( ORDERING: theta_1, theta_1_dot, theta_2, theta_2_dot, tau_1 )
dx(5) = theta_2_ddot_slk( x(1), x(4), x(2), x(5), u_tau );

end