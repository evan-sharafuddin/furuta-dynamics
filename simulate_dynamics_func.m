function [dx] = simulate_dynamics_func(x, V)
%SIMULATE_DYNAMICS_FUNC For use in furuta.slx and as odefun
arguments (Input)
    x % states (5x1)
    V % voltage input
end

arguments (Output)
    dx
end

% x1: theta_1
% x2: theta_1_dot
% x3: theta_2
% x4: theta_2_dot
% x5: i

coder.extrinsic('evalin')
theta_1_ddot_slk = evalin('base','theta_1_ddot_slk');
theta_2_ddot_slk = evalin('base','theta_2_ddot_slk');
i_dot_slk        = evalin('base', 'i_dot_slk');
tau_slk          = evalin('base', 'tau_slk');

u_tau = tau_slk( x(5) );

dx = zeros(5, 1);
% calculate theta_1_dot
dx(1) = x(2);
% calculate theta_1_ddot
dx(2) = theta_1_ddot_slk( x(1), x(2), x(3), x(4), u_tau );
% calculate theta_2_dot
dx(3) = x(4);
% calculate theta_2_ddot
dx(4) = theta_2_ddot_slk( x(1), x(2), x(3), x(4), u_tau );
% calculate i_dot
dx(5) = i_dot_slk( x(2), x(5), V );

end