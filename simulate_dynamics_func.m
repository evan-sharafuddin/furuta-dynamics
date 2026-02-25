function [dx] = simulate_dynamics_func(x, u_tau)
%SIMULATE_DYNAMICS_FUNC Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x
    u_tau
end

arguments (Output)
    dx
end

% x1: theta_1
% x2: theta_1_dot
% x3: theta_2
% x4: theta_2_dot

coder.extrinsic('evalin')
theta_1_ddot_slk = evalin('base','theta_1_ddot_slk');
theta_2_ddot_slk = evalin('base','theta_2_ddot_slk');

dx = zeros(4, 1);

% calculate theta_1_dot
dx(1) = x(2);
% calculate theta_1_ddot
dx(2) = theta_1_ddot_slk( x(1), x(2), x(3), x(4), u_tau );
% calculate theta_2_dot
dx(3) = x(4);
% calculate theta_2_ddot
dx(4) = theta_2_ddot_slk( x(1), x(2), x(3), x(4), u_tau );

end