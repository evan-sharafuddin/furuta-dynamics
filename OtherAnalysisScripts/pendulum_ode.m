function [dx] = pendulum_ode(t, x)
    % Physical Constants
    g = 9.81;
    % L = 243.66e-3;
    L = 200e-3;
    % eta = 0.2;
    eta = .00117;

    % States
    theta = x(1);
    omega = x(2);
    
    % Dynamics
    dx = zeros(2,1);
    dx(1) = omega;
    dx(2) = -(g/L)*sin(theta) - 2*eta*omega;
    
    % Output
    % y_out = k * theta;
end