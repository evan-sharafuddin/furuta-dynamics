
clear
clc
% close all

% use ode45 to simulate differential equation

load dynamics.mat

odefun = @(t, x) simulate_dynamics_func(x, 0);

[t, x] = ode45( odefun, [0 10], [0 pi/10 0 0 0 ] );

figure
subplot(5,1,1)
plot(t, x(:,1))
subplot(5,1,2)
plot(t, x(:,2))
subplot(5,1,3)
plot(t, x(:,3))
subplot(5,1,4)
plot(t, x(:,4))
subplot(5,1,5)
plot(t, x(:,5))

