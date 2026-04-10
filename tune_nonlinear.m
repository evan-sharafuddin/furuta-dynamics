clear; clc; close all;

% Load data
init_response = readtable("PendulumImpulseResponse.csv");

t = init_response.Time_s_;
y = init_response.Voltage_V_;

Ts = mean(diff(t));

tcrop = find( t > 1.15, 1, 'first' );
tt = t(tcrop:end);
yy = y(tcrop:end);

tt = tt - tt(1);

meanbias = mean( yy( tt > 18 ) );

yy = yy - meanbias;

figure
yyaxis left 
plot(tt, yy)
yline(0)

[t,y] = ode45(@pendulum_ode, [0 tt(end)], [1, 0]);
yyaxis right
plot(t, y(:,1))