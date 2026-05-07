
define_dynamics;


%% run simulation

sim = ss(Abig_s, Bbig_s, Cbig_s, Dbig_s);
simol = ss( A_s, B_s, C_s, D_s );
t = 0:0.01:10;
u = 1 - 2*(mod(t,2) >= 1);
% y = lsim( sim, zeros(size(t)), t, [0 0.1 0 0 0 0 0 0 0 0].' );
y = lsim( simol, u, t, [0 0 0 0 0].' );
% y = lsim( simol, zeros(size(t)), t, [0 0.1 0 0 0].' );

figure, plot(t, y(:,1))
figure, plot(t, y(:,2))
figure, plot(t, y(:,3))


%%
close all
t = 0:0.01:10;
DC = 12.5;
amp = 12 * (DC/100);

u_fun = @(tt) amp * ( 1 - 2*(mod(tt,2) >= 1) );
x0 = [0 0 0 0 0];
[t, y] = ode45(@(tt,y) simulate_dynamics_func(y, u_fun(tt)), [t(1) t(end)], x0);

figure
subplot(4,1,1)
plot(t, u_fun(t)), title("input")
subplot(4,1,2)
plot(t, y(:,1)), title("motor angle")
subplot(4,1,3)
plot(t, y(:,2)), title("pendulum angle")
subplot(4,1,4)
plot(t, y(:,3)), title("current")

% x0 = [0 0.1 0 0 0];
% [t, y] = ode45(@(tt,y) simulate_dynamics_func(y, 0), [t(1) t(end)], x0);



% init_response = readtable("PendulumImpulseResponse.csv");
% step_response = readtable("MotorRepeatedStepResponseDutyCycle50Percent.csv");
% t2 = step_response.Time_s_;
% y2 = step_response.Voltage_V_;
% 
% t1 = init_response.Time_s_;
% y1 = init_response.Voltage_V_;
% 
% figure, subplot(2,1,1), plot(t, y(:,1))
% subplot(2,1,2), plot(t2, movmean(y2, 1e3))
% figure; subplot(2,1,1),plot(t, y(:,2))
% subplot(2,1,2), plot( t1, y1)
% figure, plot(t, y(:,3))
