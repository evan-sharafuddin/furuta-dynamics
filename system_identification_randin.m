clear
clc 
close all

%{ 

---Evan Notes---
* added term to J_hat_0 for the encoder housing. this makes the current
signal as well as the motor angle signals much more accurate looking
compared to the experimental data
* there is some nonlinearity that we are not modelling with the motor. When
you look at the final value for the rotor angle, it is negative, while for
the simulated it is at zero. Based on the values for the input, you would
expect to end at zero. For some reason, the motor must have a different Km
or Ke going in one direction vs another, or at least that's how I'm
interpreting this. Not much control we have over that...
* IMPORTANT. had to change the sign of the pendulum angle to have it match
the first portion of the signal. Not sure if there is still a sign error in
the code somewhere
* It is very, very hard to perfectly match up the pendulum angle. Things
I've tried
    * adjusting damping (b2)
    * changing L2 and l2. One (or both) of these parameters seems to change
    the natural frequency of the pendulum arm. I was focused on matching up
    the last part where it is swinging freely. This is also a good location
    for adjusting b2
    * changing mass (m2) seems to negate some of the damping changes, so
    this should be changed in parallel with b2
    * the pendulum angle is very very sensitive to the timing of the
    direction change. Conceptually, if the direction change is moving in
    the same direction as the pendulum, we get amplificaiton. otherwise, we
    get attenuation. This can be seen when comparing the two pendulum
    signals
    * some of the differences could also be with the motor noninearity,
    which is causing some reduced motion in the rotor arm when moving in
    the forward direction.
* left Lm and Rm alone entirely. Changed Ke and Km a little bit but not
much, if at all
%}

% load dynamics
define_dynamics;

% read input
fid = fopen('scope_14.bin', 'rb', 'ieee-le');
raw = fread(fid, 'float32');
fclose(fid);

raw = raw( 29:end );
raw = raw( 1:2800000 );

% N1 = find( raw < -1e10, 1, 'first' );
% N2 = N1 + find( raw( N1+1:end ) < 1e-10, 1, 'first' );
% N3 = N2 + find( raw( N2+1:end ) < 1e-10, 1, 'first' );

N = floor(length(raw)/4);    % samples per channel
ch1 = raw(1:N);
ch2 = raw(N+1:2*N);
ch3 = raw(2*N+1:3*N);
ch4 = raw(3*N+1:4*N);

% ch1 = raw(1:(N1-1));
% ch2 = raw((N1+1):(N2-1));
% ch3 = raw((N2+1):(N3-1));
% ch4 = raw((N3+1):end);

% figure
% subplot(4,1,1), plot(ch1), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
% subplot(4,1,2), plot(ch2), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
% subplot(4,1,3), plot(ch3), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
% subplot(4,1,4), plot(ch4), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )

% u = ch1;
% x1 = ch2;
% x2 = ch3;
% x3 = ch4;

% % 20 ksamples/sec
% T = 1/20e3;
% t = ( 0 : length(u) - 1 ) * T;

function [] = plotx(~)

xline(96000)
xline(116000)
xline(136000)
xline(156000)
xline(176000)
xline(196000)
xline(216000)
xline(235500)
xline(255500)
xline(275500)
xline(295000)
xline(315000)
xline(334500)
xline(354500)
xline(374000)
xline(394000)
xline(413500)
xline(433500)
xline(453500)
xline(473000)
xline(492000)

end

vals = [0, 0, 0, -3, 5, -4, 3, -4, 6, -5, 2, 1, -3, 5, -6, 4, 2, -5, 3, -2, 4, -6, 3, 0, 0, 0, 0, 0];

T = 1e-3;          % sampling time
period = 1;        % each value lasts 1 second

Ns = period / T;   % samples per segment

u = repelem(vals, Ns).';   % repeat each value Ns times

t = (0:length(u)-1)' * T;

% run simulation

sim = ss(Abig_s, Bbig_s, Cbig_s, Dbig_s);
simol = ss( A_s, B_s, C_s, D_s );
% y = lsim( sim, zeros(size(t)), t, [0 0.1 0 0 0 0 0 0 0 0].' );
y = lsim( simol, u, t, [0 0 0 0 0].' );
% y = lsim( simol, zeros(size(t)), t, [0 0.1 0 0 0].' );

% figure
% figure, plot(t, y(:,1))
% figure, plot(t, y(:,2))
% figure, plot(t, y(:,3))

% plot everything together
movwin = 1e2;
figure

% input
lowlim = 2.2;
hilim = 28.3;

subplot(4,2,1)
plot( movmean(ch1, movwin) ), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
plotx()
subplot(4,2,3)
plot( t, u ), xlim( [lowlim hilim ])
for ii = 3:23
    xline(ii)
end

% rotor angle 
subplot(4,2,2)
plot( movmean(ch2, movwin) ), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
subplot(4,2,4)
plot( t, y(:,1) ), xlim( [lowlim hilim ]), yline(0)

% pendulum angle
subplot(4,2,5)
plot( movmean(ch3, movwin ) ), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
plotx()
xline(5e5)
xline(5.5e5)
subplot(4,2,7)
plot( t, y(:,2)*-1 ), xlim( [lowlim hilim ]), xline(23), xline(22), ylim([-1.2 1.2])
for ii = 3:23
    xline(ii)
end
xline(23.3)
xline(25.8)
yline(0)

% current
subplot(4,2,6)
plot( movmean(ch4, movwin ) ), ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] )
subplot(4,2,8)
plot( t, y(:,3)), xlim( [lowlim hilim ]), yline(0)

% %%%
% figure, hold on
% ax1 = axes;
% plot(ax1, movmean(ch3, movwin )), xlim(ax1, [82000 6e5] )
% % ylim( [0 3.3] ), yline(3.3/2), xlim( [82000 6e5] ), xline(473000), xline(492000)
% 
% ax2 = axes('Position', ax1.Position, ...
%            'Color', 'none', ...
%            'XAxisLocation', 'top', ...
%            'YAxisLocation', 'right');
% 
% plot(ax2, t, y(:,2)*-1 ), xlim(ax2, [lowlim hilim ])
% % xline(23), xline(22)
% 
% % linkaxes([ax1 ax2], 'y')   % or 'x' depending on your setup