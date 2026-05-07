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

N = floor(length(raw)/4);    % samples per channel
ch1 = raw(1:N);
ch2 = raw(N+1:2*N);
ch3 = raw(2*N+1:3*N);
ch4 = raw(3*N+1:4*N);

% find the approx number of samples per second (isn't quite 20 kSa/s)
diffs = find( abs(diff(ch1)) > 0.25);
diff(diffs); % seems like it is overwhelmingly 19794
figure, plot( abs(diff(ch1)) > 0.25), hold on, plot(ch1)

% samples seem to start at 37028
% input seems to end at 492271 (diffs(end))


function [] = xlines(~)

for ii = 1:27; xline(ii); end

end

% vals = [0, 0, 0, -3, 5, -4, 3, -4, 6, -5, 2, 1, -3, 5, -6, 4, 2, -5, 3, -2, 4, -6, 3, 0, 0, 0, 0, 0]; % 25 values after the first three zeros
vals = [-3, 5, -4, 3, -4, 6, -5, 2, 1, -3, 5, -6, 4, 2, -5, 3, -2, 4, -6, 3, 0, 0, 0, 0, 0, 0, 0, 0]; % 28 values after the first three zeros (added 3 zeros)
% create time for the real data
sps = 19794; % samples per second
Ts = 1 / sps;
Tend = 28;
t = 0:Ts:28;

startidx = 37028 + 3*sps;
endch1 = length(t) + startidx - 1;
ch1 = ch1(startidx:endch1);
figure, plot(t, ch1)

ch2 = ch2(startidx:endch1);
ch3 = ch3(startidx:endch1);
ch4 = ch4(startidx:endch1);

%%
% T = 1e-3;          % sampling time
period = 1;        % each value lasts 1 second

Ns = period / Ts;   % samples per segment

u = repelem(vals, Ns);   % repeat each value Ns times
% for some reason u is running one sample short
u = [ u 0 ];

sim = ss(Abig_s, Bbig_s, Cbig_s, Dbig_s);
simol = ss( A_s, B_s, C_s, D_s );
y = lsim( simol, u, t, [0 0 0 0 0].' );

% plot everything together
movwin = 1e2;
figure

% new plots
subplot(4,1,1)
yyaxis left, plot(t, u)
yyaxis right, plot(t, movmean(ch1, movwin))
title("Voltage Input")
xlines

subplot(4,1,2)
yyaxis left, plot(t, y(:,1))
yyaxis right, plot(t, movmean(ch2, movwin))
title("Rotor Arm Angle")
xlines

subplot(4,1,3)
yyaxis left, plot(t, y(:,2))
yyaxis right, plot(t, -movmean(ch3, movwin))
title("Pendulum Arm Angle")
xlines

subplot(4,1,4)
yyaxis left, plot(t, y(:,3))
yyaxis right, plot(t, movmean(ch4, movwin))
title("Motor Current")
xlines


% Evan notes on model fitting
%{
* do not necissarily have to match magnitudes. we can just add a tunable
gain parameter in front of each of the outputs. Hopefully the fitting
algorithm is more concerned about fitting transient behavior
* TODO ask chat to set up greyid lol
%}