clear 
clc 
close all

load ctdata.mat
load dtsim.mat

figure

% Subplot 1 — Rotor angle
subplot(3,2,1)
hold on
plot(ct0p0.time,   ct0p0.data(:,1))
plot(ct0p05.time,  ct0p05.data(:,1))
plot(ct0p099.time, ct0p099.data(:,1))
title('Rotor angle [rad]')

% Subplot 2 — Pendulum angle
subplot(3,2,2)
hold on
plot(ct0p0.time,   ct0p0.data(:,2))
plot(ct0p05.time,  ct0p05.data(:,2))
plot(ct0p099.time, ct0p099.data(:,2))
title('Pendulum angle [rad]')

% Subplot 3 — Current
subplot(3,2,3)
hold on
plot(ct0p0.time,   ct0p0.data(:,3))
plot(ct0p05.time,  ct0p05.data(:,3))
plot(ct0p099.time, ct0p099.data(:,3))
title('Current [A]')

% Subplot 4 — Rotor speed
subplot(3,2,4)
hold on
plot(ct0p0.time,   ct0p0.data(:,4))
plot(ct0p05.time,  ct0p05.data(:,4))
plot(ct0p099.time, ct0p099.data(:,4))
title('Rotor speed [rad/s]')

% Subplot 5 — Pendulum speed
subplot(3,2,5)
hold on
plot(ct0p0.time,   ct0p0.data(:,5))
plot(ct0p05.time,  ct0p05.data(:,5))
plot(ct0p099.time, ct0p099.data(:,5))
title('Pendulum speed [rad/s]')

figure

% Subplot 1 — Rotor angle
subplot(3,2,1)
hold on
plot(dtsim.time, dtsim.data(:,1))
plot(dtsim.time, dtsim.data(:,6))
title('Rotor angle [rad]')

% Subplot 2 — Pendulum angle
subplot(3,2,2)
hold on
plot(dtsim.time, dtsim.data(:,2))
plot(dtsim.time, dtsim.data(:,7))
title('Pendulum angle [rad]')

% Subplot 3 — Current
subplot(3,2,3)
hold on
plot(dtsim.time, dtsim.data(:,3))
plot(dtsim.time, dtsim.data(:,8))
title('Current [A]')

% Subplot 4 — Rotor speed
subplot(3,2,4)
hold on
plot(dtsim.time, dtsim.data(:,4))
plot(dtsim.time, dtsim.data(:,9))
title('Rotor speed [rad/s]')

% Subplot 5 — Pendulum speed
subplot(3,2,5)
hold on
plot(dtsim.time, dtsim.data(:,5))
plot(dtsim.time, dtsim.data(:,10))
title('Pendulum speed [rad/s]')