% clear 
clc 
close all

ctsim = out.CTsim;
dtsim = out.DTsim;
% note
% (:,1) - (:,5) --> plant states
% (:,6) - (:,10) --> estimator states
% (:,11) --> voltage input

function plotsys( ts, plotgt, plotest )

subplot(3,2,1)
t = ts.time;
d = ts.data;
if plotgt; plot( t, d(:,1) ); end
hold on
if plotest; plot( t, d(:,6) ); end
xlabel("Time [s]")
ylabel("x_1 / rotor angle [rad]")
legend("Ground Truth", "Estimate" )

subplot(3,2,2)
if plotgt; plot( t, d(:,2) ); end
hold on
if plotest; plot( t, d(:,7) ); end
xlabel("Time [s]")
ylabel("x_2 / pendulum angle [rad]")

subplot(3,2,3)
if plotgt; plot( t, d(:,3) ); end
hold on
if plotest; plot( t, d(:,8) ); end
xlabel("Time [s]")
ylabel("Motor current [A]")

subplot(3,2,4)
if plotgt; plot( t, d(:,4) ); end
hold on
if plotest; plot( t, d(:,9) ); end
xlabel("Time [s]")
ylabel("Rotor speed [rad/s]")

subplot(3,2,5)
if plotgt; plot( t, d(:,5) ); end
hold on
if plotest; plot( t, d(:,10) ); end
xlabel("Time [s]")
ylabel("Pendulum speed [rad/s]")

subplot(3,2,6)
plot( t, d(:,11) )
xlabel("Time [s]")
ylabel("Motor voltage input [V]")

end

figure
plotsys( ctsim, 1, 1) 
