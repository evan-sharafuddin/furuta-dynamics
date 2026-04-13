
clear
clc
close all

format shortG % so sci notation isn't used when its stupid
load dynamics.mat

lambda = eig( A_s );
fprintf("Eigenvalues of A suspended OL:\n")
disp(lambda)

% Convert system to discrete time
C_s = [ 1 0 0 0 0 ;
        0 1 0 0 0 ;
        0 0 1 0 0 ];
D_s = [ 0 ;
        0 ;
        0 ];
Ts = 1e-3;

sys_ct = ss( A_s, B_s, C_s, D_s );
sys_dt = c2d( sys_ct, Ts, 'zoh' );

% convert desired poles to discrete time
p_ct = -24:-20;
p_dt = exp( p_ct * Ts );

K_s_d = place( p )

