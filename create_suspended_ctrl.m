
clear
clc
close all

format shortG % so sci notation isn't used when its stupid
load dynamics.mat

lambda = eig( A_s );
fprintf("Eigenvalues of A suspended OL:\n")
disp(lambda)


% Pole placement for suspended pendulum 
% desired poles
% p = [ -2 -2.5 -3 -4-1j*4 -4+1j*4 ];
% p = [ -3, -3+1j*11, -3-1j*11, -20, -1];
p = [ -9 -8 -7 -8-1j*1 -8+1j*1 ];
% p = [ -2 -4 -5 -20 -60 ];
K_s = place( A_s, B_s, p );


% note the state assignment order: x1, x2, x1d, x2d, i

% switch from linear state assignment above to nonlinear assignment
% ( x1, x1d, x2, x2d, i )
K_s_nl = K_s;
K_s_nl(2) = K_s(3);
K_s_nl(3) = K_s(2);

% Design of state observer
% desired poles
% p = [ -14 -15 -16 -17 -18 ];
p = -55:-51;
% p = [-100 -110 -120 -130 -140];
% p = [ -1800 -1810 -1812 -1813 -1814];

C_s = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 1 0 0 ];
D_s = zeros( 3, 5 );

L_s = place( A_s', C_s', p )';

Abig_s = [ A_s-B_s*K_s          , B_s*K_s     ;
           zeros( size( A_s ) ) , A_s-L_s*C_s ];
Bbig_s = zeros( 10, 1 );
Cbig_s = [ eye(5) zeros(5); zeros(5) zeros(5) ];
Dbig_s = zeros( 10, 1 );

lambda = eig( Abig_s );
fprintf("Eigenvalues of A suspended CL:\n")
disp(lambda)

% create discrete time matrices
T = 1e-6; % [Hz], fastest sampling frequecy, can do 5, 3.33, 2.5, ...

Ae = A_s - L_s * C_s;
Be = [ B_s L_s ];
Ce = eye(5); % extract state
De = zeros(5,4);
observer_sys = ss( Ae, Be, Ce, De );

observer_sys_d = c2d( observer_sys, T, 'zoh' );

[ Ae_d, Be_d, Ce_d, De_d ] = ssdata( observer_sys_d );

F_SIGNED = 1;
rounding_method = 'Floor';
overflow_action = 'Saturate'; 
num_frac = 16;
num_word = 32;

Ae_fi = fi( Ae_d, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Be_fi = fi( Be_d, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Ce_fi = fi( Ce_d, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
De_fi = fi( De_d, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );

% Ae
% Ae_d
% Ae_fi
