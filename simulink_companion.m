
clear
% clc
close all

format shortG % so sci notation isn't used when its stupid
load dynamics.mat

lambda = eig( A_s );
fprintf("Eigenvalues of A suspended OL:\n")
disp(lambda)


% Pole placement for suspended pendulum 
% desired poles
p = [ -2 -2.5 -3 -4-1j*4 -4+1j*4 ];
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
p = [ -14 -15 -16 -17 -18 ];
% p = [ -10 -20 -50 -40 -60 ];
% p = [ -100 -200 -300 -40 -500 ];

C_s = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 0 0 1 ];
D_s = zeros( 3, 5 );

L_s = place( A_s', C_s', p )';

Abig_s = [ A_s-B_s*K_s          , B_s*K_s     ;
           zeros( size( A_s ) ) , A_s-L_s*C_s ];

lambda = eig( Abig_s );
fprintf("Eigenvalues of A suspended CL:\n")
disp(lambda)


% note the state assignment order: x1, x2, x1d, x2d, i

% Pole placement for inverted pendulum
% note that the system matrices for the suspended and inverted pendulum
% linearizations have the same controllability and observability properties

lambda = eig( A_i );
fprintf("Eigenvalues of A inverted OL:\n")
disp(lambda)

p = [ -2 -2.5 -3 -4-1j*4 -4+1j*4 ];
K_i = place( A_i, B_i, p );

K_i_nl    = K_i;
K_i_nl(2) = K_i(3);
K_i_nl(3) = K_i(2);

% create observer
C_i = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 0 0 1 ];
D_i = zeros( 3, 5 );

p = [ -14 -15 -16 -17 -18 ];
% p = [ -100 -200 -300 -40 -500 ];
L_i = place( A_i', C_i', p )';

% create overall state space representation
Abig_i = [ A_i-B_i*K_i          , B_i*K_i     ;
         zeros( size( A_i ) ) , A_i-L_i*C_i ];

lambda = eig( Abig_i );
fprintf("Eigenvalues of A inverted CL:\n")
disp(lambda)
