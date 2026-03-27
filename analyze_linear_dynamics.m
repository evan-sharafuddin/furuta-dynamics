
clear
clc
close all

format shortG % so sci notation isn't used when its stupid
load dynamics.mat

% Hautus test
lambda = eig( A_s );
fprintf("Eigenvalues of A:\n")
disp(lambda)

fprintf("Performing Hautus Test:\n")
for ii = 1:numel(lambda)
    l = lambda( ii );
    % l*eye(size(A_s))-A_s
    mat = [ (l*eye(size(A_s)) - A_s) , B_s ];
    rk = rank( mat );
    fprintf("Matrix for lambda=%.2f+j%.2f:\n", real(l), imag(l))
    disp( mat )
    fprintf("Rank of the above matrix: %d\n----\n", rk )
    % svd( mat )
end

% since lambda_max of A_s is so big, it dominates the other terms, so
% matlab regards the other terms as zero incorrectly
% Co = ctrb( A_s, B_s );
% svd(Co)

%{
According to the Hautus Test, all modes of the system are controllable.
Therefore, we can use pole placement to design a feedback controller in the
suspended configuration
%}

%% Design of feedback controller
% desired poles
p = [ -2 -2.5 -3 -4-1j*4 -4+1j*4 ];
K_s = place( A_s, B_s, p );

% note the state assignment order: x1, x2, x1d, x2d, i

% switch from linear state assignment above to nonlinear assignment
% ( x1, x1d, x2, x2d, i )
K_s_nl = K_s;
K_s_nl(2) = K_s(3);
K_s_nl(3) = K_s(2);

%% Assess observability of system
% in reality, we will only have the encoder angles, and the current
% measurement from the circuit

C_s = [ 1 0 0 0 0 ; 
        0 1 0 0 0 ;
        0 0 0 0 1 ];
D_s = zeros( 3, 5 );

lambda = eig( A_s );
fprintf("Eigenvalues of A:\n")
disp(lambda)

fprintf("Performing Hautus Test:\n")
for ii = 1:numel(lambda)
    l = lambda( ii );
    % l*eye(size(A_s))-A_s
    mat = [ (l*eye(size(A_s)) - A_s) ; C_s ];
    rk = rank( mat );
    fprintf("Matrix for lambda=%.2f+j%.2f:\n", real(l), imag(l))
    disp( mat )
    fprintf("Rank of the above matrix: %d\n----\n", rk )
    % svd( mat )
end

%{
According to the Hautus Test, all modes of the system are observable.
Therefore, we can use pole placement to design a feedback controller in the
suspended configuration
%}

%% Design of state observer
% desired poles
p = [ -14 -15 -16 -17 -18 ];
% p = [ -100 -200 -300 -40 -500 ];
L_s = place( A_s', C_s', p )';

% note the state assignment order: x1, x2, x1d, x2d, i

% switch from linear state assignment above to nonlinear assignment
% ( x1, x1d, x2, x2d, i )
% L_s_nl    = L_s;
% L_s_nl(2) = L_s(3);
% L_s_nl(3) = L_s(2);

%% Pole placement for inverted pendulum
% note that the system matrices for the suspended and inverted pendulum
% linearizations have the same controllability and observability properties
load dynamics.mat

lambda = eig( A_i );
fprintf("Eigenvalues of A:\n")
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
Abig = [ A_i-B_i*K_i          , B_i*K_i     ;
         zeros( size( A_i ) ) , A_i-L_i*C_i ];