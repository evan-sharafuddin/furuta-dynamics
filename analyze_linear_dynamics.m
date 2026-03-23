
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
D_s = zeros( 3, 1 );

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
