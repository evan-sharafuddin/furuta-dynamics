
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
    fprintf("Matrix for lambda=%.2f:\n", l)
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

%% Assess observability of system 
C_s = [ 0 ]