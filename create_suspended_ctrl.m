
clear
clc
close all

format shortG % so sci notation isn't used when its stupid
load dynamics.mat

lambda = eig( A_s );
fprintf("Eigenvalues of A suspended OL:\n")
disp(lambda)


% Pole placement for suspended pendulum 
% pc = [ -2 -2.5 -3 -4-1j*0.5 -4+1j*0.5 ];
% pc = [ -3, -3+1j*11, -3-1j*11, -20, -1];
% pc = [ -9 -8 -7 -8-1j*0.1 -8+1j*0.1 ];
% pc = [-20+1j*0.1, -20-1j*0.1, -21, -22+1j*0.1, -22-1j*0.1];
% pc = -24:-20;
% pc = -9:-5;
% pc = [ -100, -100+j, -10, -5, -100-j ];
% pc = [ -2 -4 -5 -6 -7 ];
pc = [-50 -25 -20 -4+j*0.5 -4-j*0.5];
% pc = -1
% pc = -35:-31;


% UNCOMMENT THE BELOW FOR CONVENTIONAL POLE PLACEMENT
K_s = place( A_s, B_s, pc );

% LQR (keep in mind that this is CT and observer based, so not robust...)
Q = [ 10 0 0 0 0 ;
      0 10 0 0 0 ;
      0 0 0 0 0 ;
      0 0 0 0 0 ;
      0 0 0 0 0 ];
R = 1;
C_s = [1 0 0 0 0 ;
       0 1 0 0 0 ;
       0 0 1 0 0 ];
D_s = zeros(3,1);
[K, S, P] = lqr( ss(A_s, B_s, C_s, D_s), Q, R );

% UNCOMMENT THE BELOW FOR LQR
% K_s = K;

% Design of state observer
% poo = [ -14 -15 -16 -17 -18 ];
% po = -55:-51;
% po = -555:-551;
% po = -604:-600;
% po = -804:-800;
% po = -504:-500;
% po = -104:-100;
po = -404:-400;
% po = [-204:-201, -500];
% po = -1504:-1500;
% po = [ -1800 -1810 -1812 -1813 -1814];

L_s = place( A_s', C_s', po )';

% convert to fixed point
F_SIGNED = 1;
rounding_method = 'Floor';
overflow_action = 'Saturate'; 
num_frac = 24;
num_word = 32;
res = 2^( -num_frac );

% create fixed point matrices
Aee = A_s - L_s * C_s;
Bee = [ B_s L_s ];
Cee = -K_s; % extract state
Dee = zeros(1,4);
observer_sys = ss( Aee, Bee, Cee, Dee );

% convert to discrete time 
T = 1e-3; 
observer_sys_d = c2d( observer_sys, T, 'zoh' );
[ Ae_de, Be_de, Ce_de, De_de ] = ssdata( observer_sys_d );

Ae_fi = fi( Ae_de, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Be_fi = fi( Be_de, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Ce_fi = fi( Ce_de, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
De_fi = fi( De_de, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
disp("Sample freq (Hz")
disp( 1/T )
disp("Controller poles")
disp(pc)
disp("Observer poles")
disp(po)
disp("A")
for i = 1:size(Ae_fi.hex,1)
    tokens = strsplit(strtrim(Ae_fi.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("B (col1: B_s; col2-4: L_s)")
for i = 1:size(Be_fi.hex,1)
    tokens = strsplit(strtrim(Be_fi.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("C (-K_s)")
for i = 1:size(Ce_fi.hex,1)
    tokens = strsplit(strtrim(Ce_fi.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("D")
for i = 1:size(De_fi.hex,1)
    tokens = strsplit(strtrim(De_fi.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end

if max(max( abs(Ae_fi.double - Ae_de) )) > res
    disp("Warning: FI encoded A matrix is saturating")
    disp("encoded")
    disp(Ae_fi.double)
    disp("original")
    disp(Ae_de)
end
if max(max( abs(Be_fi.double - Be_de) )) > res
    disp("Warning: FI encoded B matrix is saturating")
    disp("encoded")
    disp(Be_fi.double)
    disp("original")
    disp(Be_de)
end
if max(max( abs(Ce_fi.double - Ce_de) )) > res
    disp("Warning: FI encoded C matrix is saturating")
    disp("encoded")
    disp(Ce_fi.double)
    disp("original")
    disp(Ce_de)
end
if max(max( abs(De_fi.double - De_de) )) > res
    disp("Warning: FI encoded D matrix is saturating")
    disp("encoded")
    disp(De_fi.double)
    disp("original")
    disp(De_de)
end

    


%%% The following are NOT implemented on the pendulum and used only for sim %%%
% create overall controller/observer matrix 
Abig_s = [ A_s-B_s*K_s          , B_s*K_s     ;
           zeros( size( A_s ) ) , A_s-L_s*C_s ];
Bbig_s = zeros( 10, 1 );
Cbig_s = [ eye(5) zeros(5); zeros(5) zeros(5) ];
Dbig_s = zeros( 10, 1 );

% lambda = eig( Abig_s );
% fprintf("Eigenvalues of A suspended CL:\n")
% disp(lambda)

% create discrete time matrices for SLK state space block
% T = 1e-3; % [Hz], fastest sampling frequecy, can do 5, 3.33, 2.5, ...

Ae = A_s - L_s * C_s;
Be = [ B_s L_s ];
Ce = eye(5); % extract state
De = zeros(5,4);
observer_sys = ss( Ae, Be, Ce, De );

observer_sys_d = c2d( observer_sys, T, 'zoh' );

[ Ae_d, Be_d, Ce_d, De_d ] = ssdata( observer_sys_d );