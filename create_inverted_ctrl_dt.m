clc
clear 

%% BE CAREFUL! NAMING HASN'T BEEN COMPLETED UNCONFLICTED! ALSO MAKE SURE SAMPLE PERIOD IS SAME FOR ALL SCRIPTS!
format shortG % so sci notation isn't used when its stupid
load dynamics.mat

% Convert system to discrete time
C_i = [ 1 0 0 0 0 ;
        0 1 0 0 0 ;
        0 0 1 0 0 ];
D_i = [ 0 ;
        0 ;
        0 ];
T = 1/1000;

sys_ct = ss( A_i, B_i, C_i, D_i );
sys_dt = c2d( sys_ct, T, 'zoh' );

Aold = A_i;
Bold = B_i;
Cold = C_i;
Dold = D_i;

[A_i, B_i, C_i, D_i] = ssdata(sys_dt);

% Pole placement for suspended pendulum 
pc = -9:-5;
pc = exp(pc * T);

% UNCOMMENT THE BELOW FOR CONVENTIONAL POLE PLACEMENT
K_i = place( A_i, B_i, pc );

K_i = zeros(size(K_i));

% Design of state observer
% po = -304:-300;
% po = -104:-100;
po = -204:-200;
% po = -404:-400;
po = exp(po * T);

L_i = place( A_i', C_i', po )';

% convert to fixed point
F_SIGNED = 1;
rounding_method = 'Floor';
overflow_action = 'Saturate'; 
num_frac = 24;
num_word = 32;
res = 2^( -num_frac );

% create fixed point matrices
Aee_i = A_i - L_i * C_i;
Bee_i = [ B_i L_i ];
Cee_i = -K_i; % extract state
Dee_i = zeros(1,4);
% observer_sys = ss( Aee, Bee, Cee, Dee );

% do not need to convert to discrete time, so can just set the de matrices
Ae_de_i = Aee_i;
Be_de_i = Bee_i;
Ce_de_i = Cee_i;
De_de_i = Dee_i;

% calculate compensator poles
A_D = A_i - B_i*K_i - L_i*C_i;
B_D = L_i;
C_D = -K_i;
D_D = zeros(1,3);
Dz = ss(A_D, B_D, C_D, D_D);

disp("Compensator poles")
disp(pole(Dz))

% calculate plant poles

% Gz = ss( A_s, B_s, C_s, D_s, T );
% cl = feedback(Gz, -Dz);


Ae_fi_i = fi( Ae_de_i, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Be_fi_i = fi( Be_de_i, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
Ce_fi_i = fi( Ce_de_i, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
De_fi_i = fi( De_de_i, F_SIGNED, num_word, num_frac, ...
    'RoundingMethod', rounding_method, 'OverflowAction', overflow_action );
disp("Sample freq (Hz")
disp( 1/T )
disp("Controller poles (DT)")
disp(pc)
disp("Observer poles (DT)")
disp(po)
if (max(abs(po))/2/pi > 1/2*1/T )
    disp("Warning: observer is exceeding Nyquist frequency")
end

disp("A")
for i = 1:size(Ae_fi_i.hex,1)
    tokens = strsplit(strtrim(Ae_fi_i.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("B (col1: B_s; col2-4: L_s)")
for i = 1:size(Be_fi_i.hex,1)
    tokens = strsplit(strtrim(Be_fi_i.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("C (-K_s)")
for i = 1:size(Ce_fi_i.hex,1)
    tokens = strsplit(strtrim(Ce_fi_i.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end
disp("D")
for i = 1:size(De_fi_i.hex,1)
    tokens = strsplit(strtrim(De_fi_i.hex(i,:)));          % split by whitespace
    tokens = strcat('0x', tokens);               % prepend 0x
    line = strjoin(tokens, ', ');  
    line = [line ','];                    % add trailing comma% join with commas
    disp(line)
end

if max(max( abs(Ae_fi_i.double - Ae_de_i) )) > res
    disp("Warning: FI encoded A matrix is saturating")
    disp("encoded")
    disp(Ae_fi_i.double)
    disp("original")
    disp(Ae_de_i)
end
if max(max( abs(Be_fi_i.double - Be_de_i) )) > res
    disp("Warning: FI encoded B matrix is saturating")
    disp("encoded")
    disp(Be_fi_i.double)
    disp("original")
    disp(Be_de_i)
end
if max(max( abs(Ce_fi_i.double - Ce_de_i) )) > res
    disp("Warning: FI encoded C matrix is saturating")
    disp("encoded")
    disp(Ce_fi_i.double)
    disp("original")
    disp(Ce_de_i)
end
if max(max( abs(De_fi_i.double - De_de_i) )) > res
    disp("Warning: FI encoded D matrix is saturating")
    disp("encoded")
    disp(De_fi_i.double)
    disp("original")
    disp(De_de_i)
end

    


% %%% The following are NOT implemented on the pendulum and used only for sim %%%
% % create overall controller/observer matrix 
Abig_i = [ A_i-B_i*K_i          , B_i*K_i     ;
           zeros( size( A_i ) ) , A_i-L_i*C_i ];
Bbig_i = zeros( 10, 1 );
Cbig_i = [ eye(5) zeros(5); zeros(5) zeros(5) ];
Dbig_i = zeros( 10, 1 );

lambda = eig( Abig_i );
fprintf("Eigenvalues of A suspended CL:\n")
disp(lambda)

% create discrete time matrices for SLK state space block
% T = 1e-3; % [Hz], fastest sampling frequecy, can do 5, 3.33, 2.5, ...

Ae_i = A_i - L_i * C_i;
Be_i = [ B_i L_i ];
Ce_i = eye(5); % extract state
De_i = zeros(5,4);
observer_sys = ss( Ae_i, Be_i, Ce_i, De_i );

% observer_sys_d = c2d( observer_sys, T, 'zoh' );

% [ Ae_d, Be_d, Ce_d, De_d ] = ssdata( observer_sys_d );


