% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%
% % NOTE -- this is not updated, and there are issues with state ordering,
% % observer gains, etc 
% %%%%%%%%
% 
% % note the state assignment order: x1, x2, x1d, x2d, i
% 
% % Pole placement for inverted pendulum
% % note that the system matrices for the suspended and inverted pendulum
% % linearizations have the same controllability and observability properties
% 
% lambda = eig( A_i );
% fprintf("Eigenvalues of A inverted OL:\n")
% disp(lambda)
% 
% p = [ -9 -8 -7 -8-1j*1 -8+1j*1 ];
% K_i = place( A_i, B_i, p );
% 
% K_i_nl    = K_i;
% K_i_nl(2) = K_i(3);
% K_i_nl(3) = K_i(2);
% 
% % create observer
% C_i = [ 1 0 0 0 0 ; 
%         0 1 0 0 0 ;
%         0 0 0 0 1 ];
% D_i = zeros( 3, 5 );
% 
% p = [ -14 -15 -16 -17 -18 ];
% % p = [ -100 -200 -300 -40 -500 ];
% L_i = place( A_i', C_i', p )';
% 
% % create overall state space representation [ 2n x 2n ]
% Abig_i = [ A_i-B_i*K_i          , B_i*K_i     ;
%          zeros( size( A_i ) ) , A_i-L_i*C_i ];
% Bbig_i = zeros( 10, 1 );
% Cbig_i = [ eye(5) zeros(5); zeros(5) zeros(5) ];
% Dbig_i = zeros( 10, 1 );
% 
% lambda = eig( Abig_i );
% fprintf("Eigenvalues of A inverted CL:\n")
% disp(lambda)
% 
% % %% 
% % sus = ss( Abig_s, Bbig_s, Cbig_s, Dbig_s );
% % t = 0:0.01:10;
% % 
% % y = lsim( sus, zeros(length(t), 1), t, [0 0.1 0 0 0 , 0 0 0 0 0] );
