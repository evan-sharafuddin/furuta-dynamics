clear
clc 
close all

% load dynamics
% init_furuta_grey

% read input
fid = fopen('scope_14.bin', 'rb', 'ieee-le');
raw = fread(fid, 'float32');
fclose(fid);

raw = raw( 29:end );
raw = raw( 1:2800000 );

N = floor(length(raw)/4);    % samples per channel
ch1 = raw(1:N);
ch2 = raw(N+1:2*N);
ch3 = raw(2*N+1:3*N);
ch4 = raw(3*N+1:4*N);

% find the approx number of samples per second (isn't quite 20 kSa/s)
diffs = find( abs(diff(ch1)) > 0.25);
diff(diffs); % seems like it is overwhelmingly 19794

% samples seem to start at 37028
% input seems to end at 492271 (diffs(end))

vals = [-3, 5, -4, 3, -4, 6, -5, 2, 1, -3, 5, -6, 4, 2, -5, 3, -2, 4, -6, 3, 0, 0, 0, 0, 0, 0, 0, 0]; % 28 values after the first three zeros (added 3 zeros)
% create time for the real data
sps = 19794; % samples per second
Ts = 1 / sps;
Tend = 28;
t = 0:Ts:28;

startidx = 37028 + 3*sps;
endch1 = length(t) + startidx - 1;
ch1 = ch1(startidx:endch1);

ch2 = ch2(startidx:endch1);
ch3 = ch3(startidx:endch1);
ch4 = ch4(startidx:endch1);

% center about zero
mid = 3.3/2;
ch2 = ch2 - mid;
ch3 = ch3 - mid;
ch4 = ch4 - mid;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CREATE INPUT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

period = 1;                 % seconds per voltage level
Ns = round(period / Ts);    % samples per segment

u = repelem(vals, Ns);
u = [u 0];
u = u(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEASURED OUTPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

movwin = 100;

y_meas = [ ...
    movmean(ch2, movwin), ...
   -movmean(ch3, movwin), ...
    movmean(ch4, movwin) ...
];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DOWNSAMPLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Fs_target = 250;                 % target sample rate [Hz]
decim = round(sps / Fs_target);

u_ds = downsample(u, decim);

y_ds = downsample(y_meas, decim);

Ts_ds = Ts * decim;

t_ds = (0:length(u_ds)-1)' * Ts_ds;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NORMALIZE OUTPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % prevents optimizer from overweighting current channel
% for ii = 1:size(y_ds,2)
%     y_ds(:,ii) = y_ds(:,ii) ./ max(abs(y_ds(:,ii)));
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUILD IDDATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data = iddata( ...
    y_ds, ...
    u_ds, ...
    Ts_ds ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUILD NONLINEAR GREY-BOX MODEL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Order = [3 1 5];
% [ outputs inputs states ] [Ny Nu Nx]

Parameters = { ...
    35.27e-3; ...      % m1
    44.73e-3; ...      % m2
    192.16e-3; ...     % L1
    243.66e-3; ...     % L2
    0.1; ...           % Jaddl
    0.5; ...           % b1
    4.59e-4; ...       % b2
    0.0918*(60/2/pi); ... % Ke
    1.5*0.0918*(60/2/pi); ...  % Km
    1; ...
    1; ...
    2; ...
};

InitialStates = zeros(5,1);

nlgr = idnlgrey( ...
    'furuta_grey', ...
    Order, ...
    Parameters, ...
    InitialStates, ...
    0 ...              % continuous-time
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETER NAMES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nlgr.Parameters(1).Name = 'm1';
nlgr.Parameters(2).Name = 'm2';
nlgr.Parameters(3).Name = 'L1';
nlgr.Parameters(4).Name = 'L2';
nlgr.Parameters(5).Name = 'Jaddl';
nlgr.Parameters(6).Name = 'b1';
nlgr.Parameters(7).Name = 'b2';
nlgr.Parameters(8).Name = 'Ke';
nlgr.Parameters(9).Name = 'Km';
nlgr.Parameters(10).Name = 'k1';
nlgr.Parameters(11).Name = 'k2';
nlgr.Parameters(12).Name = 'k3';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHOOSE PARAMETERS TO ESTIMATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % START SMALL
% for ii = 1:12
%     nlgr.Parameters(ii).Fixed = true;
% end
% 
% % free only a few initially
% nlgr.Parameters(6).Fixed = false;   % b1
% nlgr.Parameters(7).Fixed = false;   % b2
% nlgr.Parameters(8).Fixed = false;   % Ke
% nlgr.Parameters(9).Fixed = false;   % Km
% nlgr.Parameters(10).Fixed = false;   % K1
% nlgr.Parameters(11).Fixed = false;   % K2
% nlgr.Parameters(12).Fixed = false;   % K3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETER CONSTRAINTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for ii = 1:12
    nlgr.Parameters(ii).Minimum = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SOLVER OPTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

opt = nlgreyestOptions;

opt.Display = 'on';

opt.SearchOptions.MaxIterations = 100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPORTANT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% electrical dynamics are likely stiff
nlgr.Algorithm.SimulationOptions.Solver = 'ode15s';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ESTIMATE MODEL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sys_est = nlgreyest( ...
    data, ...
    nlgr, ...
    opt ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPARE RESULTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
compare(data, sys_est)