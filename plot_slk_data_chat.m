%% Load data
clc; close all;

S_OL  = load("S_CT_OL.mat");

S_CT1 = load("S_CT_1en3.mat");
S_CT2 = load("S_CT_5en2.mat");
S_CT3 = load("S_CT_1en2.mat");

S_DT1 = load("S_DT_1en3.mat");
S_DT2 = load("S_DT_5en2.mat");
S_DT3 = load("S_DT_1en2.mat");
S_DT4 = load("S_DT_5en1.mat");
S_DT5 = load("S_DT_2en1.mat");

%% =========================
% FIGURE 1: CT + DT + OL
%% =========================
figure
compare_ALL_6(S_OL, S_CT1, S_DT1, S_CT2, S_DT2, S_CT3, S_DT3)
set(gcf, 'Units', 'inches', 'Position', [1, 1, 7, 8])

%% =========================
% FIGURE 2: DT ONLY + OL
%% =========================
figure
compare_DT_only(S_OL, S_DT3, S_DT4, S_DT5)
set(gcf, 'Units', 'inches', 'Position', [1, 1, 7, 8])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FUNCTION 1: CT + DT + OL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function compare_ALL_6(S_OL, S_CT1, S_DT1, S_CT2, S_DT2, S_CT3, S_DT3)

cases_CT = {S_CT1, S_CT2, S_CT3};
cases_DT = {S_DT1, S_DT2, S_DT3};

ct_colors = [0.7 0 0;
             0.9 0.2 0.2;
             1.0 0.4 0.4];

dt_colors = [0 0.2 0.8;
             0.2 0.5 1.0;
             0.5 0.7 1.0];

ct_alpha = 0.6;
dt_alpha = 0.6;

t_ol = S_OL.out.CTsim.time;
d_ol = S_OL.out.CTsim.data;

for k = 1:6
    
    subplot(3,2,k)
    hold on
    
    if k == 1
        i = k;
        ylab = 'x_1 [rad]';
    elseif k == 2
        i = k;
        ylab = 'x_2 [rad]';
    elseif k == 3
        i = k;
        ylab = 'x_3 [A]';
    elseif k == 4
        i = k;
        ylab = 'x_4 [rad/s]';
    elseif k == 5
        i = k;
        ylab = 'x_5 [rad/s]';
    else
        i = 11;
        ylab = 'u [V]';
    end
    
    plot(t_ol, d_ol(:,i), 'k', 'LineWidth', 2.5)
    
    for c = 1:3
        
        t_ct = cases_CT{c}.out.DTsim.time;
        d_ct = cases_CT{c}.out.DTsim.data;
        
        t_dt = cases_DT{c}.out.DTsim.time;
        d_dt = cases_DT{c}.out.DTsim.data;
        
        h_ct = plot(t_ct, d_ct(:,i), ...
            'Color', [ct_colors(c,:) ct_alpha], ...
            'LineWidth', 2.0);
        h_ct.Color(4) = ct_alpha;
        
        h_dt = stairs(t_dt, d_dt(:,i), ...
            'Color', [dt_colors(c,:) dt_alpha], ...
            'LineWidth', 2.0);
        h_dt.Color(4) = dt_alpha;
    end
    
    grid on
    
    ylabel(ylab, 'FontWeight', 'bold', 'FontSize', 14)
    xlabel('Time [s]', 'FontWeight', 'bold', 'FontSize', 14)
    % title(sprintf('Signal %d', k), 'FontWeight', 'bold', 'FontSize', 15)
    
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 13;
    ax.LineWidth = 1.3;
    
    if k == 6
        
        h_ol  = plot(nan, nan, 'k', 'LineWidth', 2.5);
        
        h_ct1 = plot(nan, nan, 'Color', ct_colors(1,:), 'LineWidth', 2.0);
        h_dt1 = plot(nan, nan, 'Color', dt_colors(1,:), 'LineWidth', 2.0);
        
        h_ct2 = plot(nan, nan, 'Color', ct_colors(2,:), 'LineWidth', 2.0);
        h_dt2 = plot(nan, nan, 'Color', dt_colors(2,:), 'LineWidth', 2.0);
        
        h_ct3 = plot(nan, nan, 'Color', ct_colors(3,:), 'LineWidth', 2.0);
        h_dt3 = plot(nan, nan, 'Color', dt_colors(3,:), 'LineWidth', 2.0);
        
        legend([h_ol, ...
                h_ct1, h_dt1, ...
                h_ct2, h_dt2, ...
                h_ct3, h_dt3], ...
            ["OL", ...
             "CT 1e-3","DT 1e-3", ...
             "CT 5e-2","DT 5e-2", ...
             "CT 1e-2","DT 1e-2"], ...
            'FontWeight','bold', ...
            'FontSize', 12)
    end
end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FUNCTION 2: DT ONLY + OL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function compare_DT_only(S_OL, S_DT1, S_DT2, S_DT3)

dt_cases = {S_DT1, S_DT2, S_DT3};

dt_colors = [0 0.2 0.8;
             0 0.6 0.3;
             0.9 0.4 0.1];

t_ol = S_OL.out.CTsim.time;
d_ol = S_OL.out.CTsim.data;

for k = 1:6
    
    subplot(3,2,k)
    hold on
        
    if k == 1
        i = k;
        ylab = 'x_1 [rad]';
    elseif k == 2
        i = k;
        ylab = 'x_2 [rad]';
    elseif k == 3
        i = k;
        ylab = 'x_3 [A]';
    elseif k == 4
        i = k;
        ylab = 'x_4 [rad/s]';
    elseif k == 5
        i = k;
        ylab = 'x_5 [rad/s]';
    else
        i = 11;
        ylab = 'u [V]';
    end
    
    plot(t_ol, d_ol(:,i), 'k', 'LineWidth', 2.5)
    
    for c = 1:3
        
        t_dt = dt_cases{c}.out.DTsim.time;
        d_dt = dt_cases{c}.out.DTsim.data;
        
        stairs(t_dt, d_dt(:,i), ...
            'Color', dt_colors(c,:), ...
            'LineWidth', 2.0)
    end
    
    grid on
    
    ylabel(ylab, 'FontWeight', 'bold', 'FontSize', 14)
    xlabel('Time [s]', 'FontWeight', 'bold', 'FontSize', 14)
    % title(sprintf('Signal %d', k), 'FontWeight', 'bold', 'FontSize', 15)
    
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 13;
    ax.LineWidth = 1.3;
    
    if k == 1
        
        h_ol = plot(nan, nan, 'k', 'LineWidth', 2.5);
        h1 = plot(nan, nan, 'Color', dt_colors(1,:), 'LineWidth', 2.0);
        h2 = plot(nan, nan, 'Color', dt_colors(2,:), 'LineWidth', 2.0);
        h3 = plot(nan, nan, 'Color', dt_colors(3,:), 'LineWidth', 2.0);
        
        legend([h_ol, h1, h2, h3], ...
            ["OL", "DT 1e-3", "DT 5e-1", "DT 2e-1"], ...
            'FontWeight','bold', ...
            'FontSize', 12)
    end
end

end