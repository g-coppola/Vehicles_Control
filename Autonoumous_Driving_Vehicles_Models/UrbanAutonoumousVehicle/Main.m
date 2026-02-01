clear; clc; close all;

load('SimulationData.mat')

Ts = scenario.SampleTime;
Tend = scenario.StopTime;

%% STATE SPACE MATRICES AND CONTROL (H2 CONTROL)
V_interval = {[0 0.8], [0.8 1.6], [1.6 2.4], [2.4 3.2], [3.2 4.1], [4.1 5]};

l = size(V_interval,2);

L = 2.8;
tau = 0.5;

trim = 'VehicleModel';
Kvec = {};

for i = 1:l
    Vx = V_interval{i}(1);
    [A,B,~,~] = linmod(trim,[0;0],0);
    [Aa,Ba,Cza,Dzua,Bwa,~] = AugmentedState(A,B);
    Ka = H_2(Aa, Ba, Bwa, Cza, Dzua);
    
    Vx = V_interval{i}(2);
    [A,B,~,~] = linmod(trim,[0;0],0);
    [Aa,Ba,Cza,Dzua,Bwa,~] = AugmentedState(A,B);
    Kb = H_2(Aa, Ba, Bwa, Cza, Dzua);

    Kvec{end+1} = [Ka Kb];
end

K1 = Kvec{1};
K2 = Kvec{2};
K3 = Kvec{3};
K4 = Kvec{4};
K5 = Kvec{5};
K6 = Kvec{6};

yaw_ref = [t,ego_yaw];
vel_signal = [t,ego_v];

simulation = sim('AutomatedVehicle');

t_sim = simulation.u.Time;
t_plot = simulation.position.Time;

x = simulation.position.Data(:,2);
y = simulation.position.Data(:,1);

x_sim = x;
y_sim = y;

yaw_r = simulation.yaw.Data(:,1);
yaw_sim = simulation.yaw.Data(:,2);

errorH2 = yaw_r-yaw_sim;

psi_sim = rad2deg(yaw_sim);

u = simulation.u.Data(:,1);

% Yaw Comparison
figure('Name', 'Yaw Comparison - H2 Control', 'WindowState', 'maximized');
customRed = [0.6350, 0.0780, 0.1840]; 

hold on; 
plot(t_sim, yaw_r, '--', 'Color', customRed, 'LineWidth', 1.5, 'DisplayName', '$\psi_{ref}$');
plot(t_sim, yaw_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', '$\psi$');
grid on;       
grid minor;    
box on;        
axis tight; 

ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.1;
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Yaw Angle} $[rad]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Yaw Tracking Performance}', 'Interpreter', 'latex', 'FontSize', 18);

legend('show', 'Interpreter', 'latex', 'Location', 'best', 'FontSize', 12);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;

% Position Evolution (Non Regulated and Regulated)
figure('Name', 'H2 Control','WindowState', 'maximized'); 

ax = gca; 
plot(scenario, 'Parent', ax, 'Meshes', 'on'); 
hold(ax, 'on'); 

z_vec = 0.1*ones(size(ego_x));
plot3(ax, ego_x, ego_y, z_vec, 'b-', 'LineWidth', 2);

z_vec_xy = 0.1*ones(size(x));
plot3(ax, x, y, z_vec_xy, 'r--', 'LineWidth', 2);

view(ax, 2);      
axis(ax, 'equal'); 
grid(ax, 'on');
hold(ax, 'off');

% Steering Angle
figure('Name', 'Steering Angle Control - H2 Control', 'WindowState', 'maximized');

customPurple = [0.4940, 0.1840, 0.5560]; 

plot(t_sim, u, '-', 'Color', customPurple, 'LineWidth', 1.5);

grid on;       
grid minor;    
box on;       

axis tight; 

xlabel('\textbf{Simulation Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);

ylabel('\textbf{Steering Angle} $\delta \ [rad]$', 'Interpreter', 'latex', 'FontSize', 14);

set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

input('PRESS FOR H_infinity CONTROL!')

%% Hinfinity CONTROL
Kvec = {};

for i = 1:l
    Vx = V_interval{i}(1);
    [A,B,~,~] = linmod(trim,[0;0],0);
    [Aa,Ba,Cza,Dzua,Bwa,Dwa] = AugmentedState(A,B);
    Ka = Hinfinity(Aa,Ba,Bwa,Cza, Dwa, Dzua);

    Vx = V_interval{i}(2);
    [A,B,~,~] = linmod(trim,[0;0],0);
    [Aa,Ba,Cza,Dzua,Bwa,~] = AugmentedState(A,B);
    Kb = Hinfinity(Aa,Ba,Bwa,Cza, Dwa, Dzua);

    Kvec{end+1} = [Ka Kb];
end

K1 = Kvec{1};
K2 = Kvec{2};
K3 = Kvec{3};
K4 = Kvec{4};
K5 = Kvec{5};
K6 = Kvec{6};

simulation = sim('AutomatedVehicle');

t_sim = simulation.u.Time;

x = simulation.position.Data(:,2);
y = simulation.position.Data(:,1);

yaw_r = simulation.yaw.Data(:,1);
yaw_sim = simulation.yaw.Data(:,2);

errorInfinity = yaw_r-yaw_sim;

u = simulation.u.Data(:,1);

% Yaw Comparison
figure('Name', 'Yaw Comparison - H_infinity Control', 'WindowState', 'maximized');
customRed = [0.6350, 0.0780, 0.1840]; 

hold on; 
plot(t_sim, yaw_r, '--', 'Color', customRed, 'LineWidth', 1.5, 'DisplayName', '$\psi_{ref}$');
plot(t_sim, yaw_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', '$\psi$');
grid on;       
grid minor;    
box on;        
axis tight; 

ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.1;
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Yaw Angle} $[rad]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Yaw Tracking Performance}', 'Interpreter', 'latex', 'FontSize', 18);

legend('show', 'Interpreter', 'latex', 'Location', 'best', 'FontSize', 12);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;

% Position Evolution (Non Regulated and Regulated)
figure('Name', 'H_infinity Control','WindowState', 'maximized'); 

ax = gca; 
plot(scenario, 'Parent', ax, 'Meshes', 'on'); 
hold(ax, 'on'); 

z_vec = 0.1*ones(size(ego_x));
plot3(ax, ego_x, ego_y, z_vec, 'b-', 'LineWidth', 2);

z_vec_xy = 0.1*ones(size(x));
plot3(ax, x, y, z_vec_xy, 'r--', 'LineWidth', 2);

view(ax, 2);      
axis(ax, 'equal'); 
grid(ax, 'on');
hold(ax, 'off');

% Steering Angle
figure('Name', 'Steering Angle Control - H_infinity Control', 'WindowState', 'maximized');

customPurple = [0.4940, 0.1840, 0.5560]; 

plot(t_sim, u, '-', 'Color', customPurple, 'LineWidth', 1.5);

grid on;       
grid minor;    
box on;       

axis tight; 

xlabel('\textbf{Simulation Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);

ylabel('\textbf{Steering Angle} $\delta \ [rad]$', 'Interpreter', 'latex', 'FontSize', 14);

set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

input('PRESS FOR COMPARISON PLOTS!')

%% COMPARISON PLOTS
% Error Hinfinity
figure('Name', 'Yaw Error', 'WindowState', 'maximized', 'Color', 'w');
subplot(2,1,1)
customBlue = [0, 0.4470, 0.7410]; 
customRed = [0.6350, 0.0780, 0.1840]; 

hold on; 

plot(t_sim, errorInfinity, 'Color', customBlue, 'LineWidth', 1.8);

yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 

grid on;       
grid minor;    
box on;        

axis tight; 
ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.2; 
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Yaw Error} $[rad]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Yaw Tracking Error Performance (H$_{\infty}$)}', 'Interpreter', 'latex', 'FontSize', 18);

set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;

% Error H2
subplot(2,1,2)
customBlue = [0, 0.4470, 0.7410]; 
customRed = [0.6350, 0.0780, 0.1840]; 

hold on; 

plot(t_sim, errorH2, 'Color', customBlue, 'LineWidth', 1.8);

yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 

grid on;       
grid minor;    
box on;        

axis tight; 
ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.2; 
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Yaw Error} $[rad]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Yaw Tracking Error Performance (H$_2$)}', 'Interpreter', 'latex', 'FontSize', 18);

set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;


% x-position
figure('Name', 'Position Comparison - H2 Control', 'WindowState', 'maximized', 'Color', 'w');
subplot(2,1,1)

customRed = [0.6350, 0.0780, 0.1840]; 
customBlue = [0, 0.4470, 0.7410];     

hold on; 

plot(t, ego_x, '--', 'Color', customRed, 'LineWidth', 1.5, 'DisplayName', '$x_{ego}$');

plot(t_plot, x_sim, 'Color', customBlue, 'LineWidth', 1.8, 'DisplayName', '$x_{regulated}$');

grid on;       
grid minor;    
box on;        

axis tight; 
ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.1; 
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Longitudinal Position} $x$ $[m]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{X-Position Performance}', 'Interpreter', 'latex', 'FontSize', 18);

legend('show', 'Interpreter', 'latex', 'Location', 'best', 'FontSize', 12);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;

% y-position
subplot(2,1,2)
customRed = [0.6350, 0.0780, 0.1840]; 
customBlue = [0, 0.4470, 0.7410];     

hold on; 

plot(t, ego_x, '--', 'Color', customRed, 'LineWidth', 1.5, 'DisplayName', '$y_{ego}$');

plot(t_plot, x_sim, 'Color', customBlue, 'LineWidth', 1.8, 'DisplayName', '$y_{regulated}$');

grid on;       
grid minor;    
box on;        

axis tight; 
ylim_curr = ylim;
margin = (ylim_curr(2) - ylim_curr(1)) * 0.1; 
ylim([ylim_curr(1)-margin, ylim_curr(2)+margin]);

xlabel('\textbf{Time} $[s]$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('\textbf{Longitudinal Position} $x$ $[m]$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{Y-Position Performance}', 'Interpreter', 'latex', 'FontSize', 18);

legend('show', 'Interpreter', 'latex', 'Location', 'best', 'FontSize', 12);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);

hold off;