%% QUAD-COPTER
clear
close all
clc

% Parameters
g = 9.81;
m = 0.8;
l = 0.3;
Ixx = 0.01567;
Iyy = 0.01567;
Izz = 0.02834;
b = 1.9232e-5;
d = 4.003e-7;
Jr = 6.01e-5;


% Simulation Time
Tsim = 14;

% Wind Effect
wind = 0;
 
% Initial Conditions
% x = [x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi,
% psi_dot]

x0 = [0.2 0 0.3 0 0.15 0 0.2 0 0.1 0 -0.1 0]';

model = 'QuadCopter';

% Linearization
xeq = [0 0 0 0 0 0 0 0 0 0 0 0]';
ueq = [m*g, 0, 0, 0]';
[A, B, C, D] = linmod(model, xeq, ueq);

Bw = zeros(12,6);
Bw(2,1) = 1/m;         
Bw(4,2) = 1/m;         
Bw(6,3) = 1/m;          

% Eigenvalues
eigA = eig(A);

% Transfer Function and Poles
sys = ss(A,B,C,D);
G = tf(sys);
p = pole(sys);

% Reachability
R = ctrb(A,B);
rankR = rank(R);

% Observability
O = obsv(A,C);
rankO = rank(O);

% Model
model = 'FeedbackControl';

%% FREE RESPONSE
% Simulation
simulation = sim('Free_Response');

phi = simulation.freeResponse.Data(:,7);
phi_dot = simulation.freeResponse.Data(:,8);
theta = simulation.freeResponse.Data(:,9);
theta_dot = simulation.freeResponse.Data(:,10);
psi = simulation.freeResponse.Data(:,11);
psi_dot = simulation.freeResponse.Data(:,12);
x = simulation.freeResponse.Data(:,1);
x_dot = simulation.freeResponse.Data(:,2);
y = simulation.freeResponse.Data(:,3);
y_dot = simulation.freeResponse.Data(:,4);
z = simulation.freeResponse.Data(:,5);
z_dot = simulation.freeResponse.Data(:,6);

t = simulation.freeResponse.Time;


% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Free Response of Quad-Copter}','Interpreter','latex')

input('PID CONTROL')
close all

%% CONTROL (PID CONTROLLER)
Kp_p = 2;
Ki_p = 0.3;
Kd_p = 1.1;

Kp_z = 35;
Ki_z = 14.5;
Kd_z = 9;

Kp_a = 13;
Ki_a = 0.5;
Kd_a = 1.5;

% Simulation
simulation = sim('PID_Control');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_pid = {u1, u2, u3, u4};
t_pid = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('\textbf{Angular Speed of Quad-Copter}','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with PID Controller of Quad-Copter}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter}','Interpreter','latex')

input('STABLITY ANALYSIS')
close all

%% CONTROL SYNTHESIS 
control = Control(A,B,Bw);

%% Check Stability
P = control.LMI_stability();

input('L1 CONTROL (DISTURBANCE FREE)')

%% L_1 Control
wind = 0;
w_period = 7;       % 7% of Tsim


lambda = 0.5;
C33 = diag([8, 0.1, 8, 0.1, 8, 0.1, 4, 0.1, 4, 0.1, 2, 0.1]);
D31 = 0.1 * eye(12, 6); 
D32 = [zeros(12-4, 4); diag([0.1, 1, 1, 1])];

K = control.L_1(C33,D31,D32,lambda);
K_1 = K;

% Eigenvalues
eig_L1 = eig(A + B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_l1 = {u1, u2, u3, u4};
t_l1 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{L}_1$ \textbf{Controller of Quad-Copter (Disturbance Free)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{L}_{1}$ \textbf{Control (Disturbance Free)}','Interpreter','latex')

input('H2 CONTROL (DISTURBANCE FREE)')

%% H_2 Control 
C22 = diag([5, 0.1, 5, 0.1, 5, 0.1, 3, 0.1, 3, 0.1, 1, 0.1]);
D22 = [zeros(12-4, 4); diag([0.05, 0.5, 0.5, 0.5])];

K = control.H_2(C22,D22);

K_2 = K;

% Eigenvalues
eig_H2 = eig(A + B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_h2 = {u1, u2, u3, u4};
t_h2 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_2$ \textbf{Controller of Quad-Copter (Disturbance Free)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{2}$ \textbf{Control (Disturbance Free)}','Interpreter','latex')

input('Hinfinity CONTROL (DISTURBANCE FREE)')
close all

%% H_infinity Control
D11 = zeros(12, 6);
C11 = diag([8, 0.1, 8, 0.1, 10, 0.1, 3, 0.1, 3, 0.1, 1, 0.1]);
D12 = [zeros(12-4, 4); diag([3, 30, 550, 550])];

K = control.H_inf(C11,D11,D12);
K_inf = K;

% Eigenvalues
eig_inf = eig(A + B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_inf = {u1, u2, u3, u4};
t_inf = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_{\infty}$ \textbf{Controller of Quad-Copter (Disturbance Free)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}$ \textbf{Control (Disturbance Free)}','Interpreter','latex')

input('COMPARISON (DISTURBANCE FREE)')
close all

%% COMPARISON CONTROLLER (NO DISTURBANCE)
figure
subplot(2,2,1)
plot(t_pid,u_pid{1},'LineWidth',1.5)
hold on
plot(t_l1,u_l1{1},'LineWidth',1.5)
plot(t_h2,u_h2{1},'LineWidth',1.5)
plot(t_inf,u_inf{1},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
legend('$u_{1,PID}(t)$','$u_{1,\mathcal{L}_1}(t)$','$u_{1,\mathcal{H}_2}(t)$','$u_{1,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t_pid,u_pid{2},'LineWidth',1.5)
hold on
plot(t_l1,u_l1{2},'LineWidth',1.5)
plot(t_h2,u_h2{2},'LineWidth',1.5)
plot(t_inf,u_inf{2},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{2,PID}(t)$','$u_{2,\mathcal{L}_1}(t)$','$u_{2,\mathcal{H}_2}(t)$','$u_{2,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t_pid,u_pid{3},'LineWidth',1.5)
hold on
plot(t_l1,u_l1{3},'LineWidth',1.5)
plot(t_h2,u_h2{3},'LineWidth',1.5)
plot(t_inf,u_inf{3},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{3,PID}(t)$','$u_{3,\mathcal{L}_1}(t)$','$u_{3,\mathcal{H}_2}(t)$','$u_{3,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t_pid,u_pid{4},'LineWidth',1.5)
hold on
plot(t_l1,u_l1{4},'LineWidth',1.5)
plot(t_h2,u_h2{4},'LineWidth',1.5)
plot(t_inf,u_inf{4},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{4,PID}(t)$','$u_{4,\mathcal{L}_1}(t)$','$u_{4,\mathcal{H}_2}(t)$','$u_{4,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of all Inputs of the Quad-Copter (Disturbance Free)}','Interpreter','latex')

input('L1 CONTROL (SCENARIO 1)')
close all

%% CONTROL (WITH DISTURBANCE) - SCENARIO 1
x0 = zeros(12,1);

wind = 0.4;
Tsim = 18;
w_period = 7;       % 7% of Tsim

%% L1 (WITH DISTURBANCE) - SCENARIO 1

% Simulation
K = K_1;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xl1 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

w = simulation.wind.Data;

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_l1 = {u1, u2, u3, u4};
ul11 = u_l1;
t_l1 = t;
tl11 = t_l1;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{L}_1$ \textbf{Controller of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{L}_{1}$ \textbf{Control (Scenario 1)}','Interpreter','latex')

figure
plot(t,w,'k','LineWidth',1.5)
sgtitle('\textbf{Wind effect on Quad-Copter}','Interpreter','latex')
grid

input('H2 CONTROL (SCENARIO 1)')
close all

%% H2 (WITH DISTURBANCE) - SCENARIO 1
% Simulation
K = K_2;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xh2 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_h2 = {u1, u2, u3, u4};
uh21 = u_h2;
t_h2 = t;
th21 = t_h2;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_2$ \textbf{Controller of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{2}$ \textbf{Control (Scenario 1)}','Interpreter','latex')

input('Hinfinity CONTROL (SCENARIO 1)')
close all

%% H_infinity (WITH DISTURBANCE) - SCENARIO 1
% Simulation
K = K_inf;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

x_inf = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_inf = {u1, u2, u3, u4};
uinf1 = u_inf;
t_inf = t;
tinf1 = t_inf;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_{\infty}$ \textbf{Controller of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}$ \textbf{Control (Scenario 1)}','Interpreter','latex')

input('COMPARISON (SCENARIO 1)')
close all

%% COMPARISON CONTROLLER (WITH DISTURBANCE) - SCENARIO 1
figure
subplot(2,2,1)
plot(t_l1,u_l1{1},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{1},'LineWidth',1.5)
plot(t_inf,u_inf{1},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
legend('$u_{1,\mathcal{L}_1}(t)$','$u_{1,\mathcal{H}_2}(t)$','$u_{1,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t_l1,u_l1{2},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{2},'LineWidth',1.5)
plot(t_inf,u_inf{2},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{2,\mathcal{L}_1}(t)$','$u_{2,\mathcal{H}_2}(t)$','$u_{2,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t_l1,u_l1{3},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{3},'LineWidth',1.5)
plot(t_inf,u_inf{3},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{3,\mathcal{L}_1}(t)$','$u_{3,\mathcal{H}_2}(t)$''$u_{3,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t_l1,u_l1{4},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{4},'LineWidth',1.5)
plot(t_inf,u_inf{4},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{4,\mathcal{L}_1}(t)$','$u_{4,\mathcal{H}_2}(t)$','$u_{4,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of all Inputs of the Quad-Copter (Scenario 1)}','Interpreter','latex')

input('L1 CONTROL (SCENARIO 2)')
close all

%% CONTROL (WITH DISTURBANCE) - SCENARIO 2
wind = 0.9;
Tsim = 25;
w_period = 15;       % 15% of Tsim

%% L1 (WITH DISTURBANCE) - SCENARIO 2
% Simulation
K = K_1;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

w = simulation.wind.Data;

xl12 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_l1 = {u1, u2, u3, u4};
ul12 = u_l1;
t_l1 = t;
tl12 = t_l1;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{L}_1$ \textbf{Controller of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{L}_{1}$ \textbf{Control (Scenario 2)}','Interpreter','latex')

figure
plot(t,w,'k','LineWidth',1.5)
sgtitle('\textbf{Wind effect on Quad-Copter}','Interpreter','latex')
grid

input('H2 CONTROL (SCENARIO 2)')
close all

%% H2 (WITH DISTURBANCE) - SCENARIO 2
% Simulation
K = K_2;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xh22 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_h2 = {u1, u2, u3, u4};
uh22 = u_h2;
t_h2 = t;
th22 = t_h2;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_2$ \textbf{Controller of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{2}$ \textbf{Control (Scenario 2)}','Interpreter','latex')

input('Hinfinity CONTROL (SCENARIO 2)')
close all

%% H_infinity (WITH DISTURBANCE) - SCENARIO 2
% Simulation
K = K_inf;
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xinf2 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_inf = {u1, u2, u3, u4};
uinf2 = u_inf;
t_inf = t;
tinf2 = t_inf;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Control with} $\mathcal{H}_{\infty}$ \textbf{Controller of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}$ \textbf{Control (Scenario 2)}','Interpreter','latex')

input('COMPARISON (SCENARIO 2)')
close all

%% COMPARISON CONTROLLER (WITH DISTURBANCE) - SCENARIO 2
figure
subplot(2,2,1)
plot(t_l1,u_l1{1},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{1},'LineWidth',1.5)
plot(t_inf,u_inf{1},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
legend('$u_{1,\mathcal{L}_1}(t)$','$u_{1,\mathcal{H}_2}(t)$','$u_{1,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t_l1,u_l1{2},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{2},'LineWidth',1.5)
plot(t_inf,u_inf{2},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{2,\mathcal{L}_1}(t)$','$u_{2,\mathcal{H}_2}(t)$','$u_{2,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t_l1,u_l1{3},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{3},'LineWidth',1.5)
plot(t_inf,u_inf{3},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{3,\mathcal{L}_1}(t)$','$u_{3,\mathcal{H}_2}(t)$','$u_{3,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t_l1,u_l1{4},'LineWidth',1.5)
hold on
plot(t_h2,u_h2{4},'LineWidth',1.5)
plot(t_inf,u_inf{4},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{4,\mathcal{L}_1}(t)$','$u_{4,\mathcal{H}_2}(t)$','$u_{4,\mathcal{H}_\infty}(t)$','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of all Inputs of the Quad-Copter (Scenario 2)}','Interpreter','latex')

input('Hinfinity/H2 CONTROL - E-CONSTRAINED (SCENARIO 1)')
close all

%% MULTI-OBJECTIVE CONTROL (H_inf/H2) - SCENARIO 1
wind = 0.4;
Tsim = 18;
w_period = 7; 

a = 5;
par = [5 3];

%% H_infinity/H2 CONTROL (E-CONSTRAINED) - SCENARIO 1
K = control.Hinf_H2_multiObjective(C11, D12, D11, C22, D22, a);
Kmo_econ = K;

% Eigenvalues
eig_HinfH2 = eig(A + B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo1 = {u1, u2, u3, u4};
t_mo1 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective ($\epsilon$-constrained)} $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{Control (Scenario 1)}','Interpreter','latex')

input('Hinfinity/H2 CONTROL - SCALARIZATION (SCENARIO 1)')

%% H_infinity/H2 CONTROL (SCALARIZATION) - SCENARIO 1
K = control.Hinf_H2_multiObjective(C11, D12, D11, C22, D22, par);
Kmo_scal = K;

% Eigenvalues
eig_HinfH2_2 = eig(A + B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo2 = {u1, u2, u3, u4};
t_mo2 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective (Scalarization)} $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{Control (Scenario 1)}','Interpreter','latex')

input('Hinfinity/H2 CONTROL - COMPARISON (SCENARIO 1)')
close all

%% COMPARISON Hinf/H2 (E-CONSTRAINED/SCALARIZATION) - SCENARIO 1
figure
subplot(2,2,1)
plot(t_mo1,u_mo1{1},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{1},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
legend('$u_{1,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{1,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t_mo1,u_mo1{2},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{2},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{2,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{2,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t_mo1,u_mo1{3},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{3},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{3,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{3,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t_mo1,u_mo1{4},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{4},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{4,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{4,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of all Inputs of the Quad-Copter (Scenario 1)}','Interpreter','latex')


input('Hinfinity/H2 CONTROL - E-CONSTRAINED (SCENARIO 2)')
close all

Tsim = 25;
w_period = 15;
wind = 0.9;

%% H_infinity/H2 CONTROL (E-CONSTRAINED) - SCENARIO 2
K = Kmo_econ;

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo1 = {u1, u2, u3, u4};
t_mo1 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective ($\epsilon$-constrained)} $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{Control (Scenario 2)}','Interpreter','latex')

input('Hinfinity/H2 CONTROL - SCALARIZATION (SCENARIO 2)')
close all

%% H_infinity/H2 CONTROL (SCALARIZATION) - SCENARIO 2
K = Kmo_scal;

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo2 = {u1, u2, u3, u4};
t_mo2 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective (Scalarization)} $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/\mathcal{H}_2$ \textbf{Control (Scenario 2)}','Interpreter','latex')

input('Hinfinity/H2 CONTROL - COMPARISON (SCENARIO 2)')
close all

%% COMPARISON Hinf/H2 (E-CONSTRAINED/SCALARIZATION) - SCENARIO 2
figure
subplot(2,2,1)
plot(t_mo1,u_mo1{1},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{1},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
legend('$u_{1,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{1,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t_mo1,u_mo1{2},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{2},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{2,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{2,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t_mo1,u_mo1{3},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{3},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{3,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{3,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t_mo1,u_mo1{4},'LineWidth',1.5)
hold on
plot(t_mo2,u_mo2{4},'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
legend('$u_{4,\mathcal{H}_\infty/\mathcal{H}_2}(t) (\epsilon-constrained)$','$u_{4,\mathcal{H}_\infty/\mathcal{H}_2}(t) (scalarization)$','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of all Inputs of the Quad-Copter (Scenario 2)}','Interpreter','latex')

input('Hinfinity/r-STABILITY CONTROL (SCENARIO 1)')
close all

%% MULTI-OBJECTIVE CONTROL (H_inf/r-stability) - SCENARIO 1
wind = 0.4;
Tsim = 18;
w_period = 7; 

alpha = 0.5;
th = pi/4;
r = 20;

K = control.Hinf_regional_stability(C11,D11,D12,alpha,th,r);
Kmo_rst = K;

% Eigenvalues
eig_ir = eig(A+B*K)

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo1 = {u1, u2, u3, u4};
t_mo1 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective} $\mathcal{H}_{\infty}/r$\textbf{-stability of Quad-Copter (Scenario 1)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/r$\textbf{-stability Control (Scenario 1)}','Interpreter','latex')

input('Hinfinity/r-STABILITY CONTROL (SCENARIO 2)')
close all

%% MULTI-OBJECTIVE CONTROL (H_inf/r-stability) - SCENARIO 2
wind = 0.9;
Tsim = 25;
w_period = 15; 

% Simulation
simulation = sim(model);

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u_mo1 = {u1, u2, u3, u4};
t_mo1 = t;

% Plot
figure
subplot(2,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t,theta,'LineWidth',1.5)
plot(t,psi,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angles [$rad$]','Interpreter','latex')
legend({'$\phi$ - Roll','$\theta$ - Pitch','$\psi$ - Yaw'},'Interpreter','latex')
title ('Angular Position of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,x,'LineWidth',1.5)
hold on
plot(t,y,'LineWidth',1.5)
plot(t,z,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Positions [$m$]','Interpreter','latex')
legend({'$x$-Position','$y$-Position','$z$-Position'},'Interpreter','latex')
title ('Inertial Position of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t,theta_dot,'LineWidth',1.5)
plot(t,psi_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Angular Speeds [$rad/s$]','Interpreter','latex')
legend({'$\dot{\phi}$ - Roll Speed','$\dot{\theta}$ - Pitch Speed','$\dot{\psi}$ - Yaw Speed'},'Interpreter','latex')
title ('Angular Speed of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t,y_dot,'LineWidth',1.5)
plot(t,z_dot,'LineWidth',1.5)
hold off
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Inertial Speeds [$m/s$]','Interpreter','latex')
legend({'$x$-Speed','$y$-Speed','$z$-Speed'},'Interpreter','latex')
title ('Inertial Speed of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Multi-Objective} $\mathcal{H}_{\infty}/r$\textbf{-stability of Quad-Copter (Scenario 2)}','Interpreter','latex')

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Inputs of the Quad-Copter} - $\mathcal{H}_{\infty}/r$\textbf{-stability Control (Scenario 2)}','Interpreter','latex')

figure(3)
grid on; hold on
xline(-alpha,'-r','LineWidth',1.5)
xlim([-22 4])
ylim([-15 15])
xlabel('$Re(z)$','Interpreter','latex')
ylabel('$Im(z)$','Interpreter','latex')
sgtitle('\textbf{LMI Region with} $\alpha = 0.5$, $\theta = \frac{\pi}{4}$, $r = 20$','Interpreter','latex')

% Plot LMI Region
for i = 1:12
    z = eig_ir(i);
    figure(3)
    plot(real(z),imag(z), 'bo', 'MarkerSize', 3,'LineWidth',1.5);
end

t = linspace(pi - th, pi + th, 100);

x_arc = r * cos(t);
y_arc = r * sin(t);

x_cone = [0, x_arc, 0];
y_cone = [0, y_arc, 0];

plot(x_cone, y_cone, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Cono Limite');
patch(x_cone, y_cone, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

input('ROBUST Hinfinity CONTROL (SCENARIO 1)')
close all

%% ROBUST CONTROL 
mv = [0.7, 1];
m_vec = [1, 0.9, 0.8, 0.7];

B_unc = @(m) [ 0 0 0 0;
               0 0 0 0;
               0 0 0 0;
               0 0 0 0;
               0 0 0 0;
               1/m 0 0 0;
               0 0 0 0;
               0 l/Ixx 0 0;
               0 0 0 0;
               0 0 l/Iyy 0;
               0 0 0 0;
               0 0 0 1/Izz];

Bw_unc = @(m) [ 0 0 0 0 0 0;
                1/m 0 0 0 0 0;
                0 0 0 0 0 0;
                0 1/m 0 0 0 0;
                0 0 0 0 0 0;
                0 0 1/m 0 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;];

Br = {};
Bwr = {};
Ar = {};

for i = 1:2
    mi = mv(i);
    Bi = B_unc(mi);
    Bwi = Bw_unc(mi);
    Ar{end+1} = A;
    Br{end+1} = Bi;
    Bwr{end+1} = Bwi;
end

robControl = RobustControl(Ar,Br,Bwr);

%% ROBUST H-infinity - SCENARIO 1
wind = 0.4;
Tsim = 25;
w_period = 5; 

K = robControl.H_infinity(C11,D11,D12);
Kinf_rob = K;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xinf_rob = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

tinf_rob = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

uinf_rob = {u1,u2,u3,u4};

input('ROBUST H2 CONTROL (SCENARIO 1)')

%% ROBUST H2 - SCENARIO 1
K = robControl.H_2(C22,D22);
K2_rob = K;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

x2_rob = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;
t2_rob = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u2_rob = {u1, u2, u3, u4};

input('ROBUST L1 CONTROL (SCENARIO 1)')

%% ROBUST L1 - SCENARIO 1
K = robControl.L_1(C33, D31, D32, lambda);
K1_rob = K;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

x1_rob = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;
t1_rob = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u1_rob = {u1, u2, u3, u4};

input('ROBUST Hinfinity CONTROL (SCENARIO 2)')

%% ROBUST CONTROL - SCENARIO 2

wind = 0.9;
Tsim = 25;
w_period = 15; 

%% ROBUST H_infinity - SCENARIO 2
K = Kinf_rob;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

xinf_rob2 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;

tinf_rob2 = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

uinf_rob2 = {u1, u2, u3, u4};

input('ROBUST H2 CONTROL (SCENARIO 2)')

%% ROBUST H2 - SCENARIO 2
K = K2_rob;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

x2_rob2 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};


t = simulation.Contr.Time;
t2_rob2 = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u2_rob2 = {u1, u2, u3, u4};

input('ROBUST L1 CONTROL (SCENARIO 2)')

%% ROBUST L1 - SCENARIO 2
K = K1_rob;

K1 = K; K2 = K; K3 = K; K4 = K;

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

x1_rob2 = {x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot};

t = simulation.Contr.Time;
t1_rob2 = t;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

u1_rob2 = {u1, u2, u3, u4};

input('Hinfinity CONTROL - GAIN SCHEDULING (SCENARIO 1)')

%% LPV and GAIN SCHEDULING - Scenario 1
l = size(m_vec,2);

vecControl = cell(1,l);

for i = 1:l-1
    mi = m_vec(i);
    mi1 = m_vec(i+1);
    Bi = B_unc(mi);
    Bi1 = B_unc(mi1);
    Bwi = Bw_unc(mi);
    Bwi1 = Bw_unc(mi1);

    Ari = {A, A}; Bri = {Bi, Bi1}; Bwri = {Bwi, Bwi1};
    vecControl{i} = RobustControl(Ari,Bri,Bwri);
end

vecControl{l} = Control(A, B_unc(m_vec(l)),Bw_unc(m_vec(l)));

%% GAIN SCHEDULING (H_infinity) - SCENARIO 1 
K1 = vecControl{1}.H_infinity(C11,D11,D12);
K2 = vecControl{2}.H_infinity(C11,D11,D12);
K3 = vecControl{3}.H_infinity(C11,D11,D12);
K4 = vecControl{4}.H_inf(C11,D11,D12);

wind = 0.4;
Tsim = 25;
w_period = 5; 

% Simulation
simulation = sim('GainScheduling');

t = simulation.massSignal.Time;
mSig = simulation.massSignal.Data;

figure 
plot(t, mSig,'LineWidth',1.5);
grid
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Mass [$kg$]','Interpreter','latex')

sgtitle('\textbf{Time-Varying Mass of the Quad-Copter}','Interpreter','latex')

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);


u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

% Plot
figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(tinf_rob,uinf_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(tinf_rob,uinf_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(tinf_rob,uinf_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(tinf_rob,uinf_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{5},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{6},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{H}_{\infty} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')


figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{7},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{8},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{9},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{10},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{11},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(tinf_rob,xinf_rob{12},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{H}_{\infty} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')


input('Hinfinity CONTROL - GAIN SCHEDULING (SCENARIO 2)')
close all

%% GAIN SCHEDULING (H_infinity)- SCENARIO 2 
wind = 0.9;
Tsim = 25;
w_period = 15; 

% Simulation
simulation = sim('GainScheduling');

t = simulation.Contr.Time;

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);


% Plot
figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(tinf_rob2,uinf_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(tinf_rob2,uinf_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(tinf_rob2,uinf_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(tinf_rob2,uinf_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{5},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{6},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{H}_{\infty} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{7},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{8},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{9},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{10},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{11},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(tinf_rob2,xinf_rob2{12},'LineWidth',1.5)
grid
legend('$\mathcal{H}_\infty$','$\mathcal{H}_\infty$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{H}_{\infty} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

input('H2 CONTROL - GAIN SCHEDULING (SCENARIO 1)')

%% GAIN SCHEDULING (H2) - SCENARIO 1 
K1 = vecControl{1}.H_2(C22,D22);
K2 = vecControl{2}.H_2(C22,D22);
K3 = vecControl{3}.H_2(C22,D22);
K4 = vecControl{4}.H_2(C22,D22);

wind = 0.4;
Tsim = 25;
w_period = 5; 

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

% Plot
figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(t2_rob,u2_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(t2_rob,u2_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(t2_rob,u2_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(t2_rob,u2_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')


subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{5},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{6},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{H}_{2} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')


figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{7},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{8},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{9},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{10},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{11},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(t2_rob,x2_rob{12},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{H}_{2} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')


input('H2 CONTROL - GAIN SCHEDULING (SCENARIO 2)')
close all

%% GAIN SCHEDULING (H2)- SCENARIO 2 
wind = 0.9;
Tsim = 25;
w_period = 15; 

% Simulation
simulation = sim('GainScheduling');

t = simulation.Contr.Time;

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);


% Plot
figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(t2_rob2,u2_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(t2_rob2,u2_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(t2_rob2,u2_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(t2_rob2,u2_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{5},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{6},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{H}_{2} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')


figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{7},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{8},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{9},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{10},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{11},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(t2_rob2,x2_rob2{12},'LineWidth',1.5)
grid
legend('$\mathcal{H}_2$','$\mathcal{H}_2$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{H}_{2} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

input('L1 CONTROL - GAIN SCHEDULING (SCENARIO 1)')
close all

%% GAIN SCHEDULING (L1) - SCENARIO 1 
K1 = vecControl{1}.L_1(C33, D31, D32, lambda);
K2 = vecControl{2}.L_1(C33, D31, D32, lambda);
K3 = vecControl{3}.L_1(C33, D31, D32, lambda);
K4 = vecControl{4}.L_1(C33, D31, D32, lambda);

wind = 0.4;
Tsim = 25;
w_period = 5; 

% Simulation
simulation = sim('GainScheduling');

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

t = simulation.Contr.Time;

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

% Plot

figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(t1_rob,u1_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(t1_rob,u1_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(t1_rob,u1_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(t1_rob,u1_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{1},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{2},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{3},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{4},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{5},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{6},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{L}_{1} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{7},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{8},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{9},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{10},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{11},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(t1_rob,x1_rob{12},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{L}_{1} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 1)}','Interpreter','latex')


input('L1 CONTROL - GAIN SCHEDULING (SCENARIO 2)')
close all

%% GAIN SCHEDULING (L1)- SCENARIO 2 
wind = 0.9;
Tsim = 25;
w_period = 15; 

% Simulation
simulation = sim('GainScheduling');

t = simulation.Contr.Time;

phi = simulation.Contr.Data(:,7);
phi_dot = simulation.Contr.Data(:,8);
theta = simulation.Contr.Data(:,9);
theta_dot = simulation.Contr.Data(:,10);
psi = simulation.Contr.Data(:,11);
psi_dot = simulation.Contr.Data(:,12);
x = simulation.Contr.Data(:,1);
x_dot = simulation.Contr.Data(:,2);
y = simulation.Contr.Data(:,3);
y_dot = simulation.Contr.Data(:,4);
z = simulation.Contr.Data(:,5);
z_dot = simulation.Contr.Data(:,6);

u1 = simulation.Inputs.Data(:,1);
u2 = simulation.Inputs.Data(:,2);
u3 = simulation.Inputs.Data(:,3);
u4 = simulation.Inputs.Data(:,4);

% Plot
figure
subplot(2,2,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(t1_rob2,u1_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Thrust [$N$]','Interpreter','latex')
title ('Total Thrust of Quad-Copter','Interpreter','latex')

subplot(2,2,2)
plot(t,u2,'LineWidth',1.5)
hold on
plot(t1_rob2,u1_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Roll Torque [$N\cdot m$]','Interpreter','latex')
title ('Roll Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,3)
plot(t,u3,'LineWidth',1.5)
hold on
plot(t1_rob2,u1_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Pitch Torque [$N\cdot m$]','Interpreter','latex')
title ('Pitch Torque of Quad-Copter','Interpreter','latex')

subplot(2,2,4)
plot(t,u4,'LineWidth',1.5)
hold on
plot(t1_rob2,u1_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Yaw Torque [$N\cdot m$]','Interpreter','latex')
title ('Yaw Torque of Quad-Copter','Interpreter','latex')

sgtitle('\textbf{Comparison of the inputs of the Quad-Copter with - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,x,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{1},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$x$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,x_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{2},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$x$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,y,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{3},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$y$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,y_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{4},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$y$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,z,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{5},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$m$]','Interpreter','latex')
title('$z$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,z_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{6},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$m/s$]','Interpreter','latex')
title('$z$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Inertial Reference Frame} $\mathcal{L}_{1} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

figure
subplot(3,2,1)
plot(t,phi,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{7},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\phi$-Position Evolution','Interpreter','latex')

subplot(3,2,2)
plot(t,phi_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{8},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\phi$-Velocity Evolution','Interpreter','latex')

subplot(3,2,3)
plot(t,theta,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{9},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\theta$-Position Evolution','Interpreter','latex')

subplot(3,2,4)
plot(t,theta_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{10},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\theta$-Velocity Evolution','Interpreter','latex')

subplot(3,2,5)
plot(t,psi,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{11},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Position [$rad$]','Interpreter','latex')
title('$\psi$-Position Evolution','Interpreter','latex')

subplot(3,2,6)
plot(t,psi_dot,'LineWidth',1.5)
hold on
plot(t1_rob2,x1_rob2{12},'LineWidth',1.5)
grid
legend('$\mathcal{L}_1$','$\mathcal{L}_1$ - Static','Interpreter','latex')
xlabel('Time [$s$]','Interpreter','latex')
ylabel('Velocity [$rad/s$]','Interpreter','latex')
title('$\psi$-Velocity Evolution','Interpreter','latex')

sgtitle('\textbf{Comparison of Rotational Reference Frame} $\mathcal{L}_{1} $\textbf{ Control of Quad-Copter - Gain Scheduling vs Static (Scenario 2)}','Interpreter','latex')

