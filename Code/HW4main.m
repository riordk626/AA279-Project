%% Load Model
clc, clear
close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

I_sim = Itotal_p;
Tfinal = 300;
axesFlag = 0;
dynamicsType="default";
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput('aquaMasterModel');
simIn.ExternalInput = M;

%% Problem 1 - Equilibrium Analysis

% Part a - Inertial Alignment 

u0 = [0,1e-9,0].';
attitudeType="euler";
om0 = deg2rad([10 0 0]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';
u = squeeze(simOut.u);

% q = squeeze(simOut.q);
% 
% u = zeros([3 size(q,2)]);
% for i=1:size(u,2)
%     u(:,i) = RtoEuler313(R_ItoP(:,:,i));
% end

figure
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/equilibrium_inertial_velocities.png')
legend

figure
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/equilibrium_inertial_angles.png')
legend

% Part b - RTN Alignment
% r0 = [] INITIAL ORBIT RADIUS IN ECI
% v0 = [] INITIAL VELOCITY IN ECI
R0 = eci2rtn(r0, v0);
u0 = RtoEuler313(R0);
attitudeType="euler";

om0 = deg2rad([0 0 10]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
% R_ECItoRTN = simOut.yout{2}.Values.Data; % ORBIT DCM OUTPUT
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';

u = zeros([3 size(t,1)]);
for i=1:size(u,2)
    R = R_ItoP(:,:,i) * R_ECItoRTN.';
    u(:,i) = RtoEuler313(R);
end

figure
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/equilibrium_RTN_velocities.png')
legend

figure
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/equilibrium_RTN_angles.png')
legend

%% Problem 2 - Stability Test

rng(10)
om0_array = deg2rad(10.*eye(3)) + 0.01.*rand([3 3]);
u0 = [0,1e-9,0].';
u0 = u0 + 0.01.*rand(size(u0));

attitudeType="euler";

fomega = figure();
fangles = figure();

for i=1:3
    om0 = om0_array(:,i);
    load_system("aquaMasterModel")
    
    simOut = sim(simIn);

    R_ItoP = simOut.yout{1}.Values.Data;
    t = simOut.t;
    n = size(t,1);
    om_p = squeeze(simOut.om_p).';
    u = squeeze(simOut.u);

    figure(fomega.Number)
    subplot(3,1,i)
    aplot = plot(t, om_p, 'LineWidth', 2);
    set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
    ylabel('\omega [rad/s]')
    hold on
    if i==3
        xlabel('t [sec]')
        legend
    end
    ax = gca();
    ax.FontSize = 14;
    f1 = gcf();


    figure(fangles.Number)
    subplot(3,1,i)
    aplot = plot(t, u, 'LineWidth', 2);
    set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
    ylabel('u [rad]')
    hold on
    if i==3
        xlabel('t [sec]')
        legend
    end
    ax = gca();
    ax.FontSize = 14;
    f2 = gcf();
end

exportgraphics(fomega, '../Images/PS4/stability_history_velocity.png')
exportgraphics(fangles, '../Images/PS4/stability_history_angles.png')

%% Problem 3 - Momentume Wheel

dynamicsType="wheel";

Ir = 1;
omr = 1;

% Momentum Wheel Equilibrium Analysis

r = [0 0 1].';

% Part a - Inertial Alignment 
u0 = [0,1e-9,0].';
om0 = deg2rad([0 0 10]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
% R_ItoRTN = simout.yout{2}.Values.Data;
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';
u = squeeze(simOut.u);

figure
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/mom_wheel_equilibrium_inertial_velocities.png')
legend

figure
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/mom_wheel_equilibrium_inertial_angles.png')
legend

% Part b - RTN Alignment

% r0 = [] INITIAL ORBIT RADIUS IN ECI
% v0 = [] INITIAL VELOCITY IN ECI
R0 = eci2rtn(r0, v0);
u0 = RtoEuler313(R0);
attitudeType="euler";
om0 = deg2rad([0 0 10]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
% R_ItoRTN = simout.yout{2}.Values.Data;
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';

u = zeros([3 size(t,1)]);
for i=1:size(u,2)
    R = R_ItoP(:,:,i) * R_ECItoRTN.';
    u(:,i) = RtoEuler313(R);
end

figure
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/mom_wheel_equilibrium_RTN_velocities.png')
legend

figure
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
xlabel('t [sec]')
ax = gca();
ax.FontSize = 14;
exportgraphics(gcf, '../Images/PS4/mom_wheel_equilibrium_RTN_angles.png')
legend

% Momentum Stability Test

rng(10)
om0_array = deg2rad(10.*eye(3)) + 0.01.*rand([3 3]);
r_array = eye(3);
Ir = 1;
omr = 1;
omr = omr + 0.01.*rand(1);
u0 = [0,1e-9,0].';
u0 = u0 + 0.01.*rand(size(u0));

fomega = figure();
fangles = figure();

for i=1:3
    om0 = om0_array(:,i);
    r = r(:,i);
    load_system("aquaMasterModel")
    
    simOut = sim(simIn);

    R_ItoP = simOut.yout{1}.Values.Data;
    t = simOut.t;
    n = size(t,1);
    om_p = squeeze(simOut.om_p).';
    u = squeeze(simOut.u);

    figure(fomega.Number)
    subplot(3,1,i)
    aplot = plot(t, om_p, 'LineWidth', 2);
    set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
    ylabel('\omega [rad/s]')
    hold on
    if i==3
        xlabel('t [sec]')
        legend
    end
    ax = gca();
    ax.FontSize = 14;
    f1 = gcf();


    figure(fangles.Number)
    subplot(3,1,i)
    aplot = plot(t, u, 'LineWidth', 2);
    set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
    ylabel('u [rad]')
    hold on
    if i==3
        xlabel('t [sec]')
        legend
    end
    ax = gca();
    ax.FontSize = 14;
    f2 = gcf();
end

exportgraphics(fomega, '../Images/PS4/mom_wheel_stability_history_velocity.png')
exportgraphics(fangles, '../Images/PS4/mom_wheel_stability_history_angles.png')

% Intermediate Stability

Ir = 1;
r = [0 1 0].';
om0 = deg2rad([0 10 0]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';
u = squeeze(simOut.u);

figure(fomega.Number)
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
hold on
xlabel('t [sec]')
legend
ax = gca();
ax.FontSize = 14;
f1 = gcf();


figure(fangles.Number)
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
hold on
xlabel('t [sec]')
legend
ax = gca();
ax.FontSize = 14;
f2 = gcf();

% Arbitrary Axis Stability

Ir = 1;
r = A_ptob.' * [0 1 0].';
om0 = A_ptob.' * deg2rad([0 10 0]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';
u = squeeze(simOut.u);

figure(fomega.Number)
aplot = plot(t, om_p, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ylabel('\omega [rad/s]')
hold on
xlabel('t [sec]')
legend
ax = gca();
ax.FontSize = 14;
f1 = gcf();


figure(fangles.Number)
aplot = plot(t, u, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\phi';'\theta'; '\psi'})
ylabel('u [rad]')
hold on
xlabel('t [sec]')
legend
ax = gca();
ax.FontSize = 14;
f2 = gcf();


%% Functions

function u = RtoEuler313(R)

    u = zeros([3 1]);

    u(1) = atan2(R(1,3), R(2,3));
    u(2) = acos(R(3,3));
    u(3) = atan2(R(3,1), -R(3,2));

end

function R_eci_to_rtn = eci2rtn(r_eci, v_eci)
    
    % Compute radial, transverse, and normal vectors in ECI frame
    r_radial_eci = r_eci;
    r_normal_eci = cross(r_radial_eci, v_eci);
    r_transverse_eci = -cross(r_radial_eci, r_normal_eci);
    
    % Normalize radial, transverse, and normal vectors to obtain unit vectors
    r_radial_eci_unit = r_radial_eci / norm(r_radial_eci);
    r_transverse_eci_unit = r_transverse_eci / norm(r_transverse_eci);
    r_normal_eci_unit = r_normal_eci / norm(r_normal_eci);
    
    % Construct rotation matrix from ECI to RTN
    R_eci_to_rtn = [r_radial_eci_unit; r_transverse_eci_unit; r_normal_eci_unit];

end