%% Load Model
clc, clear
close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

I_sim = Itotal_p;
Tfinal = 300;
axesFlag = 0;
dynamicsType="default";
attitudeType="euler";
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput('aquaMasterModel');
simIn.ExternalInput = M;

%% Problem 1 - Equilibrium Analysis

% Part a - Inertial Alignment 
u0 = [0,0.000001,0].';
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
u0 = [0,0.000001,0].';
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
u0 = [0,0.000001,0].';
u0 = u0 + 0.01.*rand(size(u0));

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
r = [0 0 1].';

% Momentum Wheel Equilibrium Analysis

% Part a - Inertial Alignment 
u0 = [0,0.000001,0].';
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
u0 = [0,0.000001,0].';
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
u0 = [0,0.000001,0].';
u0 = u0 + 0.01.*rand(size(u0));

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

exportgraphics(fomega, '../Images/PS4/mom_wheel_stability_history_velocity.png')
exportgraphics(fangles, '../Images/PS4/mom_wheel_stability_history_angles.png')

% Intermediate Stability

% Arbitrary Axis Stability