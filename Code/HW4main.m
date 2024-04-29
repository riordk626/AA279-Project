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

% Orbital Elements
e_float = 0.0000979; % eccentricity 
a_float = 7080.6; % km
i_float = 98.2; %degrees
omega_float = 120.4799; % arguement of perigee // degrees
Omega_float = 95.2063; % ascending node // degrees
nu_float = 0; % True Anomaly (in radians)
mu_float = 3.986004418e5; % Gravitational parameter of the Earth in km^3/s^2

semimajorAxis = [[0, a_float]; [10000000, a_float]];
eccentricity = [[0, e_float]; [10000000, e_float]];
inclination = [[0, i_float]; [10000000, i_float]];
omega = [[0, omega_float]; [10000000, omega_float]];
Omega = [[0, Omega_float]; [10000000, Omega_float]];
trueAnomaly = [[0, nu_float]; [10000000, nu_float]];
mu = [[0, mu_float]; [10000000, mu_float]];

orbitType = "num";

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
[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);
R0 = eci2rtn(r0, v0);
u0 = RtoEuler313(R0);
attitudeType="euler";

om0 = deg2rad([0 0 10]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
R_ECItoRTN = simOut.rtn.Data; % ORBIT DCM OUTPUT
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';

u = zeros([3 size(t,1)]);
for i=1:size(u,2)
    R = R_ItoP(:,:,i) * R_ECItoRTN(:,:,i).';
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

%% Testing simulink orbit propagator -- may remove later 

Tfinal = 300 * 20 * 3;
load_system("aquaMasterModel")
simOut = sim(simIn);

r = simOut.r.Data;
rE = 6378.1; % km

% Plot orbit in 3D
plot_orbit(r, rE);

Tfinal = 300;

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

%% Problem 3 - Momentum Wheel

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

[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);
R0 = eci2rtn(r0, v0);
u0 = RtoEuler313(R0);
attitudeType="euler";
om0 = deg2rad([0 0 10]).';

load_system("aquaMasterModel")

simOut = sim(simIn);

R_ItoP = simOut.yout{1}.Values.Data;
R_ECItoRTN = simOut.rtn.Data; % ORBIT DCM OUTPUT
t = simOut.t;
n = size(t,1);
om_p = squeeze(simOut.om_p).';

u = zeros([3 size(t,1)]);
for i=1:size(u,2)
    R = R_ItoP(:,:,i) * R_ECItoRTN(:,:,i).';
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

fomega = figure()
fangles = figure()

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
    R_eci_to_rtn = [r_radial_eci_unit, r_transverse_eci_unit, r_normal_eci_unit];

end

function [rECI, vECI] = keplerian2ECI(a, e, i, Omega, omega, nu, mu)

    % Calculate position and velocity in the perifocal coordinate system
    E = acos((e + cos(nu))/(1+(e*cos(nu))));
    n = sqrt(mu/a^3);
    r_perifocal = [a*(cos(E) - e); a*sin(E)*sqrt(1-e^2); 0];
    v_perifocal = a*n/(1-e*cos(E)) * [-sin(E); cos(E)*sqrt(1-e^2); 0];

    % Rotation matrices
    R3_Omega = [cos(-Omega) sin(-Omega) 0;
                -sin(-Omega) cos(-Omega) 0;
                0 0 1];
    R1_i = [1 0 0;
            0 cos(-i) sin(-i);
            0 -sin(-i) cos(-i)];
    R3_omega = [cos(-omega) sin(-omega) 0;
                -sin(-omega) cos(-omega) 0;
                0 0 1];

    % Transformation to ECI
    Q_perifocal2ECI = R3_Omega * R1_i * R3_omega;
    rECI = Q_perifocal2ECI * r_perifocal;
    vECI = Q_perifocal2ECI * v_perifocal;
end

function plot_orbit(r, rE)
    % Plot orbit in 3D
    figure;
    plot3(r(:,1), r(:,2), r(:,3), 'r', 'LineWidth', 2.5);
    hold on;
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    title('Aqua Orbit');
    grid on;

    [xE, yE, zE] = ellipsoid(0,0,0,rE, rE, rE, 20);
    surface(xE, yE, zE, 'FaceColor', 'blue', 'EdgeColor', 'black')
    
    axis equal;
    view(3);
end