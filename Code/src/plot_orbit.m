%% Load Model
clc, clear
close all

projectStartup;

% Orbital Elements
e_float = 0.0000979; % eccentricity 
a_float = 7080.6; % km
i_float = deg2rad(98.2); %degrees
omega_float = deg2rad(120.4799); % arguement of perigee // degrees
Omega_float = deg2rad(95.2063); % ascending node // degrees
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
initOrbital(orbitType, "MAT-File");

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

I_sim = Itotal_p;
axesFlag = 0;
dynamicsType = "default";
disturbance = "none";
sequence = "312";
magnetic= "dipole";

Tfinal = 86400;
simIn = Simulink.SimulationInput('aquaMasterModel');
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn.ExternalInput = M;
load_system("aquaMasterModel")
simOut = sim(simIn);

r = simOut.r.Data;
rE = 6378.1; % km

% Plot orbit in 3D
plotOrbit(r, rE);

lat = simOut.latitude.Data;
lon = simOut.longitude.Data;
figure;
hold on;
worldmap('World');
load coastlines; % Load coastline data for plotting
plotm(coastlat, coastlon, 'k'); % Plot coastlines
plotm(rad2deg(lat), rad2deg(lon), 'r.-'); % Plot groundtrack
title('Satellite Groundtrack');

function plotOrbit(r, rE)
    % Plot orbit in 3D
    figure;
    plot3(r(1, :), r(2, :), r(3, :), 'r', 'LineWidth', 2.5);
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