%% Load Model
% clc, clear
% close all
% 
% projectStartup;
% 
% orbitType = "num";
% initOrbital(orbitType, "MAT-File");
% 
% [rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();
% 
% I_sim = Itotal_p;
% axesFlag = 0;
% dynamicsType = "default";
% disturbance = "none";
% sequence = "312";
% magnetic= "dipole";
% 
% Tfinal = 86400;
% simIn = Simulink.SimulationInput('aquaMasterModel');
% M = timeseries(zeros([3 2]), [0 Tfinal]);
% simIn.ExternalInput = M;
% load_system("aquaMasterModel")
% simOut = sim(simIn);

r = simOut.r.Data;
r_sun = simOut.r_sun.Data;
rE = 6378.1; % km

% Plot orbit in 3D
plotOrbit(r, rE);
plotOrbit(r_sun, rE);

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
    view(2);
end