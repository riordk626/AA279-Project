%% Initialization
clc, clear
close all

projectStartup;
exportflag = true;
figurePath = '../../Images/PS6/';

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();
[areas, centroids, normalVectors] = aquaSurfaceProps();

load('orbitConstants.mat')

[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);

orbitStruct.orbitType = "num";
orbitStruct.dataSource = 'MAT-File';
% orbitStruct.dataSource = 'MATLAB File';
distStruct.dataSource = 'MAT-File';

plantStruct.I_sim = Itotal_p;
plantStruct.axesFlag = 0;
plantStruct.dynamicsType = "default";
plantStruct.attitudeType = "euler";
plantStruct.sequence = "313";

plantStruct.normalVectors = normalVectors;
plantStruct.areas = areas;
plantStruct.rcm = rcm;
plantStruct.centroids = centroids;

sensorStruct.measProcess = "default";
sensorStruct.attitudeNoiseFactor = 0;
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.starCatalog = "simple";
sensorStruct.attitudeFileName = "starTrackerSimpleUndersampled.mat";

ICstruct.r0 = r0; ICstruct.v0 = v0;

%% Problem 1

omx = 0;
omy = 0;
omz = 0;
% % 
om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;
% 
ICstruct.om0 = om0; ICstruct.R0 = R0;
Tfinal = T * 5;

% plots all torques

distStruct.disturbance = "all";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct);
simOut = sim(simIn);

t = simOut.t;
M_all = squeeze(simOut.M_all);

figure()
hold on
p = plot(t, M_all, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'all_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Total Torque')

% plot solar torque
distStruct.disturbance = "solar";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_sol = squeeze(simOut.M_sol);
r_mag = vecnorm(r,2,1);

cg2cp = 7.6578;
q = 0.66;
meanSolarFlux = 4.4e-6;
A = 224.68;

M_sol_bound = meanSolarFlux*A*(1+q)*cg2cp * ones(size(t));

figure()
plot(t, M_sol_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_sol_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_sol, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'solar_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Solar Torque')

% plots magnetic torque

distStruct.disturbance = "mag";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_mag = squeeze(simOut.M_mag);
r_mag = vecnorm(r,2,1);
mag_data = load('magConstants.mat', 'm_sat', 'Re', 'B0');
m_sat = mag_data.m_sat;
Re = mag_data.Re;
B0 = mag_data.B0;
m_sat_mag = norm(m_sat);

M_mag_bound = 2*m_sat_mag*Re^3*B0./(r_mag.^3);

figure()
plot(t, M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_mag, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'magnetic_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Magnetic Torque')

% plots aero torque

distStruct.disturbance = "aero";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct);
simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_aero = squeeze(simOut.M_aero);
r_mag = vecnorm(r,2,1);

cg2cp = 7.6578;
cd = 2;
A = 224.68;
rho = 1e-15;
vMag = norm(simOut.v.Data(:, :, 1));

M_aero_bound = 0.5*rho*vMag^2*A*cd*cg2cp * ones(size(t));


figure()
plot(t, M_aero_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_aero_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_aero, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'aero_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Aero Torque')

%% Problem 2 & 3

ombx = 0;
omby = 0;
ombz = 0;

om0 = A_ptob.' * [ombx omby ombz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;

ICstruct.om0 = om0; ICstruct.R0 = R0;

% distStruct.disturbance = "grav";
distStruct.disturbance = "all";

Tfinal = 5*T;

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
R_ECItoRTN = simOut.rtn.Data; % ORBIT DCM OUTPUT
R_error = simOut.R_error.Data;
% errorSeq = "313";
errorSeq = "312";
nsteps = length(t);
u_error = zeros([3 nsteps]);
for i=1:nsteps
    u_error(:,i) = RtoEuler(R_error(:,:,i), errorSeq);
end
values = {u_error};
valueNames = {'u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'}};
figureName = [figurePath, 'attitude_error_no_dist.png'];

fig = figure();
timeHistoryPlot(fig, t,values,valueNames,valueLabels,figureName,exportflag)

distStruct.disturbance = "grav";
% distStruct.disturbance = "all";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
R_ECItoRTN = simOut.rtn.Data; % ORBIT DCM OUTPUT
R_error = simOut.R_error.Data;
% errorSeq = "313";
errorSeq = "312";
nsteps = length(t);
u_error = zeros([3 nsteps]);
for i=1:nsteps
    u_error(:,i) = RtoEuler(R_error(:,:,i), errorSeq);
end
values = {u_error};
valueNames = {'u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'}};
figureName = [figurePath, 'attitude_error_dist.png'];

fig = figure();
timeHistoryPlot(fig, t,values,valueNames,valueLabels,figureName,exportflag)

%% Problem 4, 5, & 6

% plot euler angles from obc for omega measurements and ground truth
obc = squeeze(simOut.alphaMeasured.Data);
groundTruth = squeeze(simOut.alpha.Data);
t = simOut.t;

figure()
hold on
p = plot(t, obc, 'LineWidth', 2);
p2 = plot(t, groundTruth, 'LineWidth', 2);
set(p, {'DisplayName'}, {'\phi_{obc}'; '\theta_{obc}'; '\psi_{obc}'})
set(p2, {'DisplayName'}, {'\phi'; '\theta'; '\psi'})
xlabel('t [sec]')
ylabel('Euler Angles [rad]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'obcVsGroundOmegas.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'OBC vs Ground Truth for Measured Euler Angles')


Tfinal = 5*T;

omx = deg2rad(0.03);
omy = deg2rad(-0.02);
omz = deg2rad(0.05);
% % 
om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;
% 
ICstruct.om0 = om0; ICstruct.R0 = R0;

% plot solar torque
distStruct.disturbance = "none";
% distStruct.disturbance = "all":

%% Undersampled Hacked
exportflag = false;

sensorStruct.measProcess = "default";
sensorStruct.attitudeNoiseFactor = 0;
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.starCatalog = "simple";
sensorStruct.attitudeFileName = "starTrackerSimpleUndersampled.mat";


simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
u = wrapToPi(squeeze(simOut.alpha.Data));
R_est = simOut.R_est.Data;
nsteps = length(t);
u_est = zeros([3 nsteps]);
for i=1:nsteps
    u_est(:,i) = RtoEuler(R_est(:,:,i), plantStruct.sequence);
end

u_est_error = u - u_est;
values = {u, u_est, u_est_error};
valueNames = {'u [rad]';'u_{est} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'attitude_estimation_undersampled_det_default.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

sensorStruct.measProcess = "fictitious";
sensorStruct.attitudeSensorSolver = "deterministic";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
u = wrapToPi(squeeze(simOut.alpha.Data));
R_est = simOut.R_est.Data;
nsteps = length(t);
u_est = zeros([3 nsteps]);
for i=1:nsteps
    u_est(:,i) = RtoEuler(R_est(:,:,i), plantStruct.sequence);
end

u_est_error = u - u_est;
values = {u, u_est, u_est_error};
valueNames = {'u [rad]';'u_{est} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'attitude_estimation_undersampled_det_fictitious.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

sensorStruct.measProcess = "default";
sensorStruct.attitudeSensorSolver = "qmethod";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
u = wrapToPi(squeeze(simOut.alpha.Data));
R_est = simOut.R_est.Data;
nsteps = length(t);
u_est = zeros([3 nsteps]);
for i=1:nsteps
    u_est(:,i) = RtoEuler(R_est(:,:,i), plantStruct.sequence);
end

u_est_error = u - u_est;
values = {u, u_est, u_est_error};
valueNames = {'u [rad]';'u_{est} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'attitude_estimation_undersampled_q_default.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)


%% Oversampled Hack
exportflag = false;

sensorStruct.measProcess = "default";
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.attitudeFileName = "starTrackerSimpleOversampled.mat";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
u = wrapToPi(squeeze(simOut.alpha.Data));
R_est = simOut.R_est.Data;
nsteps = length(t);
u_est = zeros([3 nsteps]);
for i=1:nsteps
    u_est(:,i) = RtoEuler(R_est(:,:,i), plantStruct.sequence);
end

u_est_error = u - u_est;
values = {u, u_est, u_est_error};
valueNames = {'u [rad]';'u_{est} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'attitude_estimation_oversampled_det_default.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

sensorStruct.measProcess = "default";
sensorStruct.attitudeSensorSolver = "qmethod";
sensorStruct.attitudeFileName = "starTrackerSimpleOversampled.mat";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct, sensorStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
u = wrapToPi(squeeze(simOut.alpha.Data));
R_est = simOut.R_est.Data;
nsteps = length(t);
u_est = zeros([3 nsteps]);
for i=1:nsteps
    u_est(:,i) = RtoEuler(R_est(:,:,i), plantStruct.sequence);
end

u_est_error = u - u_est;
values = {u, u_est, u_est_error};
valueNames = {'u [rad]';'u_{est} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'attitude_estimation_oversampled_q_default.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)