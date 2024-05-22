%% Initialization
clc, clear
close all

projectStartup;
exportflag = true;
figurePath = '../../Images/PS7/';

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

nmeas = 11;
kalmanFilterStruct.Q = eye(6);
kalmanFilterStruct.R = eye(3*nmeas + 3);
kalmanFilterStruct.P0 = eye(6);
kalmanFilterStruct.dt_KF = 1e-1;

ICstruct.r0 = r0; ICstruct.v0 = v0;

omx = n_float;
omy = n_float;
omz = n_float;
% % 
om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;
% 
ICstruct.om0 = om0; ICstruct.R0 = R0;
Tfinal = 3*T;
dt_sc = 1e-1;

% plots all torques

distStruct.disturbance = "all";

%% Problem 2

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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

% Undersampled Hacked
exportflag = false;

sensorStruct.measProcess = "default";
sensorStruct.attitudeNoiseFactor = 0;
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.starCatalog = "simple";
sensorStruct.attitudeFileName = "starTrackerSimpleUndersampled.mat";


simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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


% Oversampled Hack
exportflag = false;

sensorStruct.measProcess = "default";
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.attitudeFileName = "starTrackerSimpleOversampled.mat";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);

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

%% Problem 5 & 6

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);
simOut = sim(simIn);