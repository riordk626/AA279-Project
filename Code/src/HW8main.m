%% Initialization
clc, clear
close all

projectStartup;
exportflag = true;
figurePath = '../../Images/PS8/';

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
sensorStruct.attitudeFileName = "attitudeMeasData.mat";

nmeas = 11;
kalmanFilterStruct.R = eye(3*nmeas + 3);
kalmanFilterStruct.P0 = (1e-3).*eye(6);
kalmanFilterStruct.Q = (1e-2).*kalmanFilterStruct.P0;
kalmanFilterStruct.dt_KF = 1;

ICstruct.r0 = r0; ICstruct.v0 = v0;

Tfinal = 3*T;
dt_sc = 1e-1;

% plots all torques

distStruct.disturbance = "all";

%% Problem 7

% Plot true attitude estimation errors
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
% distStruct.disturbance = "none";
distStruct.disturbance = "all";

sensorStruct.measProcess = "default";
sensorStruct.attitudeNoiseFactor = 0;
sensorStruct.attitudeSensorSolver = "deterministic";
sensorStruct.starCatalog = "simple";
sensorStruct.attitudeFileName = "attitudeMeasData.mat";


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
exportgraphics(gcf, figureName)
saveas(gcf, figureName)

% plot covariance from filter


% pre- and post-fit residuals (with statistics at steady state)

