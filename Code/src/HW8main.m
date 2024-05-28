%% Initialization
clc, clear
close all

projectStartup;
exportflag = false;
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
kalmanFilterStruct.R = 2.*eye(3*nmeas + 3);
kalmanFilterStruct.P0 = (1e-3).*eye(6);
kalmanFilterStruct.Q = (1e-2).*kalmanFilterStruct.P0;
kalmanFilterStruct.dt_KF = 1;

ICstruct.r0 = r0; ICstruct.v0 = v0;

Tfinal = 3*T;
dt_sc = 1e-1;

% plots all torques

distStruct.disturbance = "none";

timeUpdateTest = false;

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
om = squeeze(simOut.om_p.Data);
q_kf = squeeze(simOut.q_kf.Data);
x_kf = squeeze(simOut.x_kf.Data);
P_kf = simOut.P_kf.Data;
om_kf = x_kf(4:6, :);
nsteps = length(t);
u_kf = zeros([3 nsteps]);
sigKF = zeros([6 nsteps]);
sigTrue = zeros([6 nsteps]);
for i=1:nsteps
    Rint = q2R(q_kf(:,i));
    u_kf(:,i) = RtoEuler(Rint, plantStruct.sequence);

    sigKF(:,i) = sqrt(diag(P_kf(:,:,i)));
    sigTrue(1:3,i) = std(u(:,i) - u_kf(:,i));
    sigTrue(4:6,i) = std(om(:,i) - om_kf(:,i));
end

u_kf_error = u - u_kf;
values = {u, u_kf, u_kf_error};
valueNames = {'u [rad]';'u_{kf} [rad]'; '\Delta u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'};{'\phi'; '\theta'; '\psi'};...
    {'\Delta \phi'; '\Delta \theta'; '\Delta \psi'}};
figureName = [figurePath, 'kalman_filter_time_update_error_attitude.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

om_kf_error = om - om_kf;
values = {om, om_kf, om_kf_error};
valueNames = {'\omega [rad/s]';'\omega_{kf} [rad/s]'; '\Delta \omega [rad/s]'};
valueLabels = {{'\omega_1'; '\omega_2'; '\omega_3'};{'\omega_1'; '\omega_2'; '\omega_3'};...
    {'\Delta \omega_1'; '\Delta \omega_2'; '\Delta \omega_3'}};
figureName = [figurePath, 'kalman_filter_time_update_error_velocities.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, exportflag)

% plot covariance from filter


% pre- and post-fit residuals (with statistics at steady state)

