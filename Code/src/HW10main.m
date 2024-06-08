%% Initialization
clc, clear
close all

projectStartup;
exportflag = false;
figurePath = '../../Images/PS10/';

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
% plantStruct.attitudeType = "euler";
% plantStruct.sequence = "312";
plantStruct.attitudeType = "quat";
plantStruct.sequence = struct([]);

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
Rmag = 4e-10*eye(3);
% Rmag = 500.*eye(3);
Rstar = 2.35e-11*eye(3*(nmeas-1));
% Rstar = 500.*eye(3*(nmeas - 1));
Ratt = [Rmag, zeros([3, 3*(nmeas-1)]); zeros([3*(nmeas-1), 3]), Rstar];
Rom = 5.97e-8*eye(3);
kalmanFilterStruct.R = [Ratt, zeros([3*nmeas, 3]); zeros([3 3*nmeas]), Rom];
kalmanFilterStruct.P0 = (1e-3).*eye(6);
kalmanFilterStruct.Q = (10e-2).*kalmanFilterStruct.P0;
kalmanFilterStruct.dt_KF = 1e-1;

% gain design and tuning + testing
controlLawStruct.controlType = "PD";
controlLawStruct.errorType = "small";
wn = 0.5.*[50*n_float, 50*n_float, 50*n_float].';
zeta = [sqrt(2)/2, sqrt(2)/2, sqrt(2)/2].';
Ixyz = diag(Itotal_p);
kp = Ixyz.*(wn.^2);
kd = 2.*zeta.*sqrt(Ixyz.*kp);
controlLawStruct.controllerParams.kp = diag(kp);
controlLawStruct.controllerParams.kd = diag(kd);
controlLawStruct.dt_cont = 1e-1;

% initialize for reaction wheel test
Lw0 = 12;
A = (1/sqrt(3)).*[-1, 1, 1, -1;
                  -1, -1, 1, 1;
                  1, 1, 1, 1];
actuatorModelStruct.controlMoment = "reactionWheel";
actuatorModelStruct.actuatorParams.Lw0 = Lw0.*ones([4 1]);
actuatorModelStruct.actuatorParams.A = A;
actuatorModelStruct.actuatorParams.Lwdot_max = inf;
actuatorModelStruct.actuatorParams.Lw_max = inf;

ICstruct.r0 = r0; ICstruct.v0 = v0;

Tfinal = 3*T;
dt_sc = 1e-1;

omx_des = -n_float;
omy_des = 0;
omz_des = 0;
% % 
om_des = [omx_des omy_des omz_des].';
om0 = om_des;
R_ECItoRTN = eci2rtn(r0, v0);
% R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = [0, 0, -1;0, 1, 0;1, 0, 0];
R_des = R_RTNtoPdes;
R0 = R_RTNtoPdes * R_ECItoRTN;
ICstruct.om0 = om0; ICstruct.R0 = R0;

R_om_des.R_des = R_des;
R_om_des.om_des = om_des;

%% Problem 1

% plots all torques
distStruct.disturbance = "all";
timeUpdateTest = false;

% distStruct.disturbance = "none";
% timeUpdateTest = true;

simIn = initAqua(Tfinal, R_om_des, ICstruct, orbitStruct, plantStruct,...
    distStruct,sensorStruct,kalmanFilterStruct,controlLawStruct, actuatorModelStruct);

simOut = sim(simIn);

t = simOut.t;
nsteps = length(t);
R_ItoP = simOut.yout{1}.Values.Data;
% u = wrapToPi(squeeze(simOut.alpha.Data));
u = zeros([3 nsteps]);
q = squeeze(simOut.alpha.Data);
om = squeeze(simOut.om_p.Data);
q_kf = squeeze(simOut.q_kf.Data);
x_kf = squeeze(simOut.x_kf.Data);
P_kf = simOut.P_kf.Data;
om_kf = x_kf(4:6, :);
u_kf = zeros([3 nsteps]);
u_kf_error = zeros(size(u_kf));
sigKF = zeros([6 nsteps]);
sigTrue = zeros([6 nsteps]);
for i=1:nsteps
    u(:,i) = RtoEuler(q2R(q(:,i)), "312");
    Rint = q2R(q_kf(:,i));
    u_kf(:,i) = RtoEuler(Rint, "312");
    R_error = Rint * R_ItoP(:,:,i).';
    u_kf_error(:,i) = RtoEuler(R_error, "312");
    sigKF(:,i) = sqrt(diag(P_kf(:,:,i)));
    sigTrue(1:3,i) = std(u(:,i) - u_kf(:,i));
    sigTrue(4:6,i) = std(om(:,i) - om_kf(:,i));
end

u_kf_error = u - u_kf;
values = {u, u_kf, u_kf_error};
valueNames = {'$u$ [rad]';'$u_{kf}$ [rad]'; '$\Delta u$ [rad]'};
valueLabels = {{'$\phi$'; '$\theta$'; '$\psi$'};{'$\phi$'; '$\theta$'; '$\psi$'};...
    {'$\Delta \phi$'; '$\Delta \theta$'; '$\Delta \psi$'}};
figureName = [figurePath, 'kalman_filter_meas_update_error_attitude.png'];

% values = {q, q_kf};
% valueNames = {'$q$';'$q_{kf}$'};
% valueLabels = {{'$q_0$'; '$q_1$'; '$q_2$'; '$q_3$'};{'$q_0$'; '$q_1$'; '$q_2$'; '$q_3$'}};
% figureName = [figurePath, 'kalman_filter_meas_update_error_attitude.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, exportflag)

om_kf_error = om - om_kf;
values = {om, om_kf, om_kf_error};
valueNames = {'$\omega$ [rad/s]';'$\omega_{kf}$ [rad/s]'; '$\Delta \omega$ [rad/s]'};
valueLabels = {{'$\omega_1$'; '$\omega_2$'; '$\omega_3$'};{'$\omega_1$'; '$\omega_2$'; '$\omega_3$'};...
    {'$\Delta \omega_1$'; '$\Delta \omega_2$'; '$\Delta \omega_3$'}};
figureName = [figurePath, 'kalman_filter_meas_update_error_velocities.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, exportflag)

a_error = squeeze(simOut.a_error.Data);
valueLabels = {{'$\alpha_x$';'$\alpha_y$';'$\alpha_z$'}};
values = {a_error};
valueNames = {'$\alpha$'};
figureName = [figurePath, 'alpha_history_PD_control.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, exportflag)

z_prefit = vecnorm(squeeze(simOut.z_prefit.Data), 2, 1);
z_postfit = vecnorm(squeeze(simOut.z_postfit.Data), 2, 1);
z_error = z_prefit - z_postfit;

values = {z_prefit, z_postfit, z_error};
valueNames = {'$z_{pre}$';'$z_{post}$'; '$\Delta z$'};
figureName = [figurePath, 'kalman_filter_meas_update_residual_comparison.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, {}, figureName, false, exportflag)