%% Initialization
clc, clear
close all

projectStartup;
exportflag = false;
figurePath = '../../Images/PS9/';

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
plantStruct.sequence = "312";

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
Rstar = 2.35e-11*eye(3*(nmeas-1));
Ratt = [Rmag, zeros([3, 3*(nmeas-1)]); zeros([3*(nmeas-1), 3]), Rstar];
Rom = 5.97e-8*eye(3);
kalmanFilterStruct.R = [Ratt, zeros([3*nmeas, 3]); zeros([3 3*nmeas]), Rom];
kalmanFilterStruct.P0 = (1e-3).*eye(6);
kalmanFilterStruct.Q = (10e-2).*kalmanFilterStruct.P0;
kalmanFilterStruct.dt_KF = 1;

controlLawStruct.controlType = "dummy";
controlLawStruct.errorType = "small";
controlLawStruct.controllerParams = struct([]);
controlLawStruct.dt_cont = 1e-1;

ICstruct.r0 = r0; ICstruct.v0 = v0;

Tfinal = 3*T;
dt_sc = 1e-1;

% plots all torques

distStruct.disturbance = "all";

timeUpdateTest = false;

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

%% Problem 2


% initialize for reaction wheel test
Lw0 = 12;
A = (1/sqrt(3)).*[-1, 1, 1, -1;
                  -1, -1, 1, 1;
                  1, 1, 1, 1];
actuatorModelStruct.controlMoment = "reactionWheel";
actuatorModelStruct.actuatorParams.Lw0 = Lw0.*ones([4 1]);
actuatorModelStruct.actuatorParams.A = A;

simIn = initAqua(Tfinal, R_om_des, ICstruct, orbitStruct, plantStruct,...
    distStruct,sensorStruct,kalmanFilterStruct,controlLawStruct, actuatorModelStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
Mc = squeeze(simOut.Mc.Data);
Mout = squeeze(simOut.Mout.Data);
Lwdot = squeeze(simOut.Lwdot.Data);
values = {Mc;Mout;Lwdot};
valueNames = {'$M_c$ [Nm]';'$M_{out}$ [Nm]';'$\dot{L}_w$ [Nm]'};
valueLabels = {{'$M_x$';'$M_y$';'$M_z$'}, {'$M_x$';'$M_y$';'$M_z$'}, {'$\dot{L}_1$'...
    ;'$\dot{L}_2$';'$\dot{L}_3$';'$\dot{L}_4$'}};
figureName = [figurePath, 'reaction_wheel_model_output.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, exportflag)

% initialize for magnetorquer test
actuatorModelStruct.controlMoment = "magnetorquer";

simIn = initAqua(Tfinal, R_om_des, ICstruct, orbitStruct, plantStruct,...
    distStruct,sensorStruct,kalmanFilterStruct,controlLawStruct, actuatorModelStruct);

simOut = sim(simIn);

t = simOut.t;
R_ItoP = simOut.yout{1}.Values.Data;
Mc = squeeze(simOut.Mc.Data);
Mout = squeeze(simOut.Mout.Data);
m = squeeze(simOut.m.Data);
Ldot = simOut.Ldot.Data;
values = {Mc;Mout;m(1:2,:);Ldot};
valueNames = {'$M_c$ [Nm]';'$M_{out}$ [Nm]';'$m$';'$\dot{L}_w$'};
valueLabels = {{'$M_x$';'$M_y$';'$M_z$'}, {'$M_x$';'$M_y$';'$M_z$'}, {'$m_x$'...
    ;'$m_y$'},{'$\dot{L}_z$'}};
figureName = [figurePath, 'simple_magnetorquer_plus_wheel_model_output.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, false)

fig;
subplot(4,1,3)
ylim([-.1 .1])
subplot(4,1,4)
ylim([-.1 .1])

if exportflag
    exportgraphics(fig, figureName)
end

%% Problem 3

% error testing - not a part of the pset; just for validation
distStruct.disturbance = "all";

clear actuatorModelStruct
actuatorModelStruct.controlMoment = "ideal";
actuatorModelStruct.actuatorParams = struct([]);

clear controlLawStruct
controlLawStruct.controlType = "PD";
controlLawStruct.errorType = "small";
kp = diag([0, 0, 0]);
kd = diag([0, 0, 0]);
controlLawStruct.controllerParams.kp = kp;
controlLawStruct.controllerParams.kd = kd;
controlLawStruct.dt_cont = 1e-1;

om0 = om_des + [0.001 -0.0005 0.0002].';
ICstruct.om0 = om0;

simIn = initAqua(Tfinal, R_om_des, ICstruct, orbitStruct, plantStruct,...
    distStruct,sensorStruct,kalmanFilterStruct,controlLawStruct, actuatorModelStruct);

simOut = sim(simIn);

t = simOut.t;
n = length(t);
a_error = squeeze(simOut.a_error.Data);
valueLabels = {{'$\alpha_x$';'$\alpha_y$';'$\alpha_z$'}};
values = {a_error};
valueNames = {'$\alpha$'};
figureName = [];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, false)

% gain design and tuning + testing
clear controlLawStruct
controlLawStruct.controlType = "PD";
controlLawStruct.errorType = "small";
wn = [30*n_float, 30*n_float, 30*n_float].';
zeta = [0.5, 0.5, 0.5].';
Ixyz = diag(Itotal_p);
kp = Ixyz.*(wn.^2);
kd = 2.*zeta.*sqrt(Ixyz.*kp);
controlLawStruct.controllerParams.kp = diag(kp);
controlLawStruct.controllerParams.kd = diag(kd);
controlLawStruct.dt_cont = 1e-1;

simIn = initAqua(Tfinal, R_om_des, ICstruct, orbitStruct, plantStruct,...
    distStruct,sensorStruct,kalmanFilterStruct,controlLawStruct, actuatorModelStruct);

simOut = sim(simIn);

t = simOut.t;
n = length(t);
a_error = squeeze(simOut.a_error.Data);
valueLabels = {{'$\alpha_x$';'$\alpha_y$';'$\alpha_z$'}};
values = {a_error};
valueNames = {'$\alpha$'};
figureName = [figurePath, 'alpha_history_PD_control.png'];

fig = figure();
timeHistoryPlot(fig, t, values, valueNames, valueLabels, figureName, true, exportflag)