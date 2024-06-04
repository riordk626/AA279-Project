%% Initialization
clc, clear
close all

projectStartup;
exportflag = true;
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



ICstruct.r0 = r0; ICstruct.v0 = v0;

Tfinal = 3*T;
dt_sc = 1e-1;

% plots all torques

distStruct.disturbance = "all";

timeUpdateTest = false;
