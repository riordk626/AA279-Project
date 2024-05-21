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
kalmanFilterStruct.dt_KF = 1e-3;

ICstruct.r0 = r0; ICstruct.v0 = v0;

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
dt = 1e-3;

% plots all torques

distStruct.disturbance = "all";

%% Problem 5 & 6

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct,sensorStruct,kalmanFilterStruct);
simOut = sim(simIn);