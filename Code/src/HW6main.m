%% Initialization
clc, clear
close all

projectStartup;
exportflag = false;
figurePath = '../../Images/PS5/';

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

ICstruct.r0 = r0; ICstruct.v0 = v0;


%% Problem 2 & 3

omx = 0;
omy = 0;
omz = n_float;

R0 = A_ptob.' * [0 1 0;0 0 1;1 0 0] * eci2rtn(r0, v0);

om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;

ICstruct.om0 = om0; ICstruct.R0 = R0;

distStruct.disturbance = "grav";
% distStruct.disturbance = "all":

Tfinal = 5*T;
M = timeseries(zeros([3 2]), [0 Tfinal]);
extInputStruct.M = M;

R_RTNtoPdes_ts = R_RTNtoPdes;
R_RTNtoPdes_ts(:,:,2) = R_RTNtoPdes;
extInputStruct.R_RTNtoPdes = timeseries(R_RTNtoPdes_ts, [0 Tfinal]);
% extInputStruct.R_RTNtoPdes
simIn = initAqua(Tfinal, extInputStruct, ICstruct, orbitStruct, plantStruct, distStruct);

simOut = sim(simIn);

t = simOut.t;
% R_ECItoPdes = ;
R_ItoP = simOut.yout{1}.Values.Data;
R_ECItoRTN = simOut.rtn.Data; % ORBIT DCM OUTPUT
% R_error = ;
errorSeq = "313";
% errorSeq = "312";
u_error = RtoEuler(R_error, errorSeq);
values = {u_error};
valueNames = {'u [rad]'};
valueLabels = {{'\phi'; '\theta'; '\psi'}};
figureName = [figurePath, 'attitude_error.png'];

fig = figure();
timeHistoryPlot(fig, t,values,valueNames,valueLabels,figureName,exportflag)