%% Initialization
clc, clear
close all

projectStartup;

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

load('orbitConstants.mat')

[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);

orbitStruct.orbitType = "num";
orbitStruct.dataSource = 'MAT-File';
% orbitStruct.dataSource = 'MATLAB File';

plantStruct.I_sim = Itotal_p;
plantStruct.axesFlag = 0;
plantStruct.dynamicsType = "default";
plantStruct.attitudeType = "euler";
plantStruct.sequence = "313";

ICstruct.r0 = r0; ICstruct.v0 = v0;

%% Problem 1

omz = 0;
omx = -n_float;
omy = 0;

om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoP = [0 0 -1;0 1 0;1 0 0];
R0 = R_RTNtoP * R_ECItoRTN;

ICstruct.om0 = om0; ICstruct.R0 = R0;

distStruct.disturbance = "grav";

Tfinal = 3*T;

simIn = initAqua(Tfinal, ICstruct, orbitStruct, plantStruct, distStruct);

simOut = sim(simIn);

t = simOut.t;
R = simOut.yout{1}.Values.Data;
M_grav = squeeze(simOut.M_grav);
om_p = squeeze(simOut.om_p);