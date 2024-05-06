%% Initialization
clc, clear
close all

projectStartup;

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);

Tfinal = 300;

%% Problem 1

cb = [0 0 1].';
cp = A_ptob.' * cb;
cx = cp(1);
cy = cp(2);
cz = cp(3);

omz = sqrt(3*n_float^2 * cz);
omx = 3*n_float^2 * cz * cx/omz;
omy = 3*n_float^2 * cy * cz/omz;

om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoP = A_ptob.' * [0 1 0;0 0 1;1 0 0];
R0 = R_RTNtoP * R_ECItoRTN;

ICstruct.r0 = r0; ICstruct.v0 = v0; ICstruct.om0 = om0; ICstruct.R0 = R0;
orbitStruct.orbitType = "num";

plantStruct.I_sim = Itotal_p;
plantStruct.axesFlag = 0;
plantStruct.dynamicsType = "default";
plantStruct.attitudeType = "euler";
plantStruct.sequence = "313";

distStruct.disturbance = "grav";

simIn = initAqua(Tfinal, ICstruct, orbitStruct, plantStruct, distStruct);

simOut = sim(simIn);