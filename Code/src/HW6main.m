%% Initialization
clc, clear
close all

projectStartup;
exportflag = false;
figurePath = '../../Images/PS6/';

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

%% Problem 1

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

% plot solar torque
distStruct.disturbance = "solar";
% distStruct.disturbance = "all":

Tfinal = 10*T;

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct);

simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_mag = squeeze(simOut.M_mag);
r_mag = vecnorm(r,2,1);

cg2cp = 7.6578;
q = 0.66;
meanSolarFlux = 4.4e-6;
A = 224.68;

M_mag_bound = meanSolarFlux*A*(1+q)*cg2cp * ones(size(t));

figure()
plot(t, M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_mag, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'solar_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Solar Torque')

% plots magnetic torque

distStruct.disturbance = "mag";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct);

simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_mag = squeeze(simOut.M_mag);
r_mag = vecnorm(r,2,1);
mag_data = load('magConstants.mat', 'm_sat', 'Re', 'B0');
m_sat = mag_data.m_sat;
Re = mag_data.Re;
B0 = mag_data.B0;
m_sat_mag = norm(m_sat);

M_mag_bound = 2*m_sat_mag*Re^3*B0./(r_mag.^3);

figure()
plot(t, M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_mag, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'magnetic_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Magnetic Torque')

% plots aero torque

distStruct.disturbance = "aero";

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct);
simOut = sim(simIn);

t = simOut.t;
r = squeeze(simOut.r.Data);
M_mag = squeeze(simOut.M_mag);
r_mag = vecnorm(r,2,1);

cg2cp = 7.6578;
cd = 2;
A = 224.68;
rho = 1e-15;
vMag = norm(simOut.v.Data(:, :, 1));

M_mag_bound = 0.5*rho*vMag^2*A*cd*cg2cp * ones(size(t));


figure()
plot(t, M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_u')
hold on
plot(t, -M_mag_bound, 'b--', 'LineWidth', 2, 'DisplayName', 'M_l')
p = plot(t, M_mag, 'LineWidth', 2);
set(p, {'DisplayName'}, {'M_x';'M_y';'M_z'})
xlabel('t [sec]')
ylabel('M [Nm]')
ax = gca();
ax.FontSize = 14;
legend
figureName = [figurePath, 'aero_torque.png'];

exportgraphics(gcf, figureName)
sgtitle(gcf, 'Aero Torque')


%% Problem 2 & 3

omx = 0;
omy = 0;
omz = 0;

om0 = [omx omy omz].';
R_ECItoRTN = eci2rtn(r0, v0);
R_RTNtoBdes = [0 1 0;0 0 1;1 0 0];
R_RTNtoPdes = A_ptob.' * R_RTNtoBdes;
R0 = R_RTNtoPdes * R_ECItoRTN;

ICstruct.om0 = om0; ICstruct.R0 = R0;

distStruct.disturbance = "grav";
% distStruct.disturbance = "all":

Tfinal = 5*T;

simIn = initAqua(Tfinal, R_RTNtoPdes, ICstruct, orbitStruct, plantStruct, distStruct);

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
figureName = [figurePath, 'attitude_error.png'];

fig = figure();
timeHistoryPlot(fig, t,values,valueNames,valueLabels,figureName,exportflag)