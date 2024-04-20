clc, clear

close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

I_sim = Itotal_p;

I_sim(2,2) = I_sim(1,1);


x0_deg = [-7, 2, 5].';
x0 = deg2rad(x0_deg);
Tfinal = 300;
axesFlag = 0;
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput('eulerPropagate');
simIn.ExternalInput = M;

load_system("eulerPropagate")
% open_system("eulerPropagate")

simOut = sim(simIn);

om_p = simOut.om_p;
t = simOut.t;

om_sim = om_p;

Ix = I_sim(1,1); Iy = Ix;
Iz = I_sim(3,3);

lambda = (Iz - Ix)/Iy * x0(3);
om_al = zeros(size(om_sim));

om_al(:,3) = x0(3).*ones(size(om_al(:,3)));
om_al(:,1) = x0(1).*cos(lambda.*t) - x0(2).*sin(lambda.*t);
om_al(:,2) = x0(1).*sin(lambda.*t) + x0(2).*cos(lambda.*t);

error = om_al - om_sim;


figure()
subplot(2,1,1)
splot = plot(t, om_sim, 'LineWidth',2);
set(splot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ax = gca();
ax.FontSize = 14;
% xlabel('t [sec]')
ylabel('\omega_{sim} [rad/s]')
subplot(2,1,2)
aplot = plot(t, om_al, 'LineWidth', 2);
set(aplot, {'DisplayName'}, {'\omega_x';'\omega_y'; '\omega_z'})
ax = gca();
ax.FontSize = 14;
xlabel('t [sec]')
ylabel('\omega_{analytical} [rad/s]')
legend
exportgraphics(gcf, '../Images/sim_vs_anlt_magnitude.png')


figure()
splot = plot(t, error, 'LineWidth',2);
set(splot, {'DisplayName'}, {'r_x';'r_y'; 'r_z'})
ax = gca();
ax.FontSize = 14;
xlabel('t [sec]')
ylabel('Error [rad/s]')
legend
exportgraphics(gcf, '../Images/sim_vs_anlt_error.png')

