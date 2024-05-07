function stabilityPlot(t, om, u)

figure()
subplot(2,1,1)
omplot = plot(t, om, 'LineWidth', 2);
set(omplot, {'DisplayName'}, {'\omega_x';'\omega_y';'\omega_z'})
ylabel('\omega [rad/s]')
legend
ax = gca();
ax.FontSize = 14;
subplot(2,1,2)
uplot = plot(t, u, 'LineWidth', 2);
set(uplot, {'DisplayName'}, {'\phi';'\theta';'\psi'})
xlabel('t [sec]')
ylabel('u [rad]')
legend
ax = gca();
ax.FontSize = 14;