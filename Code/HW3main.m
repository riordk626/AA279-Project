clc, clear

close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

%% Axis Symmetric

I_sim = Itotal_p;

I_sim(2,2) = I_sim(1,1);


om0_deg = [-7, 2, 5].';
om0 = deg2rad(om0_deg);
Tfinal = 300;
axesFlag = 0;
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput('eulerPropagate');
simIn.ExternalInput = M;

load_system("eulerPropagate")
% open_system("eulerPropagate")

simOut = sim(simIn);

om_p = squeeze(simOut.om_p);
t = simOut.t;

om_sim = om_p;

Ix = I_sim(1,1); Iy = Ix;
Iz = I_sim(3,3);

lambda = (Iz - Ix)/Iy * om0(3);
om_al = zeros(size(om_sim));

om_al(3,:) = om0(3).*ones(size(om_al(3,:)));
om_al(1,:) = om0(1).*cos(lambda.*t) - om0(2).*sin(lambda.*t);
om_al(2,:) = om0(1).*sin(lambda.*t) + om0(2).*cos(lambda.*t);

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

plot3(simOut.om{1}.Values.Data(1,:), simOut.om{1}.Values.Data(2,:), simOut.om{1}.Values.Data(3,:))
%% Asymmetric

I_sim = Itotal_p;


q0 = [1 0 0 0].';
u0 = [0.1 0.1 0.1].';

simIn = Simulink.SimulationInput('aquaMasterModel');
simIn.ExternalInput = M;

legendNames = {{'\phi', '\theta', '\psi'}, {'q0', 'q1', 'q2', 'q3'}};
stateNames = {'u', 'q'};
unitNames = {'[rad]', ''};
imageNames = {'EA.png', 'quat.png'};

for Type = 1:2
    
    load_system("aquaMasterModel")
    % open_system("eulerPropagate")
    
    simOut = sim(simIn);
    
    R = simOut.yout{1}.Values.Data;
    t = simOut.t;
    
    n = size(t,1);
    
    om_p = squeeze(simOut.om_p).';
    om_i = zeros(size(om_p.'));
    L_i = zeros(size(om_i));
    
    for i=1:n
        om_i(:,i) = R(:,:,i).' * om_p(i,:).';
        L_p = Itotal_p * om_p(i,:).';
        L_i(:,i) = R(:,:,i).' * L_p;
    end

    eval([stateNames{Type}, '= squeeze(simOut.', stateNames{Type}, ');'])
    % Genrates time history of attitude parameters
    figure
    eval(['plot(t, ', stateNames{Type}, ', ''LineWidth'', 2)'])
    ax = gca();
    ax.FontSize = 14;
    xlabel('t [sec]')
    ylabel([stateNames{Type}, ' ', unitNames{Type}])
    legend(legendNames{Type})
    exportgraphics(gcf, ['../Images/time_history_', imageNames{Type}])
    
    % Generate herpolhode plot (ineretial frame polhode)
    figure
    plot3(om_i(1,:), om_i(2,:), om_i(3,:), 'LineWidth', 2)
    ax = gca();
    ax.FontSize = 14;
    axis equal
    xlabel('\omega_x')
    ylabel('\omega_y')
    zlabel('\omega_z')
    % hold off
    exportgraphics(gcf, ['../Images/herpolhode_', imageNames{Type}])

    % Generate angular momentum vector plot (inertial)
    figure
    plot3(L_i(1,:), L_i(2,:), L_i(3,:), 'LineWidth', 2)
    ax = gca();
    ax.FontSize = 14;
    xlabel('L_x')
    ylabel('L_y')
    zlabel('L_z')
    axis equal
    exportgraphics(gcf, ['../Images/angular_momentum_', imageNames{Type}])

    % Generate reference frame plot in motion
    % figure
    % 
    % ax = gca();
    % ax.FontSize = 14;
    % axis equal
    % exportgraphics(gcf, ['../Images/reference_frame_motion_', imageNames{Type}])
end