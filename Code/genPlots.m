function genPlots(I_sim, name, flag, simIn)

    load_system('eulerPropagate.slx')
    
    simOut = sim(simIn);
    
    om = simOut.om_p;
    t = simOut.t;
    
    n = size(t,1);
    
    Tvec = zeros([n 1]);
    Lvec = zeros([n 3]);
    for i=1:n
        Lvec(i,:) = I_sim*om(i,:).';
        Tvec(i) = 0.5*dot(om(i,:).', Lvec(i,:).');
    end
    
    L = vecnorm(Lvec,2,2);
    
    T = Tvec(1)
    L = L(1)

    L^2/(2*T)
    
    
    % Integration
    figure 
    hold on
    plot(t, om(:,1), 'LineWidth',2, 'DisplayName', '\omega_x')
    plot(t, om(:,2), 'LineWidth',2, 'DisplayName', '\omega_y')
    plot(t, om(:,3), 'LineWidth',2, 'DisplayName', '\omega_z')
    hold off
    ax = gca();
    ax.FontSize = 14;
    xlabel('t [sec]')
    ylabel('\omega [rad/s]')
    legend
    exportgraphics(gcf, ['../Images/omega_prop_', name, '.png'])
    
    % Energy and Momentum Ellipsoids
    Ix = I_sim(1,1);
    Iy = I_sim(2,2);
    Iz = I_sim(3,3);
    % Energy and Momentum Ellipsoids
    a_energy = sqrt(2*T/Ix);
    b_energy = sqrt(2*T/Iy);
    c_energy = sqrt(2*T/Iz);
    n = 50;
    
    x = linspace(-a_energy, a_energy, n);
    X_energy = meshgrid(x);
    Y_energy = zeros(size(X_energy));
    Z_energy = zeros(size(X_energy));
    yu = real( sqrt((1/Iy).*(2*T - Ix.*x.^2)) );
    yl = -yu;
    for i=1:length(x)
        xi = X_energy(:,i).';
        yi = linspace(yl(i), yu(i), n);
        Y_energy(:,i) = yi;
        zi = real(sqrt((1/Iz).*(2*T - Ix.*xi.^2 - Iy.*yi.^2)));
        Z_energy(:,i) = zi;
    end
    
    figure
    surface(X_energy,Y_energy,Z_energy,'FaceAlpha', 0.3, 'FaceColor', 'g', 'HandleVisibility', 'off')
    surface(X_energy,Y_energy,-Z_energy,'FaceAlpha', 0.3, 'FaceColor', 'g', 'HandleVisibility', 'off')
    hold on

    quiver3(0,0,0, a_energy, 0,0, 'LineWidth', 5, 'DisplayName', 'a')
    quiver3(0,0,0, 0, b_energy,0, 'LineWidth', 5, 'DisplayName', 'b')
    quiver3(0,0,0, 0, 0,c_energy, 'LineWidth', 5, 'DisplayName', 'c')
    view([1 1 0.5])
    ax = gca();
    ax.FontSize = 14;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    legend
    exportgraphics(gcf, ['../Images/energy_axes_', name, '.png'])
    
    a_mom = L/Ix;
    b_mom = L/Iy;
    c_mom = L/Iz;
    
    x = linspace(-a_mom, a_mom, n);
    X_mom = meshgrid(x);
    Y_mom = zeros(size(X_mom));
    Z_mom = zeros(size(X_mom));
    yu = sqrt((1/Iy^2).*(L^2 - Ix^2.*x.^2));
    yl = -yu;
    for i=1:length(x)
        xi = X_mom(:,i).';
        yi = linspace(yl(i), yu(i), n);
        Y_mom(:,i) = yi;
        zi = real(sqrt((1/Iz^2).*(L^2 - Ix^2.*xi.^2 - Iy^2.*yi.^2)));
        Z_mom(:,i) = zi;
    end
    
    figure
    surface(X_mom,Y_mom,Z_mom,'FaceAlpha', 0.3, 'FaceColor', 'b', 'HandleVisibility', 'off')
    surface(X_mom,Y_mom,-Z_mom,'FaceAlpha', 0.3, 'FaceColor', 'b', 'HandleVisibility', 'off')
    hold on

    quiver3(0,0,0, a_mom, 0,0, 'LineWidth', 5, 'DisplayName', 'a')
    quiver3(0,0,0, 0, b_mom,0, 'LineWidth', 5, 'DisplayName', 'b')
    quiver3(0,0,0, 0, 0,c_mom, 'LineWidth', 5, 'DisplayName', 'c')
    view([1 1 0.5])
    ax = gca();
    ax.FontSize = 14;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    legend
    exportgraphics(gcf, ['../Images/momentum_axes_', name, '.png'])

    figure
    surface(X_energy,Y_energy,Z_energy,'FaceAlpha', 0.3, 'FaceColor', 'g', 'HandleVisibility', 'off')
    surface(X_energy,Y_energy,-Z_energy,'FaceAlpha', 0.3, 'FaceColor', 'g', 'DisplayName', 'Energy Ellispoid')
    surface(X_mom,Y_mom,Z_mom,'FaceAlpha', 0.3,'FaceColor','b','HandleVisibility', 'off')
    surface(X_mom,Y_mom,-Z_mom,'FaceAlpha', 0.3,'FaceColor', 'b', 'DisplayName', 'Momentum Ellipsoid')
    hold on
    
    
    if flag
        markerSim = 'o';
        markerTheo = 'x';
    else
        markerSim = '-';
        markerTheo = '--';
    end

    % Polhode plots
    plot3(om(:,1), om(:,2), om(:,3), ['r' markerSim], 'LineWidth', 5, 'DisplayName', 'Polhode')
    axis equal
    view([1 1 0.5])
    ax = gca();
    ax.FontSize = 14;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    legend
    exportgraphics(gcf, ['../Images/ellipsoid_polhode_', name, '.png'])
    
    xmin = min(om(:,1));
    xmax = max(om(:,1));
    ymin = min(om(:,2));
    ymax = max(om(:,2));
    zmin = min(om(:,3));
    zmax = max(om(:,3));
    
    x = linspace(xmin, xmax, n);
    
    figure
    subplot(3,1,1)
    ax = gca();
    ax.FontSize = 14;
    ax.LineWidth = 2;
    plot(om(:,1), om(:,2), markerSim, 'DisplayName', 'Simulated', 'LineWidth', 2)
    hold on
    y = real( sqrt( (L^2 - 2*T*Iz - (Ix - Iz).*Ix.*x.^2)/((Iy - Iz)*Iy) ) );
    plot(x,y, ['r', markerTheo], 'HandleVisibility','off', 'LineWidth', 2)
    plot(x,-y,['r' markerTheo], 'DisplayName', 'Theoretical', 'LineWidth', 2)
    xlabel('x')
    ylabel('y')
    legend
    axis equal
    subplot(3,1,2)
    ax = gca();
    ax.FontSize = 14;
    ax.LineWidth = 2;
    plot(om(:,1), om(:,3), markerSim, 'LineWidth', 2)
    hold on
    xlabel('x')
    ylabel('z')
    z = real( sqrt( (L^2 - 2*T*Iy - (Ix - Iy).*Ix.*x.^2)/((Iz - Iy)*Iz) ) );
    plot(x,z, ['r' markerTheo], 'LineWidth', 2)
    axis equal
    subplot(3,1,3)
    ax = gca();
    ax.FontSize = 14;
    ax.LineWidth = 2;
    plot(om(:,2), om(:,3), markerSim, 'LineWidth', 2)
    hold on
    xlabel('y')
    ylabel('z')
    y = linspace(ymin,ymax,n);
    z = real( sqrt( (L^2 - 2*T*Ix - (Iy - Ix).*Iy.*y.^2)/((Iz - Ix)*Iz) ) );
    plot(y,z, ['r' markerTheo], 'LineWidth', 2)
    axis equal

    exportgraphics(gcf, ['../Images/planar_polhode_', name, '.png'])
end