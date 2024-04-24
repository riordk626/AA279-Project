
% Constants
rE = 6378.1; % km
AU = 1.496e8; % 1 AU in km
mu = 3.986004418e5; % Gravitational parameter of the Earth in km^3/s^2

% Define initial conditions (orbital elements)
a = 7080.6; % Semi-major axis (in km)
e = 0.0000979;   % Eccentricity
i = deg2rad(98.2); % Inclination (in radians)
omega = deg2rad(120.4799); % Arguement of perigee (in radians)
Omega = deg2rad(95.2063); % Ascending node (in radians)
nu = 0; % True Anomaly (in radians)

% Calculate orbital period
T = 2*pi*sqrt((a)^3 / mu); % Orbital period in seconds

% Define time span for propagation
t_span = orbit_prop_time_series; 

% Define initial conditions (position and velocity vectors)
[r0, v0] = keplerian2ECI(a, e, i, Omega, omega, nu, mu);

% Define function for orbital equations of motion
orbital_eqns = @(t, y) orbital_derivatives(t, y, mu);

% Use ode45 solver to propagate orbit
% [t, y] = ode45(orbital_eqns, t_span, [r0; v0]);
[t, y] = ode23(orbital_eqns, t_span, [r0; v0]);

% Extract position and velocity vectors from solution
r = y(:, 1:3);
v = y(:, 4:6);

% Find DCM from ECI to RTN for each timestep
t_size = size(t,1);
coords_orbital = zeros(size(R));
for i = 1:t_size
    % Access the ith row of the array
    r_vec = r(i, :);
    v_vec = v(i, :);

    % Store rotation matrix
    dcm = eci2rtn(r_vec, v_vec);
    coords_orbital(:,:,i) = dcm;
    
end

% Plot orbit in 3D
plot_orbit(r, rE);

function [rECI, vECI] = keplerian2ECI(a, e, i, Omega, omega, nu, mu)

    % Calculate position and velocity in the perifocal coordinate system
    E = acos((e + cos(nu))/(1+(e*cos(nu))));
    n = sqrt(mu/a^3);
    r_perifocal = [a*(cos(E) - e); a*sin(E)*sqrt(1-e^2); 0];
    v_perifocal = a*n/(1-e*cos(E)) * [-sin(E); cos(E)*sqrt(1-e^2); 0];

    % Rotation matrices
    R3_Omega = [cos(-Omega) sin(-Omega) 0;
                -sin(-Omega) cos(-Omega) 0;
                0 0 1];
    R1_i = [1 0 0;
            0 cos(-i) sin(-i);
            0 -sin(-i) cos(-i)];
    R3_omega = [cos(-omega) sin(-omega) 0;
                -sin(-omega) cos(-omega) 0;
                0 0 1];

    % Transformation to ECI
    Q_perifocal2ECI = R3_Omega * R1_i * R3_omega;
    rECI = Q_perifocal2ECI * r_perifocal;
    vECI = Q_perifocal2ECI * v_perifocal;
end

function dydt = orbital_derivatives(~, y, mu)
    % Define the equations of motion for orbital derivatives
    r = y(1:3);
    v = y(4:6);
    r_mag = norm(r);
    dvdt = -mu * r / r_mag^3;
    drdt = v;
    dydt = [drdt; dvdt];
end

function R_eci_to_rtn = eci2rtn(r_eci, v_eci)
    
    % Compute radial, transverse, and normal vectors in ECI frame
    r_radial_eci = r_eci;
    r_transverse_eci = cross(r_radial_eci, v_eci);
    r_normal_eci = cross(r_radial_eci, r_transverse_eci);
    
    % Normalize radial, transverse, and normal vectors to obtain unit vectors
    r_radial_eci_unit = r_radial_eci / norm(r_radial_eci);
    r_transverse_eci_unit = r_transverse_eci / norm(r_transverse_eci);
    r_normal_eci_unit = r_normal_eci / norm(r_normal_eci);
    
    % Construct rotation matrix from ECI to RTN
    R_eci_to_rtn = [r_radial_eci_unit; r_transverse_eci_unit; r_normal_eci_unit];

end

function plot_orbit(r, rE)
    % Plot orbit in 3D
    figure;
    plot3(r(:,1), r(:,2), r(:,3), 'r', 'LineWidth', 2.5);
    hold on;
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    title('Aqua Orbit');
    grid on;

    [xE, yE, zE] = ellipsoid(0,0,0,rE, rE, rE, 20);
    surface(xE, yE, zE, 'FaceColor', 'blue', 'EdgeColor', 'black')
    
    axis equal;
    view(3);
end
