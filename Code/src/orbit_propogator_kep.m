% Constants
rE = 6378.1; % km
AU = 1.496e8; % 1 AU in km
mu_earth = 3.986004418e5; % Gravitational parameter of the Sun in km^3/s^2

% Initial Conditions
e_float = 0.0000979; % eccentricity 
a_float = 7080.6; % km
i_float = 98.2; %degrees
omega_float = 120.4799; % arguement of perigee // degrees
Omega_float = 95.2063; % ascending node // degrees

a = [[0, a_float],
    [10000000, a_float]];
e = [[0, e_float],
    [10000000, e_float]];
i = [[0, i_float],
    [10000000, i_float]];
omega = [[0, omega_float],
    [10000000, omega_float]];
Omega = [[0, Omega_float],
    [10000000, Omega_float]];

% Calculate orbital period
T = 2*pi*sqrt((a_float)^3 / mu_earth); % Orbital period in seconds
T_minutes = T / (60); % Convert to Earth years

% Calculate mean motion
n_float = 2*pi / T; % Mean motion in rad/s
n = [[0, n_float],
    [10000000, n_float]];


fprintf('Orbital Period (T): %.2f seconds\n', T);
fprintf('Orbital Period (T): %.2f minutes\n', T_minutes);
fprintf('Mean Motion (n): %f rad/s\n', n_float);
disp(n)

% Plot orbit
figure;
plot3(out.x.Data, out.y.Data, out.z.Data, 'r', 'LineWidth', 1.5);
hold on;

[xE, yE, zE] = ellipsoid(0,0,0,rE, rE, rE, 20);
surface(xE, yE, zE, 'FaceColor', 'blue', 'EdgeColor', 'black')

% Label axes and include units
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('Aqua Orbit');
grid on;
axis equal;
view(4);

%% Sun Elements

Sun_Omega = ...
  [0 Sun_Omega_float;
   1.0E+7 Sun_Omega_float];

Sun_Omega_float = deg2rad(348.74);

Sun_T = 365.256*24*3600;

Sun_a_float = 149.6e6;

Sun_e_float = 0.0167086;

Sun_eccentricity = ...
  [0 Sun_e_float;
   1.0E+7 Sun_e_float];

Sun_i_float = deg2rad(23.44);

Sun_inclination = ...
  [0 Sun_i_float;
   1.0E+7 Sun_i_float];

Sun_n_float = 2*pi / Sun_T; % Mean motion in rad/s

Sun_mean_motion = ...
  [0 Sun_n_float;
   1.0E+7 Sun_n_float];

Sun_mu_float = 1.327124e11;

Sun_mu = ...
  [0 Sun_mu_float;
   1.0E+7 Sun_mu_float];

Sun_omega_float = deg2rad(102.93);

Sun_omega = ...
  [0 Sun_omega_float;
   1.0E+7 Sun_omega_float];

Sun_semimajorAxis = ...
  [0 7080.6;
   1.0E+7 7080.6];

Sun_trueAnomaly = ...
  [0 0;
   1.0E+7 0];

function E = solveKeplersEquation(M, e, epsilon)
    E = M; % Initial Guess
    while true
        E_new = E - (E - e*sin(E) - M) / (1 - e*cos(E)); % Newton-Raphson iteration
        if abs(E_new - E) < epsilon
            break; % Convergence reached
        end
        E = E_new;
    end
end

function [x,y,z]  = keplerian2ECI(a, e, i, Omega, omega, E)
    % Constants
    mu = 3.986e5; % Earth's gravitational parameter (km^3/s^2)

    % Convert orbital elements from degrees to radians
    i = deg2rad(i);
    Omega = deg2rad(Omega);
    omega = deg2rad(omega);

    % Calculate position and velocity in the perifocal coordinate system
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
    x = rECI(1);
    y = rECI(2);
    z = rECI(3);
end