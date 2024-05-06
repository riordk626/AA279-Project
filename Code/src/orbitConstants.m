% Orbital Elements
e_float = 0.0000979; % eccentricity 
a_float = 7080.6; % km
i_float = deg2rad(98.2); %degrees
omega_float = deg2rad(120.4799); % arguement of perigee // degrees
Omega_float = deg2rad(95.2063); % ascending node // degrees
nu_float = 0; % True Anomaly (in radians)
mu_float = 3.986004418e5; % Gravitational parameter of the Earth in km^3/s^2

semimajorAxis = [[0, a_float]; [10000000, a_float]];
eccentricity = [[0, e_float]; [10000000, e_float]];
inclination = [[0, i_float]; [10000000, i_float]];
omega = [[0, omega_float]; [10000000, omega_float]];
Omega = [[0, Omega_float]; [10000000, Omega_float]];
trueAnomaly = [[0, nu_float]; [10000000, nu_float]];
mu = [[0, mu_float]; [10000000, mu_float]];

% Calculate orbital period
T = 2*pi*sqrt((a_float)^3 / mu_float); % Orbital period in seconds

% Calculate mean motion
n_float = 2*pi / T; % Mean motion in rad/s
mean_motion = [[0, n_float],
    [10000000, n_float]];