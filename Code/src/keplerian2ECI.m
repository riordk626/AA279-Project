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