clc, clear
close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

Tfinal = 300;

%% Problem 1

cb = [0 0 1].';
cp = A_ptob.' * cb;
cx = cp(1);
cy = cp(2);
cz = cp(3);

orbitConstants;

omz = sqrt(3*n_float^2 * cz);
omx = 3*n_float^2 * cz * cx/omz;
omy = 3*n_flaot^2 * cy * cz/omz;

om0 = [omx omy omz].';
u0 = 

%% Functions

function u = RtoEuler313(R)

    u = zeros([3 1]);

    u(1) = atan2(R(1,3), R(2,3));
    u(2) = acos(R(3,3));
    u(3) = atan2(R(3,1), -R(3,2));

end

function u = RtoEuler312(R)

    u = zeros([3 1]);

    u(1) = atan2(R(1,2), R(2,2));
    u(2) = -asin(R(3,2));
    u(3) = atan2(R(3,1), R(3,3));

end

function R_eci_to_rtn = eci2rtn(r_eci, v_eci)
    
    % Compute radial, transverse, and normal vectors in ECI frame
    r_radial_eci = r_eci;
    r_normal_eci = cross(r_radial_eci, v_eci);
    r_transverse_eci = -cross(r_radial_eci, r_normal_eci);
    
    % Normalize radial, transverse, and normal vectors to obtain unit vectors
    r_radial_eci_unit = r_radial_eci / norm(r_radial_eci);
    r_transverse_eci_unit = r_transverse_eci / norm(r_transverse_eci);
    r_normal_eci_unit = r_normal_eci / norm(r_normal_eci);
    
    % Construct rotation matrix from ECI to RTN
    R_eci_to_rtn = [r_radial_eci_unit.'; r_transverse_eci_unit.'; r_normal_eci_unit.'];

end

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

function plot_orbit(r, rE)
    % Plot orbit in 3D
    figure;
    plot3(r(1, :), r(2, :), r(3, :), 'r', 'LineWidth', 2.5);
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