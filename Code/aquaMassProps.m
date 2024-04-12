function [rcm, Itotal] = aquaMassProps()


% Instrument Moments of Inertia

% instruments = {'MODIS', 'AMSU-A1', 'AMSU-A2', 'AIRS', 'HSB', 'CERES', 'AMSR-E'};
lxs = [1.5, 1.5, 0.75, 1.5, 0.75, 3.54, 1.2];
lys = [2.5, 2.5, 1.25, 1.25,1.25, 2.5, 2.5];
lzs = [1.5, 1.5, 1.5, 1.5, 1.5, 0.5, 3.3];
ms = [229, 49, 42, 177, 51, 100, 314];
Vs = lxs.*lys.*lzs;
rhos = ms./Vs;
Icms = zeros([3 3 7]);

for i = 1:7
    Icms(:,:,i) = rectInertia(lxs(i),lys(i),lzs(i),rhos(i));
end

% Chassis

V = 34.5;
Vsub = 3.3*2.5*1;
Vt = V + Vsub;
m = 1607;
rho = m/V;

It = rectInertia(6.84, 2.5, 2.5, rho);

Is = rectInertia(3.3,2.5,1,rho);

xbar = 1.77;
ybar = 0;
zbar = -0.75;

Is = parallelAxis(Is,xbar,ybar,zbar,rho,Vsub);

Ichassis = It - Is;

xbart = 3.42;
ybart = 1.25;
zbart = 1.75;

xbars = xbart + xbar;
ybars = ybart + ybar;
zbars = zbart + zbar;

xbarc = (Vt*xbart - Vsub*xbars)/V;
ybarc = (Vt*ybart - Vsub*ybars)/V;
zbarc = (Vt*zbart - Vsub*zbars)/V;

xtild = xbart - xbarc;
ytild = ybart - ybarc;
ztild = zbart - zbarc;

Ichassis = parallelAxis(Ichassis, xtild, ytild, ztild, -rho, V);

% Solar Panel

lx = 3.8;
ly = 14.2;
lz = 0.1;
V = lx*ly*lz;
m = 245;
rho = m/V;

Isolar = rectInertia(lx,ly,lz,rho);

% COM

ms = [ms, 1607, 245];
Vs = [Vs, 34.5, 5.396];
rhos = ms./Vs;
Icms = cat(3, Icms, Ichassis, Isolar);

xis = [7.29, 5.79, 4.665, 4.29, 3.915, 1.77, 7.44, xbarc, 8.04/2];
yis = [1.25, 1.25, 1.875, 0.625, 1.875, 1.25, 1.25, ybarc, 9.6];
zis = [0.75, 0.75, 0.75, 0.75, 0.75, 0.25, 3.15, zbarc, 2.25];

rcm = centerMass(ms, xis, yis, zis);

% Total Moment of Inertia

% xis(8) = 6.84/2;
% zis(8) = 1.75;

x0s = xis - rcm(1);
y0s = yis - rcm(2);
z0s = zis - rcm(3);

Itotal = zeros([3 3]);

for i=1:9
    Itotal = Itotal + parallelAxis(Icms(:,:,i), x0s(i), y0s(i), z0s(i), rhos(i), Vs(i));
end

% Sub-Functions

function I = rectInertia(lx, ly, lz, rho)

% rho = m/V;

I = zeros([3 3]);

% Ixy = 0, Iyz = 0, Ixz = 0;

Ixx = (1/12).*rho.*lx.*(ly.^3.*lz + lz.^3.*ly);
Iyy = (1/12).*rho.*ly.*(lx.^3.*lz + lz.^3.*lx);
Izz = (1/12).*rho.*lz.*(lx.^3.*ly + ly.^3.*lx);

I(1,1) = Ixx;
I(2,2) = Iyy;
I(3,3) = Izz;

end

function I = parallelAxis(Icm, x, y, z, rho, V)

Iplus = rho*V*[y^2 + z^2, -x*y, -x*z;
            -x*y, x^2 + z^2, -y*z;
            -x*z, -y*z, x^2 + y^2];

I = Icm + Iplus;

end

function xyzBar = centerMass(ms, xs, ys, zs)

xbar = sum(xs.*ms)/sum(ms);
ybar = sum(ys.*ms)/sum(ms);
zbar = sum(zs.*ms)/sum(ms);

xyzBar = [xbar, ybar, zbar];

end

end