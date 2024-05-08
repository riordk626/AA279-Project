%% Magnetic Disturbace Torque Constants

N = 2;
mu0 = 1.25663706212e-6; % [N/A^2]
ncoils = [15, 15].';
S = [0.2, 3].';
deltaI = [1, 1].';
m_satb = zeros([3 N]);
m_hatb = [0 1 0; 1 0 0].';
Re = 6731.8; % [km]
B0 = 30800e-9;
mhat_ecef = [0 sind(10) cosd(10)].';

for i=1:N
    m_satb(:,i) = mu0.*S(i).*ncoils(i).*deltaI(i).*m_hatb(:,i);
end

[~, ~, ~, A_ptob] = aquaMassProps();

m_sat = A_ptob.' * m_satb;
m_sat = sum(m_sat, 2);