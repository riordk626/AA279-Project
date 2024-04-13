clc, clear
close all

[rcm, Icm] = aquaMassProps();

%rcm = rcm.*100

[A,Icm_prime] = eig(Icm);


rcm_prime = A.'*rcm.';

x0_deg = [-7, 2, 5].';
x0 = x0_deg*pi/180;
Tfinal = 120;

load_system("eulerPropagate")

sim("eulerPropagate")

n = size(t,1);

T = zeros([n 1]);
L = zeros([n 3]);
for i=1:n
    L(i,:) = Icm_prime*om(i,:).';
    T = 0.5*dot(om(i,:).', L(i,:).');
end

figure 
hold on
plot(t, om(:,1))
plot(t, om(:,2))
plot(t, om(:,3))