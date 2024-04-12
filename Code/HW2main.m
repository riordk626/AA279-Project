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

figure 
hold on
plot(t, om(:,1))
plot(t, om(:,2))
plot(t, om(:,3))