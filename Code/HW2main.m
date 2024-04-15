clc, clear
close all

[rcm, Icm] = aquaMassProps();

%rcm = rcm.*100

[A,Icm_prime] = eig(Icm);


rcm_prime = A.'*rcm.';


% Random ICs

x0_deg = [-7, 2, 5].';
x0 = x0_deg*pi/180;
Tfinal = 120;

genPlots(Icm_prime, 'random')

x0_deg = [10, 0, 0].';
x0 = x0_deg*pi/180;
Tfinal = 120;

genPlots(Icm_prime, 'principal')
