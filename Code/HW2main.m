clc, clear
close all

[rcm, Icm] = aquaMassProps();

%rcm = rcm.*100

[A,Icm_prime] = eig(Icm)


rcm_prime = A.'*rcm.'
