clc, clear
close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();
I_sim = Itotal_p;
%rcm = rcm.*100

rcm_prime = A_ptob.'*rcm.';


% Random ICs

x0_deg = [-7, 2, 5].';
x0 = x0_deg*pi/180;
Tfinal = 120;

genPlots(I_sim, 'random', 0)

x0_deg = [10, 0, 0].';
x0 = x0_deg*pi/180;
Tfinal = 120;

genPlots(I_sim, 'principal', 1)


