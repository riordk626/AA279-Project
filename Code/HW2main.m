clc, clear
close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();
I_sim = Itotal_p;
%rcm = rcm.*100

rcm_prime = A_ptob.'*rcm.';

% Random ICs

om0_deg = [-7, 2, 5].';
om0 = om0_deg*pi/180;
Tfinal = 500;
axesFlag = 0;
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput('eulerPropagate');
simIn.ExternalInput = M;

genPlotsHW2(I_sim, 'random', 0, simIn)

om0_deg = [10, 0, 0].';
om0 = om0_deg*pi/180;
Tfinal = 120;

genPlotsHW2(I_sim, 'principal', 1, simIn)


