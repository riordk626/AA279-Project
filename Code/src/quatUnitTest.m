clc, clear
close all

load("omegaUnitTest.mat", "om")
[~, ~, Itotal_p, ~] = aquaMassProps();
Itotal_p(2,2) = Itotal_p(1,1);
t = om.Time;

Tfinal = om.Time(end);
q0 = [0 0 1 0].';
simIn = Simulink.SimulationInput('quaternionPropagate');
simIn.ExternalInput = om;

load_system("quaternionPropagate.slx")

confObj = getActiveConfigSet('quaternionPropagate');
set_param(confObj, 'OutputOption', 'SpecifiedOutputTimes')
set_param(confObj, 'OutputTimes', 't')

simOut = sim(simIn);

R = simOut.R{1}.Values.Data;

om_p = om.Data;
om_i = zeros(size(om_p.'));
L_i = zeros(size(om_i));

n = size(t,1);

for i=1:n
    om_i(:,i) = R(:,:,i).' * om_p(i,:).';
    L_p = Itotal_p * om_p(i,:).';
    L_i(:,i) = R(:,:,i).' * L_p;
end


% plot3(om_i(1,:), om_i(2,:), om_i(3,:), 'r')
% hold on
plot3(L_i(1,:), L_i(2,:), L_i(3,:), 'b')
axis equal
% hold off