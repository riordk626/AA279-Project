function [r0, v0] = orbitalICs()

orbitConstants;

[r0, v0] = keplerian2ECI(a_float, e_float, i_float, Omega_float, omega_float, nu_float, mu_float);