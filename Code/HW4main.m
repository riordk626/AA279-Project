clc, clear

close all

[rcm, Itotal_b, Itotal_p, A_ptob] = aquaMassProps();

%% Problem 2 - Stability Test

w0_array = deg2rad(10.*eye(3));

