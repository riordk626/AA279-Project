function u = attitudeECItoRTN(R_ItoP, R_ECItoRTN, sequence)
tsteps = size(R_ItoP, 3);

u = zeros([3 tsteps]);
for i=1:tsteps
    R = R_ItoP(:,:,i) * R_ECItoRTN(:,:,i).';
    u(:,i) = RtoEuler(R, sequence);
end