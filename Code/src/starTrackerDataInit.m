clc, clear, close all

rng(10)

Vstar = rand([3 2]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

save('starTrackerSimpleUndersampled.mat', "Vstar")

clear Vstar

Vstar = rand([3 10]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

save('starTrackerSimpleOversampled.mat', "Vstar")