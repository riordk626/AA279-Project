clc, clear, close all

rng(10)

Vstar = rand([3 2]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

wstar = rand([2 1]);

save('starTrackerSimpleUndersampled.mat', "Vstar", "wstar")

clear Vstar

Vstar = rand([3 10]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

wstar = (1e-2).*rand([10 1]) + 0.9;

wmag = 0.3;

save('attitudeMeasData.mat', "Vstar", "wstar", "wmag")