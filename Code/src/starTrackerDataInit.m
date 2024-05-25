clc, clear, close all

rng(10)
wmag = 0.3;

Vstar = rand([3 2]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

wstar = rand([2 1]);

save('attitudeEdgeMeasData.mat', "Vstar", "wstar", "wmag")

clear Vstar wstar

Vstar = rand([3 10]);
for i=1:size(Vstar,2)
    Vstar(:,i) = Vstar(:,i)./norm(Vstar(:,i));
end

wstar = (1e-2).*rand([10 1]) + 0.9;

save('attitudeMeasData.mat', "Vstar", "wstar", "wmag")