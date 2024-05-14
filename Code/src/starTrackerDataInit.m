clc, clear, close all

rng(10)

V = rand([3 2]);
for i=1:size(V,2)
    V(:,i) = V(:,i)./norm(V(:,i));
end

save('starTrackerSimpleUndersampled.mat', "V")

clear V

V = rand([3 10]);
for i=1:size(V,2)
    V(:,i) = V(:,i)./norm(V(:,i));
end

save('starTrackerSimpleOversampled.mat', "V")