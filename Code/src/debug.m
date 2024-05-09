
v = simOut.v.data(: , 1);
eci2principle = [1 2 3; 1 2 3; 1 2 3];

resultantAeroTorque(2, 1e-15, v, areas, normalVectors, centroids, rcm, eci2principle)

function M = resultantAeroTorque(cd, rho, v, A, N, centroid, rcm, eci2principle)

n = size(A, 1);
vUnit = v ./ norm(v);

M = zeros([3 1]);

for i=1:n
    if dot(N(i, :), vUnit) > 0
        force = -0.5 .* cd .* rho .* norm(v)^2 .* A(i) .* dot(transpose(N(i, :)), vUnit) .* vUnit;
        force_principle = eci2principle * reshape(force, [3,1]); 
        M = M + cross(transpose(centroid(i, :) - rcm), force_principle);
    end
end

end