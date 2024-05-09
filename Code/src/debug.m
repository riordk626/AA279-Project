
v = [1; 2; 3];
eci2principle = [1 2 3; 1 2 3; 1 2 3];

resultantAeroTorque(2, 1e-15, v, areas, normalVectors, centroids, rcm, eci2principle, A_ptob, 10)

function M = resultantAeroTorque(cd, rho, v, A, N, centroids, rcm, eci2principle, A_ptob, v_mag)
M = zeros([3 1]);
n = size(A, 1);

%Rotate everything to principle frame
v = eci2principle*v;
N = N.';
centroids = centroids.';
rcm = A_ptob.' * rcm.';
for i=1:n
    N(:, i) = A_ptob.' * N(:, i);
    centroids(:, i) = A_ptob.' * centroids(:, i);
end 

vUnit = v ./ v_mag;

for i=1:n
    if dot(N(:, i), vUnit) > 0
        force = -0.5 .* cd .* rho .* v_mag^2 .* A(i) .* dot(N(:, i), vUnit) .* vUnit;
        M = M + cross(centroids(:, i) - rcm, force);
    end
end
end