rStarUnitSamples = [5 3;
2 1;
0 9];

perturbMeasurements(rStarUnitSamples)

function rStarUnitPerturbed = perturbMeasurements(rStarUnitSamples)

rStarUnitPerturbed = zeros(size(rStarUnitSamples));

% Bias and standard dev from known star tracker data (Honeywell MST)
mu = 5 * (pi / 180 / 3600); % Bias error in radians (converted from 5 arcseconds)
sigma = 1 * (pi / 180 / 3600); % Standard deviation of noise in radians (converted from 1 arcseconds)

nStars = length(rStarUnitSamples(1,:));

for i = 1:nStars

    % Get one star measurement
    rStarUnit = rStarUnitSamples(:,i);

    % Convert to spherical coordinates
    x = rStarUnit(1);
    y = rStarUnit(2); 
    z = rStarUnit(3);
    r = sqrt(x^2 + y^2 + z^2);
    theta = acos(z / r);
    phi = atan2(y, x);
    
    % Calculate the displacement due to bias error
    d_bias = mu;
    
    % Generate Gaussian noise and calculate displacement
    noise = sigma .* randn(1); % Angular noise in radians
    d_noise = noise;
    
    % Add the errors to the sperical coords
    theta_perturbed = theta + d_bias + d_noise;
    phi_perturbed = phi + d_bias + d_noise;
    
    % Convert back to spherical coordinates
    x_perturbed = r * sin(theta_perturbed) * cos(phi_perturbed);
    y_perturbed = r * sin(theta_perturbed) * sin(phi_perturbed);
    z_perturbed = r * cos(theta_perturbed);
    
    u_measured = [x_perturbed; y_perturbed; z_perturbed];
    
    % Normalize the measured vector to ensure it remains a unit vector
    u_measured = u_measured ./ norm(u_measured);

    rStarUnitPerturbed(:,i) = u_measured;
    
end

end