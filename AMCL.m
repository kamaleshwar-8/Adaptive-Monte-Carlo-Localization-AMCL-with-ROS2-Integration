% MATLAB code for Adaptive Monte Carlo Localization (AMCL) using Particle Filter
num_particles = 100; 
dt = 0.1; 
max_particles = 1000; 
min_particles = 100;
v = 1; 
omega = 0.1; 
R = diag([0.2, 0.1]); 
Q = diag([0.1, 0.1, 0.05]);
landmarks = [5, 5; -5, 5; -5, -5; 5, -5];
particles = repmat([0; 0; 0], 1, num_particles) + [randn(2, num_particles); randn(1, num_particles) * 0.1];
weights = ones(1, num_particles) / num_particles;

% ROS setup
clear ros2node
node = ros2node("/matlab_node");
posPub = ros2publisher(node, '/drone/position', 'geometry_msgs/Point');
pause(0.5);

for t = 1:100
    % Prediction Step
    particles = particles + [v * cos(particles(3, :)) * dt + Q(1, 1) * randn(1, num_particles);
                             v * sin(particles(3, :)) * dt + Q(2, 2) * randn(1, num_particles);
                             omega * dt + Q(3, 3) * randn(1, num_particles)];
    % Measurement Update Step
    for i = 1:num_particles
        weight = 1;
        for j = 1:size(landmarks, 1)
            dx = landmarks(j, 1) - particles(1, i);
            dy = landmarks(j, 2) - particles(2, i);
            z = [sqrt(dx^2 + dy^2); atan2(dy, dx) - particles(3, i)] + R * randn(2, 1);
            expected = [sqrt(dx^2 + dy^2); atan2(dy, dx) - particles(3, i)];
            weight = weight * exp(-0.5 * (z - expected)' * inv(R) * (z - expected));
        end
        weights(i) = weight;
    end
    weights = weights / sum(weights);
    % Resampling Step
    indices = randsample(1:num_particles, num_particles, true, weights);
    particles = particles(:, indices);
    % Adaptive Particle Count
    spread = mean(sum((particles(1:2, :) - mean(particles(1:2, :), 2)).^2, 1));
    if spread > 5, num_particles = min(max_particles, round(num_particles * 1.5));
    elseif spread < 1, num_particles = max(min_particles, round(num_particles * 0.75));
    end
    % Re-initialize particles if necessary
    if num_particles ~= size(particles, 2)
        particles = repmat([0; 0; 0], 1, num_particles) + [randn(2, num_particles); randn(1, num_particles) * 0.1];
        weights = ones(1, num_particles) / num_particles;
    end
    % Visualization and ROS publication
    clf; 
    plot(landmarks(:,1), landmarks(:,2), 'bo', particles(1,:), particles(2,:), 'r.', mean(particles(1,:)), mean(particles(2,:)), 'bo','MarkerSize', 10, 'LineWidth', 2);
    title('Adaptive Particle Filter Localization'); 
    axis([-10 10 -10 10]); 
    drawnow;
    posMsg = ros2message('geometry_msgs/Point'); 
    [posMsg.x, posMsg.y, posMsg.z] = deal(mean(particles(1,:)), mean(particles(2,:)), 0); 
    send(posPub, posMsg);
    pause(0.1);
end