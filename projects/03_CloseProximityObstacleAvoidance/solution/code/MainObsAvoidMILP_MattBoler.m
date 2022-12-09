%{
AERO 7970 Trajectory Optimization
Project 1 - Rendezvous

@author Matt Boler
@date 10/07/2022
%}
clear all; close all;

import soln.*;

%% Define Constants
params = soln.getDefaultParams();

%% Solving obstacle avoidance problem
disp("--- Solving obstacle avoidance ---")
[X, U, ~] = soln.solveProblem(params, true);
[X_orig, U_orig, ~] = soln.solveProblem(params, false);

%% Plotting
time = linspace(0, params.T_F / 60 - 1, params.N - 1);
coords = [...
    params.X_MIN, params.Y_MIN, params.Z_MIN; ...
    params.X_MAX, params.Y_MIN, params.Z_MIN; ...
    params.X_MAX, params.Y_MAX, params.Z_MIN; ...
    params.X_MIN, params.Y_MAX, params.Z_MIN; ...
    params.X_MIN, params.Y_MIN, params.Z_MAX; ...
    params.X_MAX, params.Y_MIN, params.Z_MAX; ...
    params.X_MAX, params.Y_MAX, params.Z_MAX; ...
    params.X_MIN, params.Y_MAX, params.Z_MAX];
idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';
xc = coords(:, 1);
yc = coords(:, 2);
zc = coords(:, 3);

figure()
plot(X(1,:), X(2,:))
hold on
plot(X_orig(1,:), X_orig(2,:))
legend("With Obstacle Avoidance", "Without Obstacle Avoidance")
xlabel("X (m)")
ylabel("Y (m)")
title("Horizontal Trajectory Comparisons")

figure()
scatter3(X(1, :), X(2, :), X(3, :), '.');
hold on
scatter3(X_orig(1,:), X_orig(2,:), X_orig(3,:), '*')
patch(xc(idx), yc(idx), zc(idx), 'r', 'facealpha', 0.5);
xlabel("X (m)")
ylabel("Y (m)")
xlim([min(X(1,:)) - 10, max(X(1,:)) + 10]);
ylim([min(X(2,:)) - 10, max(X(2,:)) + 10]);
zlim([min(X(3,:)) - 10, max(X(3,:)) + 10]);
legend("With Obstacle Avoidance", "Without Obstacle Avoidance")
title("Trajectory Comparisons")

figure()
subplot(3,1,1)
plot(time, U(1,:))
hold on
plot(time, U_orig(1,:))
title("X Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

subplot(3,1,2)
plot(time, U(2,:))
hold on
plot(time, U_orig(2,:))
title("Y Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

subplot(3,1,3)
plot(time, U(3,:))
hold on
plot(time, U_orig(3,:))
title("Z Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

sgtitle("Control Effort Comparisons")

%% Solving modified obstacle avoidance problem
params = soln.getDefaultParams();
params.Y_MIN = 0.35 * 1e3;
disp("--- Solving modified obstacle avoidance ---")
[X, U, ~] = soln.solveProblem(params, true);
[X_orig, U_orig, ~] = soln.solveProblem(params, false);

%% Plotting
coords = [...
    params.X_MIN, params.Y_MIN, params.Z_MIN; ...
    params.X_MAX, params.Y_MIN, params.Z_MIN; ...
    params.X_MAX, params.Y_MAX, params.Z_MIN; ...
    params.X_MIN, params.Y_MAX, params.Z_MIN; ...
    params.X_MIN, params.Y_MIN, params.Z_MAX; ...
    params.X_MAX, params.Y_MIN, params.Z_MAX; ...
    params.X_MAX, params.Y_MAX, params.Z_MAX; ...
    params.X_MIN, params.Y_MAX, params.Z_MAX];
idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';
xc = coords(:, 1);
yc = coords(:, 2);
zc = coords(:, 3);



figure()
scatter3(X(1, :), X(2, :), X(3, :), '.');
hold on
scatter3(X_orig(1,:), X_orig(2,:), X_orig(3,:), '*')
patch(xc(idx), yc(idx), zc(idx), 'r', 'facealpha', 0.5);
xlabel("X (m)")
ylabel("Y (m)")
xlim([min(X(1,:)) - 10, max(X(1,:)) + 10]);
ylim([min(X(2,:)) - 10, max(X(2,:)) + 10]);
zlim([min(X(3,:)) - 10, max(X(3,:)) + 10]);
legend("With Obstacle Avoidance", "Without Obstacle Avoidance")
title("Modified Trajectory Comparisons")

figure()
subplot(3,1,1)
plot(time, U(1,:))
hold on
plot(time, U_orig(1,:))
title("X Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

subplot(3,1,2)
plot(time, U(2,:))
hold on
plot(time, U_orig(2,:))
title("Y Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

subplot(3,1,3)
plot(time, U(3,:))
hold on
plot(time, U_orig(3,:))
title("Z Thrust")
legend("With Obstacle Avoidance", "Without Obstace Avoidance")

sgtitle("Modified Control Effort Comparisons")