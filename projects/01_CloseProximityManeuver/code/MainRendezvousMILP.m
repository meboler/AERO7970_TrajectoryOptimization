%{
AERO 7970 Trajectory Optimization
Project 1 - Rendezvous

@author Matt Boler
@date 10/07/2022
%}
clear all; close all;

import soln.*;

%% Define Constants
% Constants
params.MU = 398600 * 1e9; % m^3 / s^2
params.R_E = 6378 * 1e3; % m
params.ALT = 550 * 1e3; % m
params.T_MAX = 200 * 1e-3; % kg * m/s^2
params.M = 50; % kg

% Solution parameters
params.X_0 = [-1e3, 0, 0, 0, 0, 0]'; % start point
params.X_F = [0, 0, 0, 0, 0, 0]'; % goal point
params.T_F = 30 * 60; % finish time in seconds
params.N = 100; % # of discretization points

%% Performance comparisons
disp("---Beginning Performance Comparisons---")
n_comp = 10;
dual_simplex_times = zeros(1, n_comp);
interior_point_times = zeros(1, n_comp);
cvx_times = zeros(1, n_comp);

% Note: For fairness, timing for all methods includes code to formulate problem,
% since CVX will not let you separate the formulation and solution steps.

for i = 1 : n_comp
    tic
    [X_simplex, U_simplex, ~] = soln.linprogSolve(params, 'dual-simplex');
    dual_simplex_times(i) = toc;

    tic
    [X_interior, U_interior, ~] = soln.linprogSolve(params, 'interior-point');
    interior_point_times(i) = toc;

    tic
    [X_cvx, U_cvx, ~] = soln.cvxSolve(params);
    cvx_times(i) = toc;

    if i == 1
        figure(1)
        scatter(X_simplex(1,:), X_simplex(2,:), 'o');
        hold on
        scatter(X_interior(1,:), X_interior(2,:), '*');
        scatter(X_cvx(1,:), X_cvx(2,:), '+')
        legend("Dual-Simplex", "Interior-Point", "CVX");
        xlabel("X (m)")
        ylabel("Y (m)")
        title("Trajectory Comparisons (Fuel Optimal)")

        figure(2)

        time = linspace(0, params.T_F / 60 - 1, params.N - 1);

        subplot(3,1,1)
        plot(time, U_simplex(1, :), 'o');
        hold on
        plot(time, U_interior(1,:), '*');
        plot(time, U_cvx(1,:), '+')
        title("Thrust X-Component")
        legend("Dual-Simplex", "Interior-Point", "CVX", "Location", 'southwest')
        xlabel('Time (Min)')
        ylabel('Thrust (N)')
        ylim([-params.T_MAX, params.T_MAX])

        subplot(3,1,2)
        plot(time, U_simplex(2, :), 'o');
        hold on
        plot(time, U_interior(2,:), '*');
        plot(time, U_cvx(2,:), '+')
        title("Thrust Y-Component")
        xlabel("Time (Min)")
        ylabel("Thrust (N)")
        ylim([-params.T_MAX, params.T_MAX])

        subplot(3,1,3)
        plot(time, U_simplex(3, :), 'o');
        hold on
        plot(time, U_interior(3,:), '*');
        plot(time, U_cvx(3,:), '+');
        title("Thrust Z-Component")
        xlabel("Time (Min)")
        ylabel("Thrust (N)")
        ylim([-params.T_MAX, params.T_MAX])

        sgtitle("Control Comparisons (Fuel Optimal)")
    end
end

disp("Solver Performances:")
disp("Dual-Simplex: " + string(mean(dual_simplex_times)));
disp("Interior-Point: " + string(mean(interior_point_times)));
disp("CVX: " + string(mean(cvx_times)));

%% Minimum Maneuver Time
disp("---Beginning Minimum Time Search---")
success = true;
while success == true
    params.T_F = params.T_F - 60;
    [X_min_time_simplex_tmp, U_min_time_simplex_tmp, status_simplex] = soln.linprogSolve(params, 'dual-simplex');
    [X_min_time_interior_tmp, U_min_time_interior_tmp, status_interior] = soln.linprogSolve(params, 'interior-point');
    [X_min_time_cvx_tmp, U_min_time_cvx_tmp, status_cvx] = soln.cvxSolve(params);

    if status_simplex == 0 || status_interior == 0 || status_cvx == 0
        success = false;
    else
        X_min_time_simplex = X_min_time_simplex_tmp;
        U_min_time_simplex = U_min_time_simplex_tmp;
        
        X_min_time_interior = X_min_time_interior_tmp;
        U_min_time_interior = U_min_time_interior_tmp;

        X_min_time_cvx = X_min_time_cvx_tmp;
        U_min_time_cvx = U_min_time_cvx_tmp;
    end
end
disp("Minimum maneuver time: " + string( round(params.T_F/60 + 1) ) + " minutes")

figure(3)
scatter(X_min_time_simplex(1,:), X_min_time_simplex(2,:), 'o');
hold on
scatter(X_min_time_interior(1,:), X_min_time_interior(2,:), '*');
scatter(X_min_time_cvx(1,:), X_min_time_cvx(2,:), '+')
legend("Dual-Simplex", "Interior-Point", "CVX");
xlabel("X (m)")
ylabel("Y (m)")
title("Trajectory Comparisons (Time Optimal)")

figure(4)

time = linspace(0, params.T_F / 60 - 1, params.N - 1);

subplot(3,1,1)
plot(time, U_min_time_simplex(1, :), 'o');
hold on
plot(time, U_min_time_interior(1,:), '*');
plot(time, U_min_time_cvx(1,:), '+')
title("Thrust X-Component")
legend("Dual-Simplex", "Interior-Point", "CVX", "Location", 'southwest')
xlabel('Time (Min)')
ylabel('Thrust (N)')
ylim([-params.T_MAX, params.T_MAX])

subplot(3,1,2)
plot(time, U_min_time_simplex(2, :), 'o');
hold on
plot(time, U_min_time_interior(2,:), '*');
plot(time, U_min_time_cvx(2,:), '+')
title("Thrust Y-Component")
xlabel("Time (Min)")
ylabel("Thrust (N)")
ylim([-params.T_MAX, params.T_MAX])

subplot(3,1,3)
plot(time, U_min_time_simplex(3, :), 'o');
hold on
plot(time, U_min_time_interior(3,:), '*');
plot(time, U_min_time_cvx(3,:), '+');
title("Thrust Z-Component")
xlabel("Time (Min)")
ylabel("Thrust (N)")
ylim([-params.T_MAX, params.T_MAX])

sgtitle("Control Comparisons (Time Optimal)")