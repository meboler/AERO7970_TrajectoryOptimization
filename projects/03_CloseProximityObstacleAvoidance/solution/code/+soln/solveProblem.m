function [X, U, status] = solveProblem(params, obstacle)
%LINPROGSOLVE Summary of this function goes here
%   Detailed explanation goes here

%% Define System
dt = params.T_F / params.N;

% Angular velocity of LVLH frame
OMEGA = sqrt(params.MU / (params.ALT + params.R_E)^3); % = rad / s

A = [0, 0, 0, 1, 0, 0; ...
    0, 0, 0, 0, 1, 0; ...
    0, 0, 0, 0, 0, 1; ...
    3 * OMEGA^2, 0, 0, 0, 2 * OMEGA, 0; ...
    0, 0, 0, -2 * OMEGA, 0, 0; ...
    0, 0, -OMEGA^2, 0, 0, 0];
F = expm(A*dt);

B = [0, 0, 0; ...
    0, 0, 0; ...
    0, 0, 0; ...
    1 / params.M, 0, 0; ...
    0, 1 / params.M, 0; ...
    0, 0, 1 / params.M];
G = B*dt;

%% Build Problem

% State and control variables
X = optimvar('X', 6, params.N);
U = optimvar('U', 3, params.N - 1, ...
    "LowerBound", -params.T_MAX, ...
    "UpperBound", params.T_MAX);

% Constraint variables for L1 optimization
S = optimvar('S', 3, params.N - 1, ...
    "LowerBound", 0);

problem = optimproblem;
problem.Objective = sum(S, 'all');

% Boundary and dynamics constraints
state_cons = optimconstr(6, params.N + 1);
state_cons(:, 1) = X(:, 1) == params.X_0;
state_cons(:, 2:params.N) = ...
    X(:, 2:params.N) == F * X(:, 1:(params.N - 1)) + G * U(:, 1:(params.N - 1));
state_cons(:, (params.N + 1)) = X(:, params.N) == params.X_F;

problem.Constraints.pcons = state_cons;

% L1 variable constraints
pos_cons = optimconstr(3, params.N - 1);
for i = 1 : params.N - 1
    pos_cons(:, i) = U(:, i) <= S(:, i);
end

neg_cons = optimconstr(3, params.N - 1);
for i = 1 : params.N - 1
    neg_cons(:, i) = U(:, i) >= -S(:, i);
end

problem.Constraints.scons = pos_cons;
problem.Constraints.tcons = neg_cons;


% Obstacle avoidance constraints
if obstacle == true
    % Switching variables for Big-M
    Z = optimvar('Z', 6, params.N, ...
        'Type', 'integer', ...
        'LowerBound', 0, ...
        'UpperBound', 1);

    M = 1e5;

    upper_cons = optimconstr(3, params.N);
    lower_cons = optimconstr(3, params.N);
    active_cons = optimconstr(params.N);

    for i = 1 : params.N
        upper_cons(1, i) = -X(1, i) <= -params.X_MAX + Z(1, i) * M;
        upper_cons(2, i) = -X(2, i) <= -params.Y_MAX + Z(2, i) * M;
        upper_cons(3, i) = -X(3, i) <= -params.Z_MAX + Z(3, i) * M;

        lower_cons(1, i) = X(1, i) <= params.X_MIN + Z(4, i) * M;
        lower_cons(2, i) = X(2, i) <= params.Y_MIN + Z(5, i) * M;
        lower_cons(3, i) = X(3, i) <= params.Z_MIN + Z(6, i) * M;

        active_cons(i) = sum(Z(:, i)) <= 5;
    end
    problem.Constraints.ucons = upper_cons;
    problem.Constraints.lcons = lower_cons;
    problem.Constraints.acons = active_cons;
end

options = optimoptions('intlinprog');

%% Solve Problem
[soln, ~, flag, ~] = solve(problem, 'Options', options);
X = soln.X;
U = soln.U;

status = flag == 1;

end

