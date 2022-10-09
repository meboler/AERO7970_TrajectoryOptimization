%{
Demo LinProg solver before cleaning everything up for submission.
%}
clc; clear all; close all;

import soln.linprogSolve;
method = 'dual-simplex';

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
params.T_F = 10 * 60; % finish time in seconds
params.N = 100; % # of discretization points

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
    "LowerBound", 0, ...
    "UpperBound", 3 * params.T_MAX);

problem = optimproblem;
problem.Objective = sum(S, 'all');

% Boundary and dynamics constraints
state_cons = optimconstr(6, params.N + 1);
state_cons(:, 1) = X(:, 1) == params.X_0;
state_cons(:, 2:params.N) = ...
    X(:, 2:params.N) == F * X(:, 1:(params.N - 1)) + G * U(:, 1:(params.N - 1));
state_cons(:, (params.N + 1)) = X(:, params.N) == params.X_F;

% L1 variable constraints
pos_cons = optimconstr(3, params.N - 1);
for i = 1 : params.N - 1
    pos_cons(:, i) = U(:, i) <= S(:, i);
end
neg_cons = optimconstr(3, params.N - 1);
for i = 1 : params.N - 1
    neg_cons(:, i) = U(:, i) >= -S(:, i);
end

problem.Constraints.pcons = state_cons;
problem.Constraints.scons = pos_cons;
problem.Constraints.tcons = neg_cons;

options = optimoptions('linprog', 'Algorithm', method); %, 'Display', 'off');

%% Solve Problem
soln = solve(problem, 'Options', options);
X = soln.X;
U = soln.U;