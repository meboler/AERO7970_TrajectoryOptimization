%{
Demo LinProg solver before cleaning everything up for submission.
%}
clc; clear all; close all;

%% Define Constants
MU = 398600 * 1e9; % m^3 / s^2
R_E = 6378 * 1e3; % m
ALT = 550 * 1e3; % m
T_MAX = 200 * 1e-3; % kg * m/s^2
M = 50; % kg
T_F = 30 * 60; % seconds
N = 100; % # of discretization points

% Angular velocity of LVLH frame
OMEGA = sqrt(MU / (ALT + R_E)^3); % = rad / s

%% Define System
dt = T_F / N;

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
     1/M, 0, 0; ...
     0, 1/M, 0; ...
     0, 0, 1/M];
G = B*dt;

%% Build Problem

X_0 = [-1e3, 0, 0, 0, 0, 0]';
X_F = [0, 0, 0, 0, 0, 0]';

% State and control variables
X = optimvar('X', 6, N);
U = optimvar('U', 3, N-1, ...
    "LowerBound", -T_MAX, ...
    "UpperBound", T_MAX);

% Constraint variables for L1 optimization
S = optimvar('S', 3, N-1, ...
    "LowerBound", 0, ...
    "UpperBound", 3 * T_MAX);

problem = optimproblem;
problem.Objective = sum(S, 'all');

% Boundary and dynamics constraints
state_cons = optimconstr(6, N+1);
state_cons(:, 1) = X(:, 1) == X_0;
state_cons(:, 2:N) = X(:, 2:N) == F * X(:, 1:(N-1)) + G * U(:, 1:(N-1));
state_cons(:, (N+1)) = X(:, N) == X_F;

% L1 variable constraints
pos_cons = optimconstr(3, N-1);
for i = 1 : N-1
    pos_cons(:, i) = U(:, i) <= S(:, i);
end
neg_cons = optimconstr(3, N-1);
for i = 1 : N-1
    neg_cons(:, i) = U(:, i) >= -S(:, i);
end

problem.Constraints.pcons = state_cons;
problem.Constraints.scons = pos_cons;
problem.Constraints.tcons = neg_cons;

%% Solve

options = optimoptions('linprog', 'Algorithm', 'dual-simplex');
tic
soln = solve(problem, 'Options', options);
toc