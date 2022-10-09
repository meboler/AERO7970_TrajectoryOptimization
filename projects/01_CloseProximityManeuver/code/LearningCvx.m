%{
Demo CVX solver before cleaning everything up for submission.
%}
clc; clear all; close all;

import soln.cvxSolve;

%% Define Solution Parameters

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

%% Solve Problem

cvx_begin
    variable X(6, params.N)
    variable U(3, params.N - 1)
    variable S(3, params.N - 1)
    
    minimize( sum(S(:)) )
    subject to
        % Boundary conditions
        X(:, 1) == params.X_0;
        X(:, params.N) == params.X_F;

        % Actuator limits
        U(:) <= params.T_MAX;
        U(:) >= -params.T_MAX;

        % L1 norm variables
        S(:) >= 0;
        S(:) <= 3 * params.T_MAX;

        for i = 1 : params.N - 1
            % Dynamics
            X(:, i+1) == F*X(:, i) + G * U(:, i);
            
            % L1 norm limits
            U(:, i) <= S(:, i);
            U(:, i) >= -S(:, i);
        end
cvx_end