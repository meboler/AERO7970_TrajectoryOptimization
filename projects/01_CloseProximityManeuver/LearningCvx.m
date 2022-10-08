%{
Demo CVX solver before cleaning everything up for submission.
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

cvx_begin
    variable X(6, N)
    variable U(3, N-1)
    variable S(3, N-1)
    
    minimize( sum(S(:)) )
    subject to
        % Boundary conditions
        X(:, 1) == X_0;
        X(:, N) == X_F;

        % Actuator limits
        U(:) <= T_MAX;
        U(:) >= -T_MAX;

        % L1 norm variables
        S(:) >= 0;

        for i = 1 : N-1
            % Dynamics
            X(:, i+1) == F*X(:, i) + G * U(:, i);
            
            % L1 norm limits
            U(:, i) <= S(:, i);
            U(:, i) >= -S(:, i);
        end
cvx_end
