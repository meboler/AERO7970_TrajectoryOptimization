function params = getDefaultParams()
%GETDEFAULTPARAMS Summary of this function goes here
%   Detailed explanation goes here

%% Define Constants
% Constants
params.MU = 398600 * 1e9; % m^3 / s^2
params.R_E = 6378 * 1e3; % m
params.ALT = 550 * 1e3; % m
params.T_MAX = 200 * 1e-3; % kg * m/s^2
params.M = 50; % kg

% Obstacle parameters
params.X_MIN = -0.6 * 1e3; % m
params.X_MAX = -0.5 * 1e3; % m

params.Y_MIN = 0.45 * 1e3; % m
params.Y_MAX = 0.55 * 1e3; % m

params.Z_MIN = -0.01 * 1e3; % m
params.Z_MAX = 0.01 * 1e3; % m

% Solution parameters
params.X_0 = [-1e3, 0, 0, 0, 0, 0]'; % start point
params.X_F = [0, 0, 0, 0, 0, 0]'; % goal point
params.T_F = 30 * 60; % finish time in seconds
params.N = 100; % # of discretization points
end

