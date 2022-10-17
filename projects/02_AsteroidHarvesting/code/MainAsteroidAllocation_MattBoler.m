%{

%}
clc; clear all; close all;
import soln.*;

%% Load and process data
% Load as table
asteroidTable = soln.getData();

% Convert to matrix and remove ID column
asteroidMatrix = table2array(asteroidTable);
asteroidMatrix = asteroidMatrix(:, 2:end);

% Make it simple
asteroidMatrix = asteroidMatrix(1:25, :);

[nAsteroids, nStations] = size(asteroidMatrix);

%% Build problem
problem = soln.buildProblem(asteroidMatrix);

%% Solve

% https://www.mathworks.com/help/optim/ug/tuning-integer-linear-programming.html

options = optimoptions(...
    'intlinprog');

[soln, fval, flag, out] = solve(problem);

selections = soln.X;
lowerBound = soln.S;

%% Visualize solution
masses = zeros(1, nStations);
for i = 1 : nStations
    masses(i) = asteroidMatrix(:, i)' * selections(:, i);
end

figure()
bar(masses)
title("Station Masses")