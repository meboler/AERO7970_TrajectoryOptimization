%{

%}
clear all; close all;
format longg 

import soln.*;

%% Load and process data
% Load as table
asteroidTable = soln.getData();

% Convert to matrix and remove ID column
asteroidMatrix = table2array(asteroidTable);
asteroidMatrix = asteroidMatrix(:, 2:end);

[nAsteroids, nStations] = size(asteroidMatrix);

%% Build problem
problem = soln.buildProblem(asteroidMatrix);

%% Solve
% https://www.mathworks.com/help/optim/ug/tuning-integer-linear-programming.html
opts = optimoptions('intlinprog', ...
    'MaxTime', 30 * 60);

[soln, fval, flag, out] = solve(problem, 'Solver', 'intlinprog', 'Options', opts);

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