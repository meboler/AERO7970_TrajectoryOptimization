%{

%}
clc; clear all; close all;
format long

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
    'MaxTime', 1 * 60 * 60, ...
    'CutGeneration', 'advanced', ...
    'IntegerPreprocess', 'advanced', ...
    'PlotFcn', 'optimplotmilp');

[soln, fval, flag, out] = solve(problem, 'Solver', 'intlinprog', 'Options', opts);

selections = soln.X;
lowerBound = soln.S;

%% Visualize solution
masses = zeros(1, nStations);
for i = 1 : nStations
    masses(i) = asteroidMatrix(:, i)' * selections(:, i);
end

[minMass, minIdx] = min(masses);

figure()
bc = bar(masses);
bc.FaceColor = 'flat';
bc.CData(minIdx, :) = [1, 0, 0];
title("Station Masses");
ylabel("Mass (kg)")

disp("Final cost: " + string(1e-10 * minMass))
