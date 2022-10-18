%{

Form:

max min f'x s.t. 
    x on [0, 1]
    each asteroid selected once
    
general take: maximize S with constraint that all masses greater than S

X will be a vector of [s1, s2, ..., sn]' where 
    s_i = [0, ..., 1, ..., 0] e.g. a selector

%}
clc; clear all; close all;
import soln.*;

%% Load and process data
asteroidMatrix = ...
    [3, 2; ...
     2, 4; ...
     1, 2];
[nAsteroids, nStations] = size(asteroidMatrix);

%% Demo selector matrix

% A = [nStations, nAsteroids] = asteroidMatrix'
% x = [nAsteroids, ?]
% b = [nStations, ?]

A = asteroidMatrix;
s1 = [1, 0, 1]';
s2 = [0, 1, 0]';

%% Try solving with matlab

X = optimvar('X', nAsteroids, nStations, ...
    'Type', 'integer', ...
    'LowerBound', 0, ...
    'UpperBound', 1);
S = optimvar('S', 'LowerBound', 0);

problem = optimproblem('ObjectiveSense', 'maximize'); 
problem.Objective = S;

% Mass at each station 
mass_cons = optimconstr(nStations);
for i = 1 : nStations
    mass_cons(i) = A(:, i)' * X(:, i) >= S;
end
problem.Constraints.mcons = mass_cons;

% Select every asteroid once
sel_cons = optimconstr(nAsteroids);
for i = 1 : nAsteroids
    sel_cons(i) = sum(X(i, :)) == 1;
end
problem.Constraints.scos = sel_cons;

[sol, fval, flag, out] = solve(problem)