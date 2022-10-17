function [problem] = buildProblem(asteroidData)
%BUILDPROBLEM Summary of this function goes here
%   Detailed explanation goes here

[nAsteroids, nStations] = size(asteroidData);

% Create maximization problem
problem = optimproblem('ObjectiveSense', 'maximize');

% Optimize over selection matrix X
X = optimvar('X', nAsteroids, nStations, ...
    'Type', 'integer', ...
    'LowerBound', 0, ...
    'UpperBound', 1);

% Optimize lower bound of station masses
S = optimvar('S', 'LowerBound', 0);

% Set mass lower bound as optimization target
problem.Objective = S;

% Each station's mass is greater than the lower bound
massCons = optimconstr(nStations);
for i = 1 : nStations
    massCons(i) = asteroidData(:, i)' * X(:, i) >= S;
end
problem.Constraints.mCons = massCons;

% Each asteroid is selected once
selCons = optimconstr(nAsteroids);
for i = 1 : nAsteroids
    selCons(i) = sum(X(i, :)) == 1;
end
problem.Constraints.sCons = selCons;

end

