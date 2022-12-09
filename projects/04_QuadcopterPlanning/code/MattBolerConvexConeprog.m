%{
Solve a quadrotor trajectory optimization problem using Matlab's coneprog
solver

@author Matt Boler
@date December 06, 2022
%}
clc; clear all; close all;

%% Generate Plots
problem = soln.ConeprogProblem();
problem.obstacles = [0.5, 1.5, 0; 1.5, 3.5, 0]';
[Xf, history] = problem.solve();
problem.plot(history)

problem = soln.ConeprogProblem();
problem.obstacles = [1.0, 2.0, 0; 2.0, 5.0, 0]';
[Xf, history] = problem.solve();
problem.plot(history)

%% Performance Testing
disp("Testing ConeProg performance")
times = zeros(1, 10);
for i = 1 : 10
    solver = soln.ConeprogProblem();
    tic;
    solver.solve();
    dt = toc;
    times(i) = dt;
    disp(" - Iteration: " + num2str(i) + ", " + num2str(dt))
end
disp("Average time to solve with ConeProg: ")
disp(mean(times));