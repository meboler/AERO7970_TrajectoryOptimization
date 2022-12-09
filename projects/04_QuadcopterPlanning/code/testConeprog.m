%{

%}
clc; clear all; close all;

%% Tests
testConeprogSoln()

%% Functions
function testConeprogSoln()
    problem = soln.ConeprogProblem();
    [X0, status] = problem.initialize();
    [Xf, status, history] = problem.refine(X0);
    problem.plot(history)
end