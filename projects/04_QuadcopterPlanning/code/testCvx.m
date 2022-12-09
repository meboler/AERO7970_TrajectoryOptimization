%{

%}
clc; clear all; close all;

%% Tests
testCvxSoln()

%% Functions
function testCvxSoln()
    problem = soln.CvxProblem();
    [X0, status] = problem.initialize();
    [Xf, status, history] = problem.refine(X0);
    problem.plot(history)
end