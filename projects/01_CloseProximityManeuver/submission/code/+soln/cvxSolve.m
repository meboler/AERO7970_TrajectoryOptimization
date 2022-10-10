function [X, U, status] = cvxSolve(params)
%CVXSOLVE Summary of this function goes here
%   Detailed explanation goes here

%% Define System
dt = params.T_F / params.N;

% Angular velocity of LVLH frame
OMEGA = sqrt(params.MU / (params.ALT + params.R_E)^3); % = rad / s

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
     1 / params.M, 0, 0; ...
     0, 1 / params.M, 0; ...
     0, 0, 1 / params.M];
G = B*dt;

%% Solve Problem

cvx_begin quiet
    variable X(6, params.N)
    variable U(3, params.N - 1)
    variable S(3, params.N - 1)
    
    minimize( sum(S(:)) )
    subject to
        % Boundary conditions
        X(:, 1) == params.X_0;
        X(:, params.N) == params.X_F;

        % Actuator limits
        U(:) <= params.T_MAX;
        U(:) >= -params.T_MAX;

        % L1 norm variables
        S(:) >= 0;

        for i = 1 : params.N - 1
            % Dynamics
            X(:, i+1) == F*X(:, i) + G * U(:, i);
            
            % L1 norm limits
            U(:, i) <= S(:, i);
            U(:, i) >= -S(:, i);
        end
cvx_end

status = strcmp(cvx_status, 'Solved');

end

