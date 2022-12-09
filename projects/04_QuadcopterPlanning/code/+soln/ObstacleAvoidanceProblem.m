classdef ObstacleAvoidanceProblem
    %OBSTACLEAVOIDANCEPROBLEM Base class for obstacle avoidance solvers
    
    properties (Abstract)
        name string
    end

    properties
        % Area bounds
        xLim (1, 2) double = [-1, 4]
        yLim (1, 2) double = [-1, 7]
        zLim (1, 2) double = [-1, 1]

        % Obstacles
        nObstacles              = 2
        obstacles (3, :) double = [0.5, 1.5, 0; 1.5, 3.5, 0]'
        R                double = 0.4
        H         (3, 3) double = diag([1, 1, 0])

        % Dynamic properties
        gravity     double = 9.81;
        uLim (1, 2) double = [0.6, 23.2];
        thetaMax    double = 100;

        % Initial and final conditions
        rInitial (3, 1) double = [0, 0, 0]'
        rFinal   (3, 1) double = [2.5, 6, 0]'
        vInitial (3, 1) double = [0, 0, 0]'
        vFinal   (3, 1) double = [0, 0, 0]'
        tFinalInit      double = 2.0
        tFinal          double = 2.0

        % Optimization parameters
        nPoints          = 25
        nMaxIters        = 5
        rTol      double = 0.5
        w         double = 1e-5
        alpha     double = 1.5

        % Utility objects
        e1 (3, 1) = [1, 0, 0]'
        e2 (3, 1) = [0, 1, 0]'
        e3 (3, 1) = [0, 0, 1]'
    end
    
    % Override these methods for your implementation!
    methods (Abstract)
        [solution, status] = initialize(this)
        [solution, status, history] = refine(this, initialSolution)
    end
    
    % Core class methods
    methods
        function [res, history] = solve(this)
            %SOLVE Solve Problem 2 using Algorithm 1 from [1]
            %   Detailed explanation goes here

            isSolved = false;

            % Step 1: Set initial tFinal
            this.tFinal = this.tFinalInit;

            while ~isSolved
                % Step 2: Solve P2 without Eq4
                [initialSolution, initialStatus] = this.initialize();

                %   Step 2.1: If feasible, apply successive convexification
                if initialStatus == 1
                    [refinedSolution, refinedStatus, history] = this.refine(initialSolution);
                    %       Step 2.1.1: Exit if converged
                    if refinedStatus == 1
                        isSolved = true;
                        %       Step 2.1.2: Else if nIters > nMaxIters, set tf =
                        %       alpha*tf and return to Step 1
                    else
                        this.tFinal = this.alpha * this.tFinal;
                        disp("Reseting: " + num2str(this.tFinal))
                    end

                    %   Step 2.2: Else return to Step 1 and set tf = alpha * tf
                else
                    this.tFinal = this.alpha * this.tFinal;
                end
            end

            res = refinedSolution;
        end

        function plot(this, solutions)
            %PLOT Plot a solution to the problem
            %   Detailed explanation goes here
            arguments
                this
                solutions (1, :) struct
            end

            figure()
            hold on

            dt = this.tFinal / this.nPoints;
            times = 0 : dt : this.tFinal - dt;

            % Plot boundary conditions
            scatter(this.rInitial(1), this.rInitial(2))
            scatter(this.rFinal(1), this.rFinal(2))

            % Plot trajectory
            trajLabels = [];
            for i = 1 : length(solutions)
                sol = solutions(i);
                plot(sol.r(1,:), sol.r(2,:))
                trajLabels = [trajLabels, "Iteration: " + num2str(i)];
            end

            % Plot obstacles
            for j = 1 : this.nObstacles
            rectangle(...
                'Position', [this.obstacles(1, j) - this.R, this.obstacles(2, j) - this.R, 2*this.R, 2*this.R], ...
                'Curvature', [1, 1])
            end

            % Plot state bounds
            line(...
                [this.xLim(1), this.xLim(1)], ...
                [this.yLim(1), this.yLim(2)])
            line(...
                [this.xLim(2), this.xLim(2)], ...
                [this.yLim(1), this.yLim(2)])
            line(...
                [this.xLim(1), this.xLim(2)], ...
                [this.yLim(1), this.yLim(1)])
            line(...
                [this.xLim(1), this.xLim(2)], ...
                [this.yLim(2), this.yLim(2)])

            % Set properties
            title("Generated Trajectory: " + this.name)
            xlabel("X Position (m)")
            ylabel("Y Position (m)")
            legend(["Initial Position", "Goal Position", trajLabels], ...
                'Location', 'southeast')
            xlim(this.xLim)
            ylim(this.yLim)
            axis equal
            grid on
            grid minor
            hold off

            figure()
            hold on
            for i = 1 : length(solutions)
                sol = solutions(i);
                plot(times(1:end-1), vecnorm(sol.u, 2, 1));
            end
            title("Generated Control: " + this.name)
            xlabel("Time (s)")
            ylabel("||U||_2 (m/s^2)")
            legend(trajLabels)

        end

    end
end

