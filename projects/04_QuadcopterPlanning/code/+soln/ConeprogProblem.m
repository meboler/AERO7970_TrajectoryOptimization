classdef ConeprogProblem < soln.ObstacleAvoidanceProblem
    %CONEPROGPROBLEM Coneprog-based solver
    properties
        name = "Coneprog"
    end

    methods
        function this = ConeprogProblem()
            %CONEPROGPROBLEM Construct an instance of this class
            %   Detailed explanation goes here
        end

        % Solver methods
        function [solution, status] = initialize(this)
            %INITIALIZE Solve the initial relaxed problem
            %   Detailed explanation goes here

            problem = optimproblem;

            % Apply boundary conditions
            r = optimvar('r', 3, this.nPoints, ...
                "LowerBound", ones(3, this.nPoints) .* [this.xLim(1); this.yLim(1); this.zLim(1)], ...
                "UpperBound", ones(3, this.nPoints) .* [this.xLim(2); this.yLim(2); this.zLim(2)]);
            v = optimvar('v', 3, this.nPoints);
            u = optimvar('u', 3, this.nPoints - 1);

            rBoundaryCons = optimconstr(3, 2);
            vBoundaryCons = optimconstr(3, 2);
            uBoundaryCons = optimconstr(3, 2);

            rBoundaryCons(:, 1) = r(:, 1) == this.rInitial;
            vBoundaryCons(:, 1) = v(:, 1) == this.vInitial;
            uBoundaryCons(:, 1) = u(:, 1) == this.gravity * this.e3;

            rBoundaryCons(:, end) = r(:, end) == this.rFinal;
            vBoundaryCons(:, end) = v(:, end) == this.vFinal;
            uBoundaryCons(:, end) = u(:, end) == this.gravity * this.e3;

            % Apply dynamic constraints
            rDynamicCons = optimconstr(3, this.nPoints - 1);
            vDynamicCons = optimconstr(3, this.nPoints - 1);

            dt = this.tFinal / this.nPoints;
            for t = 1 : this.nPoints - 1
                rDynamicCons(:, t) = r(:, t+1) == r(:, t) + dt * v(:, t);
                vDynamicCons(:, t) = v(:, t+1) == v(:, t) + dt * ( u(:, t) - this.gravity * this.e3 );
            end

            % Apply input constraints
            gamma = optimvar('gamma', 1, this.nPoints-1, ...
                'LowerBound', this.uLim(1), ...
                'UpperBound', this.uLim(2));
            uSlackCons = optimconstr(1, this.nPoints-1);
            uAngleCons = optimconstr(1, this.nPoints-1);

            for t = 1 : this.nPoints-1
                uSlackCons(t) = norm(u(:, t)) <= gamma(t);
                uAngleCons(t) = gamma(t) * cosd(this.thetaMax) <= this.e3' * u(:, t);
            end

            problem.Constraints.rBoundaryCons = rBoundaryCons;
            problem.Constraints.vBoundaryCons = vBoundaryCons;
            problem.Constraints.uBoundaryCons = uBoundaryCons;
            problem.Constraints.rDynamicCons = rDynamicCons;
            problem.Constraints.vDynamicCons = vDynamicCons;
            problem.Constraints.uSlackCons = uSlackCons;
            problem.Constraints.uAngleCons = uAngleCons;

            % Convert quadratic cost to cone form
            y = optimvar('y');
            phi = [gamma, y]';

            H = 2 * this.w * dt * eye(length(gamma));
            A = sqrtm(H);
            Asc = blkdiag(A, 1);

            yCons = optimconstr(1);
            yCons(1) = norm(Asc * phi) <= (y+1);

            problem.Constraints.yCons = yCons;

            problem.Objective = (y + 0.5);

            options = optimoptions('coneprog', 'Display', 'none');
            [solution, ~, status] = solve(problem, ...
                'Solver', 'coneprog', ...
                'Options', options);
        end

        function [solution, status, history] = refine(this, initialSolution)
            %REFINE Apply successive convexification to refine the inital
            %solution
            %   Detailed explanation goes here

            isSolved = false;
            numTries = 0;
            
            % Save initialization point
            prevSolution = initialSolution;
            prevSolution.nu = zeros(2, 1);
            history = [prevSolution];

            while ~isSolved && numTries < this.nMaxIters
                problem = optimproblem;

                % Apply boundary constraints
                r = optimvar('r', 3, this.nPoints, ...
                    "LowerBound", ones(3, this.nPoints) .* [this.xLim(1); this.yLim(1); this.zLim(1)], ...
                    "UpperBound", ones(3, this.nPoints) .* [this.xLim(2); this.yLim(2); this.zLim(2)]);
                v = optimvar('v', 3, this.nPoints);
                u = optimvar('u', 3, this.nPoints - 1);
    
                rBoundaryCons = optimconstr(3, 2);
                vBoundaryCons = optimconstr(3, 2);
                uBoundaryCons = optimconstr(3, 2);
    
                rBoundaryCons(:, 1) = r(:, 1) == this.rInitial;
                vBoundaryCons(:, 1) = v(:, 1) == this.vInitial;
                uBoundaryCons(:, 1) = u(:, 1) == this.gravity * this.e3;
    
                rBoundaryCons(:, end) = r(:, end) == this.rFinal;
                vBoundaryCons(:, end) = v(:, end) == this.vFinal;
                uBoundaryCons(:, end) = u(:, end) == this.gravity * this.e3;
                
                % Apply dynamic constraints
                rDynamicCons = optimconstr(3, this.nPoints - 1);
                vDynamicCons = optimconstr(3, this.nPoints - 1);
    
                dt = this.tFinal / this.nPoints;
                for t = 1 : this.nPoints - 1
                    rDynamicCons(:, t) = r(:, t+1) == r(:, t) + dt * v(:, t);
                    vDynamicCons(:, t) = v(:, t+1) == v(:, t) + dt * ( u(:, t) - this.gravity * this.e3 );
                end

                % Apply input constraints
                gamma = optimvar('gamma', 1, this.nPoints-1, ...
                    'LowerBound', this.uLim(1), ...
                    'UpperBound', this.uLim(2));
                uSlackCons = optimconstr(1, this.nPoints-1);
                uAngleCons = optimconstr(1, this.nPoints-1);
    
                for t = 1 : this.nPoints-1
                    uSlackCons(t) = norm(u(:, t)) <= gamma(t);
                    uAngleCons(t) = gamma(t) * cosd(this.thetaMax) <= this.e3' * u(:, t);
                end

                % Apply linearized obstacle constraints
                nu = optimvar('nu', this.nObstacles, ...
                    "LowerBound", 0);
                obsCollisionCons = optimconstr(this.nObstacles, this.nPoints);

                for j = 1 : this.nObstacles
                    p = this.obstacles(:, j);
                    
                    for t = 1 : this.nPoints
                        rPrev = prevSolution.r(:, t);
                        rCur = r(:, t);

                        deltaR = rPrev - p;
                        delR = rCur - rPrev;

                        xi = norm(this.H * deltaR);
                        zeta = this.H' * this.H * deltaR / (xi + 1e-8);

                        obsCollisionCons(j, t) = ...
                            xi + zeta' * delR >= this.R - nu(j);
                    end
                end

                problem.Constraints.rBoundaryCons = rBoundaryCons;
                problem.Constraints.vBoundaryCons = vBoundaryCons;
                problem.Constraints.uBoundaryCons = uBoundaryCons;
                problem.Constraints.rDynamicCons = rDynamicCons;
                problem.Constraints.vDynamicCons = vDynamicCons;
                problem.Constraints.uSlackCons = uSlackCons;
                problem.Constraints.uAngleCons = uAngleCons;
                problem.Constraints.obsCollisionCons = obsCollisionCons;

                % Convert quadratic cost to cone form
                y = optimvar('y');
                phi = [gamma, y]';

                H = 2 * this.w * dt * eye(length(gamma));
                A = sqrtm(H);
                Asc = blkdiag(A, 1);

                yCons = optimconstr(1);
                yCons(1) = norm(Asc * phi) <= (y+1);

                problem.Constraints.yCons = yCons;

                problem.Objective = (y + 0.5) + sum(nu, 'all');

                options = optimoptions('coneprog', 'Display', 'none');
                [solution, ~, status] = solve(problem, ...
                    'Solver', 'coneprog', ...
                    'Options', options);
                history = [history, solution];

                % Check convergence
                delta = norm(solution.r - prevSolution.r, 'inf');
                if delta < this.rTol && status == 1
                    isSolved = true;
                else
                    prevSolution = solution;
                    numTries = numTries + 1;
                end
            end

            % Postprocess before output
            status = status && isSolved;

        end

    end
end

