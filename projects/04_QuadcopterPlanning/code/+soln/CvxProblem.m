classdef CvxProblem < soln.ObstacleAvoidanceProblem
    %CVXPROBLEM Summary of this class goes here
    %   Detailed explanation goes here

    properties
        name = "Cvx"
    end
    
    methods
        function obj = CvxProblem()
            %CVXPROBLEM Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function [solution, status] = initialize(this)
            %INITIALIZE Solve the initial relaxed problem
            %   Detailed explanation goes here
            
            dt = this.tFinal / this.nPoints;

            % Form problem
            cvx_begin quiet
                % Declare variables
                variable R(3, this.nPoints);
                variable V(3, this.nPoints);
                variable U(3, this.nPoints-1);
                variable Gamma(this.nPoints-1);

                % Define problem
                minimize( this.w * sum_square(Gamma) * dt )
                subject to
                    % Box constraints
                    R(1,:) >= this.xLim(1);
                    R(2,:) >= this.yLim(1);
                    R(3,:) >= this.zLim(1);

                    R(1,:) <= this.xLim(2);
                    R(2,:) <= this.yLim(2);
                    R(3,:) <= this.zLim(2);

                    % Boundary conditions
                    R(:, 1) == this.rInitial;
                    R(:, this.nPoints) == this.rFinal;

                    V(:, 1) == this.vInitial;
                    V(:, this.nPoints) == this.vFinal;

                    U(:, 1) == this.gravity * this.e3;
                    U(:, this.nPoints-1) == this.gravity * this.e3;
                    
                    % Dynamic constraints
                    for i = 1 : this.nPoints-1
                        R(:, i+1) == R(:, i) + dt * V(:, i);
                        V(:, i+1) == V(:, i) + dt * (U(:, i) - this.gravity * this.e3);
                    end

                    % Input constraints
                    Gamma(:) >= this.uLim(1);
                    Gamma(:) <= this.uLim(2);

                    for i = 1 : this.nPoints-1
                        norm(U(:, i)) <= Gamma(i);
                        Gamma(i) * cosd(this.thetaMax) <= this.e3' * U(:, i);
                    end

            cvx_end

            % Postprocess
            status = strcmp(cvx_status, 'Solved');

            solution.r = R;
            solution.v = V;
            solution.u = U;
        end

        function [solution, status, history] = refine(this, initialSolution)
            %REFINE Apply successive convexification to refine the inital
            %solution
            %   Detailed explanation goes here

            isSolved = false;
            numTries = 0;
            dt = this.tFinal / this.nPoints;
            
            % Save initialization point
            prevSolution = initialSolution;
            history = [prevSolution];

            while ~isSolved && numTries < this.nMaxIters
                % Form problem
                cvx_begin quiet
                    % Declare variables
                    variable R(3, this.nPoints);
                    variable V(3, this.nPoints);
                    variable U(3, this.nPoints-1);
                    variable Gamma(this.nPoints-1);
                    variable Nu(this.nObstacles);
    
                    % Define problem
                    minimize( this.w * sum_square(Gamma) * dt + sum(Nu) )
                    subject to
                        % Box constraints
                        R(1,:) >= this.xLim(1);
                        R(2,:) >= this.yLim(1);
                        R(3,:) >= this.zLim(1);
    
                        R(1,:) <= this.xLim(2);
                        R(2,:) <= this.yLim(2);
                        R(3,:) <= this.zLim(2);
    
                        % Boundary conditions
                        R(:, 1) == this.rInitial;
                        R(:, this.nPoints) == this.rFinal;
    
                        V(:, 1) == this.vInitial;
                        V(:, this.nPoints) == this.vFinal;
    
                        U(:, 1) == this.gravity * this.e3;
                        U(:, this.nPoints-1) == this.gravity * this.e3;
                        
                        % Dynamic constraints
                        for i = 1 : this.nPoints-1
                            R(:, i+1) == R(:, i) + dt * V(:, i);
                            V(:, i+1) == V(:, i) + dt * (U(:, i) - this.gravity * this.e3);
                        end
    
                        % Input constraints
                        Gamma(:) >= this.uLim(1);
                        Gamma(:) <= this.uLim(2);
    
                        for i = 1 : this.nPoints-1
                            norm(U(:, i)) <= Gamma(i);
                            Gamma(i) * cosd(this.thetaMax) <= this.e3' * U(:, i);
                        end

                        % Linearized obstacle constraints
                        Nu >= 0;

                        for j = 1 : this.nObstacles
                            p = this.obstacles(:, j);
                            for t = 1 : this.nPoints
                                rPrev = prevSolution.r(:, t);
                                rCur = R(:, t);

                                deltaR = rPrev - p;
                                delR = rCur - rPrev;

                                xi = norm(this.H * deltaR);
                                zeta = this.H' * this.H * deltaR / (xi + 1e-8);

                                xi + zeta' * delR >= this.R - Nu(j);
                            end
                        end
                cvx_end

                % Build output structs
                solution.r = R;
                solution.v = V;
                solution.u = U;

                status = strcmp(cvx_status, 'Solved');

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

            % Postprocess
            status = status && isSolved;
        end
    end
end

