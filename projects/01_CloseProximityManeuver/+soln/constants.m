%{
Define constants for the problem
%}

% Gravitational parameter of the earth
MU = 398600; % km^3/s^2

% Radius of the eartGRAVh
RADIUS = 6378; % km

% Orbital altitude
ALTITUDE = 550; % km

% Mass of the chaser
MASS = 50; % kg

% Max thrust value
T_MAX = 200; % milliNewton

% Initial state
P_0 = [-1, 0, 0];
V_0 = [0, 0, 0];
X_0 = [P_0, V_0]';