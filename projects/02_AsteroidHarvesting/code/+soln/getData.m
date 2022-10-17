function [df] = getData(path)
%GETDATA Summary of this function goes here
%   Detailed explanation goes here
arguments
    path string = '/home/matt/Documents/Classes/AERO7970_TrajectoryOptimization/projects/02_AsteroidHarvesting/data/AsteroidDataFile.txt';
end

% Read table
df = readtable(path);

% Set names for table columns
varNames = {'ID'};
for i = 2 : 13
    name = "Station" + string(i-1);
    varNames{i} = convertStringsToChars(name);
end
df.Properties.VariableNames = varNames;

end

