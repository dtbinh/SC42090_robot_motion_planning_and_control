%% Test Script

% Tests the implemented (local) Dynamic Window Approach with a simple simulation.

more off;
startup;

close all;
clear all;

%% Define parameters
parameters.simTime = 5.0;
parameters.timestep = 0.1; 
parameters.nVelSamples = 11;        % should be uneven
parameters.nOmegaSamples = 11;      % should be uneven
parameters.robotRadius = 0.1;

% Scores 0
parameters.headingScoring = 0.1;
parameters.velocityScoring = 0.8;
parameters.obstacleDistanceScoring = 0.1;

% Scores 1
% parameters.headingScoring = 0.1;
% parameters.velocityScoring = 0.45;
% parameters.obstacleDistanceScoring = 0.45;

% Scores 2
% parameters.headingScoring = 0.1;
% parameters.velocityScoring = 0.15;
% parameters.obstacleDistanceScoring = 0.75;

assert(parameters.headingScoring+parameters.velocityScoring+parameters.obstacleDistanceScoring == 1,...
    'Sum of scorings must be unity')

parameters.objectiveFcnSmoothingKernel = fspecial('gaussian', [3,3], 1.0);
parameters.maxVel = 0.2;
parameters.maxOmega = pi;
parameters.maxAcc = 1.0;
parameters.maxOmegaDot = pi;
parameters.plot = false;
parameters.connectivity = 8;
parameters.goalBrakingDistance = 0.5;

% use the local DWA approach
parameters.globalPlanningOn = false;

% Goal 0 
% Define the goal position
% goalPosition.x = 3.0;
% goalPosition.y = 3.5;

% Goal 1
% goalPosition.x = 0;
% goalPosition.y = 0;

% Goal 2
goalPosition.x = 1;
goalPosition.y = 4;

%% Load a map
fileDir = fileparts(mfilename('fullpath'));
[ img ] = loadMapFromImage( [fileDir, '/../maps/simple_100x100.png'] );
map = createMap([-1.0, -0.5], 0.05, img);

%% Define start pose of the robot
% Start 0
% The Local DWA should be able to reach the goal starting at this pose
% x0 = 1.5;
% y0 = 1.5;
% h0 = 0.0;
% v0 = 0.0;
% omega0 = 0.0;

% Start 1
% Use this starting point instead (comment in) to see how the robot gets stuck
% x0 = 0.5;
% y0 = 2.0;
% h0 = 0.0;
% v0 = 0.0;
% omega0 = 0.0;

% Start 2
% x0 = -0.5;
% y0 = 3.5;
% h0 = 0.0;
% v0 = 0.0;
% omega0 = 0.0;

% Start 3
x0 = -0.5;
y0 = 0;
h0 = 0.0;
v0 = 0.0;
omega0 = 0.0;

% Initialize robot
robotState.x = x0;
robotState.y = y0;
robotState.heading = h0;
robotState.vel = v0;
robotState.omega = omega0;

%% plot the map
figure;
plotMap(map);
hold on;
plot(goalPosition.y, goalPosition.x, 'or', 'MarkerFaceColor', 'r');

%% Call the dynamic window approach (DWA) function
goalDist = hypot(robotState.y - goalPosition.y, robotState.x - goalPosition.x);
nSpeedZeroCnt = 0; % Let's count the number of successive zero robot speeds to detect whether we are stuck and then abort the simulation
robotIsStuck = 0;
handles = [];

bool_plot = false;
ii = 0;
clear x_hist y_hist;
while ~robotIsStuck
    ii = ii+1;
    % compute the commands
    [ v, omega, debug ] = dynamicWindowApproach( robotState, goalPosition, map, parameters );
    % update the robot pose (we assume that it perfectly executes the
    % commands)
    robotState = updateRobotState(robotState, v, omega, parameters.timestep);
    
    if(bool_plot)
        % plot the robot position
        plot(robotState.y, robotState.x, 'og', 'MarkerFaceColor', 'g');
        % plot the trajectory set
        if debug.valid
            handles = plotTrajectories(debug, handles);
        end   
        drawnow;
    else
        x_hist(ii) = robotState.x;
        y_hist(ii) = robotState.y;
    end
    robotState;
    
    goalDist = hypot(robotState.y - goalPosition.y, robotState.x - goalPosition.x);
    
    % Detect whether robot is stuck or has reached the goal
    if (robotState.vel < 1e-2)
        nSpeedZeroCnt = nSpeedZeroCnt + 1;
    else
        nSpeedZeroCnt = 0;
    end
    if nSpeedZeroCnt > 20
        robotIsStuck = 1;
    end
    if ii>1500 break; end
end

%% Plot trajectory
if(~bool_plot)
%     plot(y_hist(1), x_hist(1), 'ok', 'MarkerFaceColor', 'k'); hold on
    plot(y_hist, x_hist, '-r');     
end
% print(sprintf('figures/x0_-%f_y0_%f_h0_%f_xf_%f_yf_%f.eps',x0,y0,h0,goalPosition.x,goalPosition.y),'-depsc2');
print(sprintf('figures/s_heading_%f_s_velocity_%f_s_distance_%f.eps',...
    parameters.headingScoring,...
    parameters.velocityScoring,...
    parameters.obstacleDistanceScoring),'-depsc2');
