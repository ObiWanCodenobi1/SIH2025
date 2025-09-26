function [scenario, egoVehicle] = RashDriving()
% RashDriving: horizontal ego vehicle with smooth random curve
% Other vehicle: vertical straight path with variable speed
% Stops on collision or when other vehicle reaches destination

%% Create scenario
scenario = drivingScenario('SampleTime', 0.04);

%% Roads
road(scenario, [150 24.2 0; 54 24.2 0], 'Lanes', lanespec([2 2]), 'Name', 'Road');   % horizontal
road(scenario, [105 75 0; 105 -20 0], 'Lanes', lanespec([2 2]), 'Name', 'Road1');  % vertical

%% Ego vehicle (horizontal, smooth random curve)
egoVehicle = vehicle(scenario, 'ClassID',1,'Position',[56 24.2 0], ...
    'Mesh', driving.scenario.carMesh, 'Name','egoVehicle');

numControlPoints = 8;   % more control points = higher curvature
xControl = linspace(56, 135, numControlPoints);
laneCenterY = 24.2;
laneHalfWidth = 2.0;    % max lateral deviation

rng('shuffle');
yControl = laneCenterY + (rand(1,numControlPoints)-0.5)*2*laneHalfWidth; % random within lane

% Spline interpolation for smooth random curve
numPoints = 300;  
xInterp = linspace(56, 135, numPoints);
yInterp = spline(xControl, yControl, xInterp)';

zInterp = zeros(numPoints,1);
waypointsEgo = [xInterp' yInterp zInterp];

% Constant speed (randomized for each run)
speedEgo = (5 + 10*rand)*ones(numPoints,1);

smoothTrajectory(egoVehicle, waypointsEgo, speedEgo);

%% Other vehicle (vertical, straight) with variable speed
otherVehicle = vehicle(scenario, 'ClassID',1,'Position',[105 73 0], ...
    'Mesh', driving.scenario.carMesh, 'Name','otherVehicle');

waypointsOther = [105 73 0; 105 -20 0];  % straight vertical
speedOther = 8 + 4*rand(2,1);  % speed between 8-12 m/s
smoothTrajectory(otherVehicle, waypointsOther, speedOther);

%% Vehicle sizes for bounding-box collision (approx)
egoLength = 4.5; egoWidth = 2.0;       % horizontal car
otherLength = 4.5; otherWidth = 2.0;   % vertical car

%% Run simulation with bounding-box collision detection
figure; plot(scenario);

maxSteps = 500;

for i = 1:maxSteps
    % Stop simulation if already ended
    if ~scenario.IsRunning
        break;
    end

    advance(scenario);
    pause(0.05);

    % Get current positions (X,Y)
    egoPos = egoVehicle.Position(end,1:2);
    otherPos = otherVehicle.Position(end,1:2);

    % Bounding-box collision check
    if abs(egoPos(1) - otherPos(1)) <= (egoLength+otherLength)/2 && ...
       abs(egoPos(2) - otherPos(2)) <= (egoWidth + otherWidth)/2
        disp('⚠️ Collision detected! Simulation stopped.');
        break;
    end

    % Stop if other vehicle reaches final waypoint
    if norm(otherPos - [105 -20]) < 0.5
        disp('✅ Other vehicle reached destination. Simulation finished.');
        break;
    end
end

end
