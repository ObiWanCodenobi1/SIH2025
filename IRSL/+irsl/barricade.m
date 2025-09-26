function [scenario, egoVehicle] = Barricade(barricades)
% Barricade: Simulation with one car and static barricades
% INPUT: barricades -> Nx2 array of [x, y] positions
% Road length: 100 m, 2 lanes, slightly wider for better view
% Displays pathline, car speed, and barricade coordinates
% The car stops if it collides with a barricade

%% Road parameters
roadLength = 100;  % meters
roadWidth = 10;    % total width for 2 lanes
scenario = drivingScenario('SampleTime', 0.04);

%% Road (horizontal, origin at bottom-left) - 2 lanes
road(scenario, [0 0 0; roadLength 0 0], 'Lanes', lanespec([2 2]), 'Name', 'Road');

%% Barricade bounds check
numBarr = size(barricades,1);
for k = 1:numBarr
    x = barricades(k,1);
    y = barricades(k,2);
    if x < 0 || x > roadLength || y < -roadWidth/2 || y > roadWidth/2
        error('🚫 Barricade %d at (%.1f, %.1f) is outside the road limits!', k, x, y);
    end
end

%% Ego vehicle
egoVehicle = vehicle(scenario, 'ClassID',1,'Position',[0 0 0], ...
    'Mesh', driving.scenario.carMesh, 'Name','EgoVehicle');

%% Car trajectory (straight horizontal)
numPoints = 200;
xWaypoints = linspace(0, roadLength, numPoints)';
yWaypoints = zeros(numPoints,1);
zWaypoints = zeros(numPoints,1);
waypointsEgo = [xWaypoints yWaypoints zWaypoints];
speedEgo = 10*ones(numPoints,1);  % constant 10 m/s
smoothTrajectory(egoVehicle, waypointsEgo, speedEgo);

%% Figure & plot scenario
figure;
hPlot = axes;
plot(scenario,'Parent',hPlot); 
hold on;

% Zoom to fit road
xlim([0 roadLength]);
ylim([-roadWidth roadWidth]);

% Draw barricades as rectangles (black)
barSize = [2 2]; % length x width
hBarr = gobjects(numBarr,1);
for k = 1:numBarr
    x = barricades(k,1);
    y = barricades(k,2);
    hBarr(k) = rectangle('Position',[x-barSize(1)/2, y-barSize(2)/2, barSize(1), barSize(2)],...
        'FaceColor',[0 0 0],'EdgeColor','k');  % black fill, black edge
end

% Pathline for ego vehicle (bright blue)
hPath = plot(nan,nan,'b-','LineWidth',2,'DisplayName','Ego Path');
hCar = plot(nan,nan,'bo','MarkerFaceColor','b','MarkerSize',6);  % blue marker for current car

% Speed display (right side)
speedText = annotation('textbox',[0.85 0.8 0.12 0.05],...
    'String','Speed: 0 m/s','EdgeColor','none','Color',[1 1 1],...
    'FontSize',10,'FontWeight','bold');

% Barricade coordinates display (left side)
barTextStr = strings(numBarr,1);
for k = 1:numBarr
    barTextStr(k) = sprintf('Bar%d: (%.1f, %.1f)', k, barricades(k,1), barricades(k,2));
end
barText = annotation('textbox',[0.02 0.8 0.25 0.05*numBarr],...
    'String', strjoin(barTextStr,'\n'),...
    'EdgeColor','none','Color',[1 1 1],...  % white text
    'FontSize',9,'FontWeight','normal');

%% Simulation loop
egoLength = 4.5; egoWidth = 2.0;
maxSteps = 500;

for i = 1:maxSteps
    if ~scenario.IsRunning
        break;
    end

    advance(scenario);
    pause(0.05);

    % Current position
    egoPos = egoVehicle.Position(1:2);

    % Update pathline
    hPath.XData(end+1) = egoPos(1);
    hPath.YData(end+1) = egoPos(2);

    % Update blue marker for current car position
    hCar.XData = egoPos(1);
    hCar.YData = egoPos(2);

    % Update speed display
    egoVel = norm(egoVehicle.Velocity);
    speedText.String = sprintf('Ego: %.1f m/s', egoVel);

    % Collision check with barricades
    for k = 1:numBarr
        barPos = barricades(k,:);
        if abs(egoPos(1)-barPos(1)) <= (egoLength+barSize(1))/2 && ...
           abs(egoPos(2)-barPos(2)) <= (egoWidth+barSize(2))/2
            disp('⚠️ Collision with barricade! Simulation stopped.');
            return
        end
    end

    % Stop at end of road
    if egoPos(1) >= roadLength
        disp('✅ Ego Vehicle reached end of road. Simulation finished.');
        break;
    end
end
end
