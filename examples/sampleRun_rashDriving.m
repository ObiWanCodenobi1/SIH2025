clear; close all; clc;
disp('--------------------------------------------------');
disp('Accelerating High-Fidelity Road Network Modeling');
disp('          for Indian Traffic Simulations         ');
disp('--------------------------------------------------');

% --- Simulation Parameters ---
NUM_VEHICLES = 100;
SIMULATION_TIME = 30;
TIME_STEP = 0.05;
rng('default');

% --- 1. Road Network Generation ---
disp('Generating 3D road network...');
road_network = createIndianRoadNetwork3D();
disp('Road network generated.');

% --- 2. Vehicle Initialization ---
disp('Initializing vehicles...');
vehicles = [];
for i = 1:NUM_VEHICLES
    
    start_node = randi(length(road_network.nodes));
    end_node = randi(length(road_network.nodes));
    while start_node == end_node
        end_node = randi(length(road_network.nodes));
    end
    
    % Use MATLAB's graph functions to find a path
    G = graph(road_network.edges(:,1), road_network.edges(:,2));
    path = shortestpath(G, start_node, end_node);
    
    if isempty(path) || length(path) < 2
       continue; % Skip if no path or path is too short
    end
    vehicles(i).id = i;
    vehicles(i).position = road_network.nodes(start_node, :);
    vehicles(i).path = path;
    vehicles(i).current_path_index = 1;
    vehicles(i).target_node = path(2);
    
    vehicles(i).velocity = [0 0 0];
    vehicles(i).speed = 0;
    vehicles(i).max_speed = 15 + rand*5; % m/s (approx 54-72 km/h)
    
    % Vehicle dimensions
    vehicles(i).length = 4.5;
    vehicles(i).width = 1.8;
    vehicles(i).height = 1.6;
    
    % Assign a unique driver persona and color
    [vehicles(i).persona, vehicles(i).color] = getDriverPersona();
end
disp('Vehicles initialized.');

% --- 3. Main Simulation Loop ---
disp('Starting simulation...');
tic; % Timer start
for t = 0:TIME_STEP:SIMULATION_TIME
    
    % Visualize current state
    visualizeSimulation3D(road_network, vehicles);

    % Update simulation title
    figure_handle = findobj('Type', 'figure', 'Name', 'Realistic 3D Indian Traffic Simulation');
    if isempty(figure_handle) || ~isvalid(figure_handle)
        disp('Simulation window closed by user.');
        break;
    end
    sgtitle(figure_handle, sprintf('Simulation Time: %.1f s', t));
    
    % Update each vehicle based on driver behavior & physics
    for i=1:length(vehicles)
        vehicles(i) = applyDriverBehavior(vehicles(i), vehicles, road_network);
        vehicles(i) = updateVehicleState(vehicles(i), road_network, TIME_STEP);
    end
    
end
toc;
disp('Simulation finished.');

function road_network = createIndianRoadNetwork3D()
    % Returns a set of hardcoded nodes and edges for a 3D Indian style road network
    
    nodes = [
        0, 0, 0;        % 1: Ground level intersection
        100, 0, 0;      % 2
        0, 100, 0;      % 3
        100, 100, 0;    % 4
        200, 0, 0;      % 5
        200, 100, 0;    % 6
        
        % Flyover / Overpass section
        100, -50, 10;   % 7: Flyover start
        100, 150, 10;   % 8: Flyover end
        
        % Side road
        -50, 100, 0;    % 9
    ];
    
    edges = [
        1, 2;
        1, 3;
        2, 4;
        3, 4;
        2, 5;
        4, 6;
        5, 6;
        
        2, 7;
        7, 8;
        8, 4;
        
        9, 3;
    ];
    
    road_network.nodes = nodes;
    road_network.edges = edges;
    road_network.num_lanes = 2;
end

function [persona, color] = getDriverPersona()
    r = rand();
    if r < 0.2
        persona.name = 'Aggressive';
        persona.speed_factor = 1.25;
        persona.following_dist_factor = 0.6;
        persona.lane_change_tendency = 0.1;
        color = [1, 0, 0];
    elseif r < 0.5
        persona.name = 'Cautious';
        persona.speed_factor = 0.8;
        persona.following_dist_factor = 1.5;
        persona.lane_change_tendency = 0.001;
        color = [0, 0, 1];
    elseif r < 0.9
        persona.name = 'Normal';
        persona.speed_factor = 1.0;
        persona.following_dist_factor = 1.0;
        persona.lane_change_tendency = 0.01;
        color = [0, 1, 0];
    else
        persona.name = 'Distracted';
        persona.speed_factor = 0.9 + rand()*0.2;
        persona.following_dist_factor = 1.2;
        persona.lane_change_tendency = 0.005;
        color = [1, 0.5, 0];
    end
end

function current_vehicle = applyDriverBehavior(current_vehicle, all_vehicles, road_network)
    if current_vehicle.current_path_index >= length(current_vehicle.path)
        return;
    end
    persona = current_vehicle.persona;
    target_speed = current_vehicle.max_speed * persona.speed_factor;
    
    MIN_DISTANCE = 5;
    safe_following_dist = MIN_DISTANCE * persona.following_dist_factor;
    
    vehicle_in_front = false;
    min_dist_to_front = inf;
    
    start_node_pos = road_network.nodes(current_vehicle.path(current_vehicle.current_path_index), :);
    target_node_pos = road_network.nodes(current_vehicle.target_node, :);
    my_direction = (target_node_pos - start_node_pos) ./ norm(target_node_pos - start_node_pos);
    if all(isnan(my_direction))
        my_direction = [1 0 0];
    end
    
    for i=1:length(all_vehicles)
        if current_vehicle.id == all_vehicles(i).id
            continue;
        end
        other_vehicle = all_vehicles(i);
        vec_to_other = other_vehicle.position - current_vehicle.position;
        dist_to_other = norm(vec_to_other);
        if dot(vec_to_other, my_direction) > 0.9 && dist_to_other < min_dist_to_front
            min_dist_to_front = dist_to_other;
            vehicle_in_front = true;
        end
    end
    
    if vehicle_in_front && min_dist_to_front < (safe_following_dist + 15)
        slowdown_factor = max(0, (min_dist_to_front - safe_following_dist) / 15);
        target_speed = target_speed * slowdown_factor;
    end
    
    current_vehicle.speed = target_speed;
end

function vehicle = updateVehicleState(vehicle, road_network, dt)
    if vehicle.current_path_index >= length(vehicle.path)
        return;
    end
    target_pos = road_network.nodes(vehicle.target_node, :);
    dist_to_target = norm(vehicle.position - target_pos);
    if dist_to_target < 5 
        vehicle.current_path_index = vehicle.current_path_index + 1;
        if vehicle.current_path_index >= length(vehicle.path)
            vehicle.speed = 0;
            vehicle.velocity = [0 0 0];
            return;
        else
            vehicle.target_node = vehicle.path(vehicle.current_path_index + 1);
            target_pos = road_network.nodes(vehicle.target_node, :);
        end
    end
    direction = target_pos - vehicle.position;
    if norm(direction) > 0
        direction = direction / norm(direction);
    else
        direction = [0 0 0];
    end
    current_speed = norm(vehicle.velocity);
    acceleration = (vehicle.speed - current_speed) * 0.5;
    new_speed = current_speed + acceleration * dt;
    vehicle.velocity = direction * new_speed;
    vehicle.position = vehicle.position + vehicle.velocity * dt;
end

function visualizeSimulation3D(road_network, vehicles)
    persistent h_fig h_axes h_light vehicle_patches;
    if isempty(h_fig) || ~isvalid(h_fig)
        h_fig = figure('Name', 'Realistic 3D Indian Traffic Simulation', 'NumberTitle', 'off', 'WindowState', 'maximized');
        h_axes = axes('Parent', h_fig, 'DataAspectRatio', [1 1 1], 'Color', [0.8 0.9 1]);
        hold(h_axes, 'on');
        grid(h_axes, 'on');
        xlabel(h_axes, 'X (m)');
        ylabel(h_axes, 'Y (m)');
        zlabel(h_axes, 'Z (m)');
        view(h_axes, -37.5, 30);
        
        plotRoads(h_axes, road_network);
        
        vehicle_patches = gobjects(length(vehicles), 1);
        for i=1:length(vehicles)
            vehicle_patches(i) = createCarModel(vehicles(i), h_axes);
        end
        
        h_light = camlight('headlight');
        lighting gouraud;
        material dull;
    end
    
    for i=1:length(vehicles)
        pos = vehicles(i).position;
        if vehicles(i).current_path_index >= length(vehicles(i).path)
            if length(vehicles(i).path) > 1
                start_node_index = vehicles(i).path(end-1);
                target_node_index = vehicles(i).path(end);
            else
                start_node_index = vehicles(i).path(end);
                target_node_index = vehicles(i).path(end);
            end
            start_node_pos = road_network.nodes(start_node_index, :);
            target_node_pos = road_network.nodes(target_node_index, :);
        else
            start_node_pos = road_network.nodes(vehicles(i).path(vehicles(i).current_path_index), :);
            target_node_pos = road_network.nodes(vehicles(i).target_node, :);
        end
        direction = target_node_pos - start_node_pos;
        if norm(direction) > 0
            direction = direction / norm(direction);
        else
            direction = [1 0 0];
        end
        angle = atan2(direction(2), direction(1));
        translation = makehgtform('translate', pos);
        rotation = makehgtform('zrotate', angle);
        if isvalid(vehicle_patches(i))
            set(vehicle_patches(i), 'Matrix', translation * rotation);
        end
    end
    
    if isvalid(h_light)
        camlight(h_light, 'headlight');
    end
    
    drawnow limitrate;
end

function plotRoads(h_axes, road_network)
    lane_width = 10;
    road_color = [0.4 0.4 0.4];
    
    for i=1:size(road_network.edges,1)
        node1 = road_network.nodes(road_network.edges(i,1), :);
        node2 = road_network.nodes(road_network.edges(i,2), :);
        road_vec = node2 - node1;
        if norm(road_vec) == 0; continue; end
        road_dir = road_vec / norm(road_vec);
        perp_vec = cross(road_dir, [0 0 1]);
        if norm(perp_vec) == 0; perp_vec = [0 1 0]; end
        perp_vec = perp_vec / norm(perp_vec);
        road_half_width = road_network.num_lanes * lane_width / 2;
        v1 = node1 - perp_vec * road_half_width;
        v2 = node1 + perp_vec * road_half_width;
        v3 = node2 + perp_vec * road_half_width;
        v4 = node2 - perp_vec * road_half_width;
        patch(h_axes, 'Vertices', [v1; v2; v3; v4], 'Faces', [1 2 3 4], ...
              'FaceColor', road_color, 'EdgeColor', 'none');
        if road_network.num_lanes > 1
            plot3(h_axes, [node1(1), node2(1)], [node1(2), node2(2)], [node1(3)+0.01, node2(3)+0.01], ...
                  'w--', 'LineWidth', 1.5);
        end
    end
end

function car_handle = createCarModel(vehicle, h_axes)
    l = vehicle.length;
    w = vehicle.width;
    h = vehicle.height;
    h_transform = hgtransform('Parent', h_axes);
    car_handle = h_transform;
    body_verts = [-l/2,-w/2,0; l/2,-w/2,0; l/2,w/2,0; -l/2,w/2,0; -l/2,-w/2,h*0.6; l/2,-w/2,h*0.6; l/2,w/2,h*0.6; -l/2,w/2,h*0.6];
    body_faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch('Vertices', body_verts, 'Faces', body_faces, 'FaceColor', vehicle.color, 'EdgeColor', 'k', 'Parent', h_transform);
    cabin_verts = [-l/4,-w/2*0.9,h*0.6; l/4,-w/2*0.9,h*0.6; l/4,w/2*0.9,h*0.6; -l/4,w/2*0.9,h*0.6; -l/5,-w/2*0.9,h; l/5,-w/2*0.9,h; l/5,w/2*0.9,h; -l/5,w/2*0.9,h];
    cabin_faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 5 6 7 8];
    patch('Vertices', cabin_verts, 'Faces', cabin_faces, 'FaceColor', [0.3 0.7 0.9], 'EdgeColor', 'k', 'Parent', h_transform);
end
