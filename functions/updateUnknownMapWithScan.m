function updateUnknownMapWithScan(robot, knownMap, unknownMap)
    % Create and configure the lidar sensor
    lidarRange = 8; % meter
    lidar = rangeSensor;
    lidar.HorizontalAngle = [-pi pi];                  % Full 360-degree scan
    lidar.HorizontalAngleResolution = 0.01;            % Set resolution of lidar scan
    lidar.Range = [0 30];                      % Lidar scanning range

    % Perform Lidar scan on the known map
    [ranges, angles] = lidar([robot.position, robot.orientation], knownMap);
    scan = lidarScan(ranges, angles);                  % Create lidarScan object
    
    % Insert the scan into the unknown map
    insertRay(unknownMap, [robot.position, robot.orientation], scan, lidarRange); % Insert rays into the unknown map
    
    % Visualize updated unknown map (optional)
    show(unknownMap, 'FastUpdate', true);
    drawnow;
end
