% Function to plan the path
function path = planPath(robot, nextDestination, unknownMap)
    % Get the occupancy grid from the occupancyMap
    grid = getOccupancy(unknownMap);  % Get the occupancy grid (probability values)
    
    % Convert to binary occupancy map: threshold values to create binary map
    binaryGrid = grid > unknownMap.DefaultValue;  % Assuming occupancy probability > 0.5 is occupied

    % Define cell size (you may adjust this based on your map)
    cellSize = 10;  % Adjust according to your occupancy map settings
    
    % Create binaryOccupancyMap from the binary grid
    binaryMap = binaryOccupancyMap(binaryGrid, cellSize);
    inflate(binaryMap, 0.3);  % Inflate map for robot safety margin
    planner = plannerAStarGrid(binaryMap);
    
    % Convert robot pose and next destination to grid coordinates
    startGrid = world2grid(binaryMap, robot.position);
    goalGrid = world2grid(binaryMap, nextDestination);
    
    setOccupancy(binaryMap, startGrid, 0, 'grid')

    % Plan the path
    path = plan(planner, startGrid, goalGrid);
    
    % If no path is found, return empty
    if isempty(path)
        path = [];
    else
        % Convert grid path to world coordinates
        path = grid2world(binaryMap, path);
    end
end
