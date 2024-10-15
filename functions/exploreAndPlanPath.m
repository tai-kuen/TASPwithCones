function [plannedPath,cones,borderLines] = exploreAndPlanPath(robotPose, knownMap, unknownMap, maxrange, trajectoryX,trajectoryY,cones, borderLines)
    % EXPLOREANDPLANPATH Performs Lidar scan, frontier finding, clustering,
    % and path planning.
    %   Inputs:
    %       robotPose - The robot's current [x, y, theta] position
    %       knownMap - The known map where the robot operates
    %       unknownMap - The map where the robot is exploring (unknown space)
    %       maxrange - The maximum range of the Lidar sensor
    %
    %   Output:
    %       path - Planned path from current pose to the selected next destination
    
    % Perform Lidar scan in the known map
    % Add Lidar sensor to scan the known map
    lidar = rangeSensor;
    lidar.HorizontalAngle = [-pi pi];
    lidar.HorizontalAngleResolution = 0.01;
    lidar.Range = [0 30];
    [ranges, angles] = lidar(robotPose, knownMap);
    scan = lidarScan(ranges, angles);
    
    % Insert scan into the unknown map
    insertRay(unknownMap, robotPose, scan, maxrange);

    % Detect new cones in range
    i = 1;
    while i <= size(cones, 1)
        conePos = cones(i, :);
        distanceToCone = norm(conePos - robotPose(1:2));
    
        if distanceToCone <= 6
            % Check if cone's circular region contains any element of any borderLine
            circularRegionRadius = 2;  % Example radius for the circular region around the cone
            coneInCurrentBorderLine = false;
            closestBorderPos = [];
            minDistance = Inf;
    
            % Check all existing borderLines for proximity to the cone
            for k = 1:length(borderLines)
                currentBorderLine = borderLines{k};  % Get the current border line
    
                % Check each position in the current border line
                for j = 1:size(currentBorderLine, 1)
                    borderPos = currentBorderLine(j, :);
                    distanceToBorderPos = norm(borderPos - conePos);
    
                    if distanceToBorderPos <= circularRegionRadius
                        coneInCurrentBorderLine = true;
                    end
    
                    % Update the closest borderPos if this one is closer
                    if distanceToBorderPos < minDistance
                        minDistance = distanceToBorderPos;
                        closestBorderPos = borderPos;
                    end
                end
            end
    
            % If the cone is within proximity of the border line, mark the closest line as occupied
            if coneInCurrentBorderLine && ~isempty(closestBorderPos)
                % Mark the line between closestBorderPos and conePos as occupied
                numPoints = 10;  % Example number of points along the line
                for t = linspace(0, 1, numPoints)
                    intermediatePoint = (1 - t) * closestBorderPos + t * conePos;  % Compute intermediate points
    
                    % Add intermediatePoint as occupied
                    setOccupancy(knownMap, intermediatePoint, 1);
                end
    
                % Add the cone to the current borderLine
                borderLines{k} = [currentBorderLine; conePos];
            else
                % Create a new borderLine and assign conePos to it
                newBorderLine = conePos;  % Create a new borderLine entry
                borderLines{end + 1} = newBorderLine;  % Add the new borderLine to borderLines
            end
    
            % Remove the cone from the cones array
            cones(i, :) = [];  % Remove the current cone
        else
            i = i + 1;  % Increment i only if no cone is removed
        end
    end

    
    % Show the updated unknown map
    show(unknownMap);

    % Plot tracjectory
    h = plot(trajectoryX, trajectoryY, 'b-'); 
    % Draw the robot's start position
    plot(trajectoryX(1),trajectoryY(1), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    
    % Find frontier cells (boundary between explored and unexplored areas)
    frontierCells = findFrontier(unknownMap);
    
    % Cluster the frontier cells based on distance threshold
    distanceThreshold = 0.5;
    clusters = clusterFrontiers(frontierCells, distanceThreshold);
    
    % Plot the clustered frontiers
    colors = lines(length(clusters));  % Use MATLAB's lines colormap
    for clusterIdx = 1:length(clusters)
        clusterPoints = clusters{clusterIdx};  % Get points for the current cluster
        
        % Plot the points in the current cluster
        plot(clusterPoints(:, 1), clusterPoints(:, 2), 'o', 'MarkerSize', 1, ...
             'Color', colors(clusterIdx, :), 'MarkerFaceColor', colors(clusterIdx, :));
    end
    
    % Select the next destination (nearest cluster center)
    selectedClusterIdx = -1;
    [nextDestination, clusters, selectedClusterIdx] = selectNextDestination(clusters, robotPose(1:2),false,selectedClusterIdx);
    
    % Plot the next destination
    plot(nextDestination(1), nextDestination(2), 'ro', 'MarkerSize', 5);
    
    % Path planning function to find the path from current pose to the next destination
    plannedPath = planPath(robotPose, nextDestination, unknownMap);

    % Check if the path is empty
    if isempty(plannedPath)
        % Find another nearby cluster point if the path is empty
        [nextDestination, clusters, selectedClusterIdx] = selectNextDestination(clusters, robotPose(1:2),true,selectedClusterIdx);
        if ~isempty(nextDestination)
            % Recalculate the path to the new destination
            plannedPath = planPath(robotPose, nextDestination, unknownMap);
            % Optionally, plot the new destination
            plot(nextDestination(1), nextDestination(2), 'ro', 'MarkerSize', 5);
        else
            disp('No nearby cluster points available!'); 
            plannedPath = [];  % Set path to empty if no destination is found
        end
    end
    
    % Optionally, you can plot the path here as well
    plot(plannedPath(:, 1), plannedPath(:, 2), 'g-', 'LineWidth', 1.5);

    % Update the plot
    set(h, 'XData', trajectoryX, 'YData', trajectoryY);
    drawnow;
end
