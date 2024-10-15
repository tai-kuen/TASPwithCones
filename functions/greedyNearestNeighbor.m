function coneLines = greedyNearestNeighbor(detectedConeLocations, coneLines, knownMap)
    % Function to detect cones and update the cone lines and map with the Greedy Nearest Neighbor algorithm
    
    visited = zeros(size(detectedConeLocations, 1), 1);  % Array to track visited cones
    circularRegionRadius = 2;  % Example maximum connection distance
    numPoints = 10;  % Number of points used to mark lines as occupied
    
    % Main loop to detect and connect cones
    % Ensure visited has the same number of elements as detectedConeLocations
    visited = false(size(detectedConeLocations, 1), 1);
    
    while any(~visited)  % Continue until all cones are visited
        % Find the first unvisited cone
        idx = find(~visited, 1);
        currentCone = detectedConeLocations(idx, :);
        visited(idx) = true;  % Mark it as visited
        newConeLine = currentCone;  % Initialize a new line
        
        % Iteratively connect nearest neighbors within the circularRegionRadius
        while true
            % Get all unvisited cones
            unvisitedCones = detectedConeLocations(~visited, :);
            
            if ~isempty(unvisitedCones)  % Check if there are any unvisited cones
                % Compute distances between currentCone and all unvisited cones
                distances = vecnorm(unvisitedCones - currentCone, 2, 2);  % Correct array operation
                
                % Find the minimum distance and its index
                [minDist, minIdx] = min(distances);
                
                % Check if the nearest unvisited cone is within the circularRegionRadius
                if minDist <= circularRegionRadius
                    % Directly get the nearest cone without extra indexing
                    nearestCone = unvisitedCones(minIdx, :);
                    
                    % Mark the line between currentCone and nearestCone as occupied
                    for t = linspace(0, 1, numPoints)
                        intermediatePoint = (1 - t) * currentCone + t * nearestCone;
                        setOccupancy(knownMap, intermediatePoint, 1);  % Mark as occupied in the map
                    end
                    
                    % Plot the connection line
                    plot([currentCone(1), nearestCone(1)], [currentCone(2), nearestCone(2)], 'Color', [1, 0.5, 0], 'LineWidth', 2);
                    
                    % Add the nearest cone to the line and mark it as visited
                    newConeLine = [newConeLine; nearestCone];
                    
                    % Find the original index of nearestCone in detectedConeLocations
                    nearestConeIdx = find(all(detectedConeLocations == nearestCone, 2), 1);
                    visited(nearestConeIdx) = true;
                    
                    % Update current cone to the nearest one
                    currentCone = nearestCone;
                else
                    % No nearby unvisited cone, finish the current line
                    break;
                end
            else
                % All cones were visited
                break;
            end
        end
        
        % Add the new line to coneLines
        coneLines{end + 1} = newConeLine;
    end
    
    % Post-process: Check if the first and last cone of each line can be connected to form polygons
    for k = 1:length(coneLines)
        currentLine = coneLines{k};
        firstCone = currentLine(1, :);
        lastCone = currentLine(end, :);
        
        % If the first and last cone are close enough, connect them to close the polygon
        if norm(firstCone - lastCone) <= circularRegionRadius
            for t = linspace(0, 1, numPoints)
                intermediatePoint = (1 - t) * firstCone + t * lastCone;
                setOccupancy(knownMap, intermediatePoint, 1);  % Mark as occupied in the map
            end
            
            % Plot the closing connection
            plot([firstCone(1), lastCone(1)], [firstCone(2), lastCone(2)], 'Color', [1, 0.5, 0], 'LineWidth', 2);
        end
    end
end
