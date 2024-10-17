function coneLines = greedyNearestNeighbor(detectedConeLocations, coneLines, knownMap)
    % Function to detect cones and update the cone lines and map with the Greedy Nearest Neighbor algorithm,
    % allowing for two simultaneous current cones (front and back of the line), with one end expanding if only one can.

    visited = false(size(detectedConeLocations, 1), 1);  % Track visited cones
    circularRegionRadius = 2;  % Maximum distance to connect cones
    numPoints = 10;  % Number of points for marking lines as occupied

    while any(~visited)  % Continue until all cones are visited
        % Find the first unvisited cone
        idx = find(~visited, 1);
        currentConeFront = detectedConeLocations(idx, :);  % Initialize front cone
        currentConeBack = currentConeFront;  % Initialize back cone (same for now)
        visited(idx) = true;  % Mark first cone as visited
        newConeLine = currentConeFront;  % Initialize the new line
        
        % Expand the line from both front and back ends as long as any can expand
        while true
            % Get all unvisited cones
            unvisitedCones = detectedConeLocations(~visited, :);
            
            % Track whether any expansion happened in this iteration
            expansionOccurred = false;

            % Check for nearest cone to the front
            if ~isempty(unvisitedCones)
                distancesFront = vecnorm(unvisitedCones - currentConeFront, 2, 2);
                [minDistFront, minIdxFront] = min(distancesFront);
                
                % If a nearby unvisited cone is found for the front
                if minDistFront <= circularRegionRadius
                    nearestConeFront = unvisitedCones(minIdxFront, :);
                    % Mark the line between currentConeFront and nearestConeFront
                    for t = linspace(0, 1, numPoints)
                        intermediatePoint = (1 - t) * currentConeFront + t * nearestConeFront;
                        setOccupancy(knownMap, intermediatePoint, 1);  % Mark as occupied
                    end
                    % Plot the connection for front
                    plot([currentConeFront(1), nearestConeFront(1)], [currentConeFront(2), nearestConeFront(2)], 'Color', [1, 0.5, 0], 'LineWidth', 2);

                    % Add nearest cone to the front of the line
                    newConeLine = [nearestConeFront; newConeLine];  % Add to the beginning
                    
                    % Update the current front cone and mark it as visited
                    currentConeFront = nearestConeFront;
                    nearestConeIdxFront = find(all(detectedConeLocations == nearestConeFront, 2), 1);
                    visited(nearestConeIdxFront) = true;
                    expansionOccurred = true;  % Expansion happened at the front
                end
            end
            
            % Get all unvisited cones
            unvisitedCones = detectedConeLocations(~visited, :);

            % Check for nearest cone to the back
            if ~isempty(unvisitedCones)
                distancesBack = vecnorm(unvisitedCones - currentConeBack, 2, 2);
                [minDistBack, minIdxBack] = min(distancesBack);
                
                % If a nearby unvisited cone is found for the back
                if minDistBack <= circularRegionRadius
                    nearestConeBack = unvisitedCones(minIdxBack, :);
                    % Mark the line between currentConeBack and nearestConeBack
                    for t = linspace(0, 1, numPoints)
                        intermediatePoint = (1 - t) * currentConeBack + t * nearestConeBack;
                        setOccupancy(knownMap, intermediatePoint, 1);  % Mark as occupied
                    end
                    % Plot the connection for back
                    plot([currentConeBack(1), nearestConeBack(1)], [currentConeBack(2), nearestConeBack(2)], 'Color', [1, 0.5, 0], 'LineWidth', 2);

                    % Add nearest cone to the back of the line
                    newConeLine = [newConeLine; nearestConeBack];  % Add to the end
                    
                    % Update the current back cone and mark it as visited
                    currentConeBack = nearestConeBack;
                    nearestConeIdxBack = find(all(detectedConeLocations == nearestConeBack, 2), 1);
                    visited(nearestConeIdxBack) = true;
                    expansionOccurred = true;  % Expansion happened at the back
                end
            end
            
            % If neither front nor back can expand, stop expanding the current line
            if ~expansionOccurred
                break;
            end
        end
        
        % Add the completed cone line to the list
        coneLines{end + 1} = newConeLine;
    end

    % Post-process: Check if first and last cone of each line can be connected to form polygons
    for k = 1:length(coneLines)
        currentLine = coneLines{k};
        firstCone = currentLine(1, :);
        lastCone = currentLine(end, :);
        
        % If the first and last cones are close enough, connect them to close the polygon
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
