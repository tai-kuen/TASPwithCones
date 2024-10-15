function [coneLocations, coneLines] = detectAndUpdateCones(coneLocations, robot, coneLines, knownMap)
    % Function to detect new cones in range and update the coneLines and map accordingly
    
    % Iterate through the cones
    i = 1;
    while i <= size(coneLocations, 1)
        conePos = coneLocations(i, :);
        distanceToCone = norm(conePos - robot.position);
    
        if distanceToCone <= 5  % If cone is within the detection range
            plot(conePos(1), conePos(2), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', [1, 0.5, 0]);

            % Define circular region radius around the cone
            circularRegionRadius = 2;  % Example radius for the circular region around the cone
            isConeCloseToCurrentConeLine = false;
            closestConeInConeLine = [];
            minDistance = Inf;
    
            % Check all existing coneLines for proximity to the cone
            for k = 1:length(coneLines)
                currentConeLine = coneLines{k};  % Get the current border line
    
                % Check each cone in the current cone line
                for j = 1:size(currentConeLine, 1)
                    coneInConeLine = currentConeLine(j, :);
                    distanceToBorderPos = norm(coneInConeLine - conePos);
    
                    % Check if the cone is near the current border line
                    if distanceToBorderPos <= circularRegionRadius
                        isConeCloseToCurrentConeLine = true;
                    end
    
                    % Find the closest border position to the cone
                    if distanceToBorderPos < minDistance
                        minDistance = distanceToBorderPos;
                        closestConeInConeLine = coneInConeLine;
                    end
                end
            end
    
            % If cone is close to a cone line, mark the connected line as occupied
            if isConeCloseToCurrentConeLine && ~isempty(closestConeInConeLine)
                % Mark the line between closestConeInConeLine and conePos as occupied
                numPoints = 10;  % Number of intermediate points along the line
                for t = linspace(0, 1, numPoints)
                    intermediatePoint = (1 - t) * closestConeInConeLine + t * conePos;  % Compute intermediate points
                    
                    % Set these intermediate points as occupied in the known map
                    setOccupancy(knownMap, intermediatePoint, 1);
                end

                % Plot the connection line with orange color
                plot([closestConeInConeLine(1), conePos(1)], [closestConeInConeLine(2), conePos(2)], ...
                     'Color', [1, 0.5, 0], 'LineWidth', 2);
    
                % Add the cone to the current coneLine
                coneLines{k} = [currentConeLine; conePos];
            else
                % Create a new borderLine and assign conePos to it
                newBorderLine = conePos;  % New borderLine entry
                coneLines{end + 1} = newBorderLine;  % Add to coneLines
            end
    
            % Remove the detected cone from the cones array
            coneLocations(i, :) = [];  % Remove the current cone
        else
            i = i + 1;  % Increment index only if cone is not removed
        end
    end
end
