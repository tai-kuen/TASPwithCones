function coneLines = detectAndUpdateCones(coneLocations, detectedConeLocations, robot, coneLines, knownMap)
    % Function to monitor for new cones and call greedyNearestNeighbor when a new cone is detected
    
    i = 1;
    detectionRange = 5;  % Detection range for cones
    newConeDetected = false;  % Flag to track new cone detection
    
    while i <= size(coneLocations, 1)
        conePos = coneLocations(i, :);
        distanceToCone = norm(conePos - robot.position);
    
        if distanceToCone <= detectionRange  % If cone is within the detection range
            % Check if this cone has already been detected
            if isempty(detectedConeLocations) || ~ismember(conePos, detectedConeLocations, 'rows')
                % Add new detectedConeLocations
                detectedConeLocations = [detectedConeLocations; conePos];
                newConeDetected = true;  % Mark that a new cone was detected

                % Plot the detected cone
                plot(conePos(1), conePos(2), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', [1, 0.5, 0]);
            end
        end

        i = i + 1;  % Continue to the next cone
    end

    % Only call the greedyNearestNeighbor function if a new cone was detected
    if newConeDetected
        coneLines = greedyNearestNeighbor(detectedConeLocations, coneLines, knownMap);
    end
end