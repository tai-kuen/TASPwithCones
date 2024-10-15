function robot = moveAlongPlannedPath(robot, plannedPath, linearVelocityGain, angularVelocityGain, goalTolerance, timeStep)
    % Loop through each waypoint in the planned path
    for i = 1:size(plannedPath, 1)
        % Get the current waypoint from the planned path
        currentGoal = plannedPath(i, :);
        
        % Move towards the current waypoint until the robot is within goalTolerance
        while norm(robot.position - currentGoal) > goalTolerance
            % Calculate the distance and angle to the current waypoint
            deltaPos = currentGoal - robot.position;
            distanceToGoal = norm(deltaPos);
            angleToGoal = atan2(deltaPos(2), deltaPos(1)) - robot.orientation;

            % Ensure angle is within -pi to pi range
            angleToGoal = atan2(sin(angleToGoal), cos(angleToGoal));
            
            % Calculate the absolute turn angle
            turnAngle = abs(angleToGoal);

            % Set a threshold for determining if the robot is turning
            turnThreshold = pi / 4; % You can adjust this threshold
            
            % Adjust linear velocity based on the turn angle
            if turnAngle < turnThreshold
                % Maintain speed when going straight
                linearVelocity = linearVelocityGain * distanceToGoal; 
            else
                % Slow down when turning
                linearVelocity = linearVelocityGain * distanceToGoal * (turnThreshold / turnAngle);
                linearVelocity = max(0.1, linearVelocity); % Ensure linear velocity doesn't go below a threshold
            end
            
            angularVelocity = angularVelocityGain * angleToGoal;

            % Update robot position and orientation
            prevPosition = robot.position; % Store previous position
            robot.position = robot.position + [cos(robot.orientation), sin(robot.orientation)] * linearVelocity * timeStep;
            robot.orientation = robot.orientation + angularVelocity * timeStep;

            % Store the robot's trajectory
            robot.trajectory = [robot.trajectory; robot.position];

            % Pause to simulate real-time movement
            % Append only the new segment to the trajectory plot
            plot([prevPosition(1), robot.position(1)], [prevPosition(2), robot.position(2)], 'b-', 'LineWidth', 0.5); % Plot new segment
            drawnow;
        end
    end
end
