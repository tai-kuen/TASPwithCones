function TASPgoal = TASPpathPlanning(startPos, unknownMap, taspCellSize)
    TASPgoal = []; % TASPgoal to be returned

    % Declare BTP as persistent so it retains its value between function calls
    persistent BTP;
    persistent TASPtrajectory;
    persistent plotBTP;
    
    % Initialize BTP only once, during the first call to TASP
    if isempty(TASPtrajectory) || size(TASPtrajectory, 1) < 2
        TASPtrajectory = startPos(1:2); % start position of the robot as first trajectory
        TASPcurrentPos = TASPtrajectory(end,:);
        BTP = []; % Initialize BTP to an empty array
        plotBTP = []; 
        TASPprevPos = [startPos(1) - cos(startPos(3))*1,startPos(2) - sin(startPos(3))*1];
    else
        TASPcurrentPos = TASPtrajectory(end,:);
        TASPprevPos = TASPtrajectory(end-1,:);
    end
    frontCell = getFrontCell(TASPcurrentPos, TASPprevPos, taspCellSize);

    % Check surroundings for free cells and update BTP
    freeCells = getFreeCells(TASPcurrentPos, unknownMap, taspCellSize, TASPtrajectory);
    BTP = updateBTP(BTP, freeCells);

    % Update plotBTP only if BTP is not empty
    if ~isempty(BTP)
        % Delete the previous BTP plot if it exists
        if ~isempty(plotBTP)
            delete(plotBTP);
        end
        
        % Plot the updated BTP points and store the handle
        plotBTP = plot(BTP(:, 1), BTP(:, 2), 'bo', 'MarkerSize', 3);
    end
    
    % If no free cells, move to the closest BTP
    if isempty(freeCells)
        if isempty(BTP)
            disp('No BTP available. Stopping the robot.');
            return; % Stop if no BTP available
        end
        TASPtrajectory(end+1, :) =  findClosestBTP(TASPcurrentPos, BTP, startPos, unknownMap);

    % Check if the cell directly in front of the robot's current direction is free
    elseif ismember(frontCell, freeCells, 'rows')
        % The cell in front is free, move straight
        TASPtrajectory(end+1, :) = frontCell;

    % If only one free cell, move towards it
    elseif size(freeCells, 1) == 1
        TASPtrajectory(end+1, :) = freeCells(1,:);

    % If two free cells, choose the one that points away from starting position
    elseif size(freeCells, 1) == 2
        cell1 = freeCells(1,:);
        cell2 = freeCells(2,:);
        if isAwayFromStart(cell1, startPos, TASPcurrentPos) && ~isAwayFromStart(cell2, startPos, TASPcurrentPos)
            TASPtrajectory(end+1, :) = cell1;
        elseif ~isAwayFromStart(cell1, startPos, TASPcurrentPos) && isAwayFromStart(cell2, startPos, TASPcurrentPos)
            TASPtrajectory(end+1, :) = cell2;
        else
            % Both point away, select the one with longer straight motion
            if getDistanceToObstacle(TASPcurrentPos, cell1, unknownMap, taspCellSize) > getDistanceToObstacle(TASPcurrentPos, cell2, unknownMap, taspCellSize)
                TASPtrajectory(end+1, :) = cell1;
            else
                TASPtrajectory(end+1, :) = cell2;
            end
        end
    end

    % Find the row in BTP that matches the last position of trajectory
    rowToDelete = ismember(BTP, TASPtrajectory(end,:), 'rows');
    % Delete the matching row from BTP
    BTP(rowToDelete, :) = [];
    
    TASPgoal = TASPtrajectory(end,:);
end

function freeCells = getFreeCells(position, unknownMap, taspCellSize, TASPtrajectory)
    % Get the surrounding neighboring cells around the robot's current position
    neighbors = [position(1)-taspCellSize, position(2); 
                 position(1)+taspCellSize, position(2); 
                 position(1), position(2)-taspCellSize;
                 position(1), position(2)+taspCellSize];
    
    freeCells = [];
    halfSize = taspCellSize / 2; % Half size of the square area

    % Loop through each neighboring cell
    for i = 1:size(neighbors, 1)
        centerCell = neighbors(i, :);
        allFree = true; % Flag to check if all cells in the square area are free
        
        % Define the boundaries of the square area around the center cell
        xRange = centerCell(1) - halfSize: 1/unknownMap.Resolution : centerCell(1) + halfSize;
        yRange = centerCell(2) - halfSize: 1/unknownMap.Resolution : centerCell(2) + halfSize;

        % Loop through each cell in the square area centered at the neighbor
        for x = xRange
            for y = yRange
                currentCell = [x, y];
                % Check if the current cell is occupied
                if unknownMap.getOccupancy(currentCell) >= 0.5
                    allFree = false; % Set flag to false if any cell is occupied
                    break; % Exit the loop early if an occupied cell is found
                end
            end
            if ~allFree
                break; % Exit the outer loop if an occupied cell is found
            end
        end
        
        % If all cells in the square area are free, add the center cell to freeCells
        if allFree && ~ismember(centerCell, TASPtrajectory, 'rows')
            freeCells = [freeCells; centerCell]; % Add the center cell to free cells list
        end
    end
end



function BTP = updateBTP(BTP, freeCells)
    % Loop through the free cells and update BTP
    for i = 1:size(freeCells, 1)
        % Check if the free cell is already in BTP
        if ~isempty(BTP)
            if ~ismember(freeCells(i, :), BTP, 'rows')
                BTP = [BTP; freeCells(i, :)]; % Append free cell to BTP
            end
        else
            BTP = freeCells(i, :);
        end
    end
end

function closestBTP = findClosestBTP(TASPcurrentPos, BTP, startPos, unknownMap)
    % Initialize variables to store the minimum path distance and the corresponding BTP
    minPathDistance = inf;
    closestBTP = [];
    
    % Loop through each BTP to calculate the path distance from TASPcurrentPos
    for i = 1:size(BTP, 1)
        % Plan the path to the current BTP
        TASPcurrent.position = TASPcurrentPos;
        path = planPath(TASPcurrent, BTP(i, :), unknownMap);
        
        if ~isempty(path)
            % Calculate the path distance (sum of Euclidean distances between waypoints)
            pathDistance = sum(sqrt(sum(diff(path).^2, 2)));
            
            % Check if this is the shortest path so far
            if pathDistance < minPathDistance
                minPathDistance = pathDistance;
                closestBTP = BTP(i, :);
            elseif pathDistance == minPathDistance
                % If path distances are the same, select the BTP farthest from startPos
                startDistanceCurrent = norm(BTP(i, :) - startPos(1:2));
                startDistanceClosest = norm(closestBTP - startPos(1:2));
                if startDistanceCurrent > startDistanceClosest
                    closestBTP = BTP(i, :);  % Choose the one farther from startPos
                end
            end
        end
    end
end

function frontCell = getFrontCell(TASPcurrentPos, TASPprevPos, taspCellSize)
    % Calculate the movement direction
    direction = TASPcurrentPos - TASPprevPos;
    % Get the front cell based on the direction
    frontCell = TASPcurrentPos + direction / norm(direction) * taspCellSize;
end

function isAway = isAwayFromStart(cell, startPos, TASPcurrentPos)
    % Calculate the Euclidean distance of the cell from the starting position
    distCellToStart = norm(cell - startPos(1:2));
    
    % Calculate the Euclidean distance of the current position from the starting position
    distCurrentToStart = norm(TASPcurrentPos - startPos(1:2));
    
    % The cell is considered "away" if it is farther from the start than the current position
    isAway = distCellToStart > distCurrentToStart;
end

function distance = getDistanceToObstacle(TASPcurrentPos, cell, unknownMap, taspCellSize)
    % Calculate direction from current position to the cell
    direction = (cell - TASPcurrentPos) / norm(cell - TASPcurrentPos);  % Unit vector in the direction

    % Initialize distance to 0
    distance = 0;
    
    % Set the starting cell for distance calculation
    currentCell = cell;
    
    while true
        % Move in the calculated direction by one step size (taspCellSize)
        nextCell = currentCell + direction * taspCellSize;
        
        % Check the occupancy of the next cell in the unknownMap
        occupancyValue = unknownMap.getOccupancy(nextCell);
        
        % Stop if the cell is occupied or unexplored
        if occupancyValue >= 0.5
            break;
        end
        
        % Otherwise, continue moving and increase the distance
        distance = distance + taspCellSize;  % Increment the distance
        currentCell = nextCell;  % Update current cell to next cell
    end
end
