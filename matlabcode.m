clc; clear;
rosshutdown;
rosinit('http://10.21.195.237:11311');

disp('Waiting for map...');
mapSub = rossubscriber('/map', 'nav_msgs/OccupancyGrid');
mapData = receive(mapSub, 10);
disp('Map received.');

map = occupancyMap(readOccupancyGrid(mapData));
inflate(map, 0.25);

figure('Name', 'Warehouse Map', 'NumberTitle', 'off');
show(map); 
title('Warehouse Occupancy Grid (0.25m Inflation)');
drawnow;

fixedStart = getRandomValidPoint(map);
goal = getRandomValidGoal(map, fixedStart);
originalGoal = goal;

if ~isSafeStart(fixedStart, map, 0.15)
    error('❌ Start point is too close to obstacles.');
end
if ~isSafeStart(goal, map, 0.15)
    error('❌ Goal point is too close to obstacles.');
end

disp(['Start Position: [', num2str(fixedStart(1)), ', ', num2str(fixedStart(2)), ']']);
disp(['Goal Position: [', num2str(goal(1)), ', ', num2str(goal(2)), ']']);

disp('Planning optimal path using A* algorithm...');
pathToGoal = aStarPathPlanning(map, fixedStart, goal);

if isempty(pathToGoal)
    disp('⚠ A* path planning failed! Switching to Dijkstra...');
    pathToGoal = dijkstraPathPlanning(map, fixedStart, goal);
end

if isempty(pathToGoal)
    error('❌ No valid path could be found between start and goal.');
end

disp('Planning return path...');
pathToStart = aStarPathPlanning(map, goal, fixedStart);
if isempty(pathToStart)
    pathToStart = flip(pathToGoal);
end

pathToGoal = validatePath(pathToGoal, map);
pathToStart = validatePath(pathToStart, map);

if size(pathToGoal, 1) > 1
    initialYaw = atan2(pathToGoal(2,2)-pathToGoal(1,2), pathToGoal(2,1)-pathToGoal(1,1));
else
    initialYaw = 0;
end

teleportRobot(fixedStart, initialYaw);
pause(2);

figure('Name', 'Planned Path', 'NumberTitle', 'off');
show(map); hold on;
plot(fixedStart(1), fixedStart(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(pathToGoal(:,1), pathToGoal(:,2), 'b-', 'LineWidth', 2);
plot(pathToStart(:,1), pathToStart(:,2), 'm-', 'LineWidth', 2);
title('Validated Paths with 0.15m Clearance');
legend('Start', 'Goal', 'Path to Goal', 'Path to Start');
hold off;
drawnow;

disp('✅ Paths successfully planned and validated.');

robotPose = [fixedStart, initialYaw];

cmdPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
cmdMsg = rosmessage(cmdPub);
pause(1);

lidarSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
pause(1);

odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');
pause(1);

goalTolerance = 0.05;
controlRate = robotics.Rate(15);
minObstacleDistance = 0.05;
slowDownDistance = 0.75;
emergencyStopDistance = 0.025;

currentPhase = 'toGoal';
goalReached = false;
startReached = false;
replanCount = 0;
maxReplans = 5;
lastObstacleTime = 0;
recoveryState = 'normal';

pathFollowingError = [];
currentWaypointIndex = 1;

disp('🚀 Starting bidirectional navigation...');

navigationFig = figure('Name', 'Real-time Navigation', 'NumberTitle', 'off');

while (~goalReached || ~startReached) && ishandle(navigationFig)
    try
        odomMsg = receive(odomSub, 50);
        pose = odomMsg.Pose.Pose;
        robotPose(1) = pose.Position.X;
        robotPose(2) = pose.Position.Y;
        quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
        euler = quat2eul(quat);
        robotPose(3) = euler(1);
    catch
        warning('Could not get odometry, using estimated pose');
    end
    
    if strcmp(currentPhase, 'toGoal')
        targetPoint = originalGoal;
        currentPath = pathToGoal;
        distanceThreshold = goalTolerance;
    else
        targetPoint = fixedStart;
        currentPath = pathToStart;
        distanceThreshold = goalTolerance;
    end
    
    [currentWaypointIndex, distToPath] = updateWaypointIndex(robotPose, currentPath, currentWaypointIndex);
    pathFollowingError = [pathFollowingError; distToPath];
    
    distanceToTarget = norm(robotPose(1:2) - targetPoint);
    if distanceToTarget <= distanceThreshold
        if distanceToTarget > 0.02
            [v, omega] = getVelocityCommands(robotPose, currentPath, currentWaypointIndex, currentPhase);
            v = v * min(1, distanceToTarget/distanceThreshold);
            sendVelocityCommand(cmdPub, cmdMsg, v, omega);
        else
            stopRobot(cmdPub, cmdMsg);
            
            if strcmp(currentPhase, 'toGoal')
                disp('🎉 Goal successfully reached! Rotating to face return path...');
                goalReached = true;
                
                if size(pathToStart, 1) > 1
                    returnYaw = atan2(pathToStart(2,2)-pathToStart(1,2), pathToStart(2,1)-pathToStart(1,1));
                else
                    returnYaw = robotPose(3) + pi;
                end
                
                rotateToOrientation(cmdPub, cmdMsg, odomSub, returnYaw);
                
                odomMsg = receive(odomSub, 50);
                pose = odomMsg.Pose.Pose;
                robotPose(1) = pose.Position.X;
                robotPose(2) = pose.Position.Y;
                quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
                euler = quat2eul(quat);
                robotPose(3) = euler(1);
                
                currentWaypointIndex = 1;
                currentPhase = 'toStart';
                pause(1);
            else
                disp('🏁 Start position successfully reached! Navigation complete.');
                startReached = true;
                break;
            end
        end
        continue;
    end
    
    try
        scanData = receive(lidarSub, 50);
    catch
        warning('Failed to get LiDAR scan, retrying...');
        continue;
    end
    
    [emergencyStop, obstacleDetected, obstacleDistance, obstacleAngle] = ...
        detectObstacles(scanData, emergencyStopDistance, slowDownDistance);
    
    switch recoveryState
        case 'normal'
            [v, omega] = getVelocityCommands(robotPose, currentPath, currentWaypointIndex, currentPhase);
            
            if emergencyStop
                disp('🛑 EMERGENCY STOP! Obstacle too close');
                stopRobot(cmdPub, cmdMsg);
                recoveryState = 'avoiding';
                avoidanceStartTime = toc;
            elseif obstacleDetected
                if replanCount > 0
                    disp('⚠ Obstacle detected after replanning! Initiating new avoidance');
                    stopRobot(cmdPub, cmdMsg);
                    recoveryState = 'avoiding';
                    avoidanceStartTime = toc;
                else
                    [v, omega] = adjustForObstacle(v, omega, obstacleDistance, obstacleAngle);
                end
            end
            
            sendVelocityCommand(cmdPub, cmdMsg, v, omega);
            
        case 'avoiding'
            [v, omega] = avoidObstacleContinuous(obstacleAngle, obstacleDistance);
            sendVelocityCommand(cmdPub, cmdMsg, v, omega);
            
            if ~emergencyStop && ~obstacleDetected
                disp('✅ Obstacle cleared. Stopping to replan path...');
                stopRobot(cmdPub, cmdMsg);
                recoveryState = 'replanning';
                replanStartTime = toc;
            end
            
        case 'replanning'
            stopRobot(cmdPub, cmdMsg);
            
            if toc - replanStartTime > 1
                disp('🔄 Replanning path after avoidance...');
                
                if strcmp(currentPhase, 'toGoal')
                    newPath = replanPath(robotPose(1:2), originalGoal, map);
                    pathToGoal = newPath;
                else
                    newPath = replanPath(robotPose(1:2), fixedStart, map);
                    pathToStart = newPath;
                end
                
                if ~isempty(newPath)
                    currentWaypointIndex = 1;
                    
                    if size(newPath, 1) > 1
                        newYaw = atan2(newPath(2,2)-newPath(1,2), newPath(2,1)-newPath(1,1));
                    else
                        newYaw = robotPose(3);
                    end
                    
                    rotateToOrientation(cmdPub, cmdMsg, odomSub, newYaw);
                    robotPose(3) = newYaw;
                    
                    recoveryState = 'normal';
                    replanCount = replanCount + 1;
                    
                    figure(navigationFig);
                    clf; show(map); hold on;
                    plot(robotPose(1), robotPose(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
                    plot(originalGoal(1), originalGoal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
                    plot(fixedStart(1), fixedStart(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
                    
                    if strcmp(currentPhase, 'toGoal')
                        plot(pathToGoal(:,1), pathToGoal(:,2), 'b-', 'LineWidth', 2);
                        plot(pathToStart(:,1), pathToStart(:,2), 'm-', 'LineWidth', 1);
                    else
                        plot(pathToStart(:,1), pathToStart(:,2), 'm-', 'LineWidth', 2);
                        plot(pathToGoal(:,1), pathToGoal(:,2), 'b-', 'LineWidth', 1);
                    end
                    
                    title(sprintf('Replanned Path (Attempt %d)', replanCount));
                    hold off;
                    drawnow;
                    
                    pause(1);
                else
                    warning('Failed to find new path! Trying again...');
                    replanStartTime = toc;
                end
            end
    end
    
    if ishandle(navigationFig)
        figure(navigationFig);
        clf; show(map); hold on;
        
        plot(pathToGoal(:,1), pathToGoal(:,2), 'b-', 'LineWidth', 1.5);
        plot(pathToStart(:,1), pathToStart(:,2), 'm-', 'LineWidth', 1.5);
        plot(originalGoal(1), originalGoal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        plot(fixedStart(1), fixedStart(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
        
        if strcmp(currentPhase, 'toGoal')
            plot(pathToGoal(:,1), pathToGoal(:,2), 'c-', 'LineWidth', 2);
            plot(pathToGoal(currentWaypointIndex,1), pathToGoal(currentWaypointIndex,2), 'cx', 'MarkerSize', 15, 'LineWidth', 2);
        else
            plot(pathToStart(:,1), pathToStart(:,2), 'c-', 'LineWidth', 2);
            plot(pathToStart(currentWaypointIndex,1), pathToStart(currentWaypointIndex,2), 'mx', 'MarkerSize', 15, 'LineWidth', 2);
        end
        
        plot(robotPose(1), robotPose(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
        quiver(robotPose(1), robotPose(2), 0.2*cos(robotPose(3)), 0.2*sin(robotPose(3)), ...
              'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        text(0.05, 0.95, ['Phase: ', currentPhase], 'Units', 'normalized', 'Color', 'w', 'FontSize', 10);
        text(0.05, 0.90, ['State: ', recoveryState], 'Units', 'normalized', 'Color', 'w', 'FontSize', 10);
        text(0.05, 0.85, ['Distance to target: ', num2str(distanceToTarget, '%.2f'), 'm'], ...
             'Units', 'normalized', 'Color', 'w', 'FontSize', 10);
        text(0.05, 0.80, ['Waypoint: ', num2str(currentWaypointIndex), '/', num2str(size(currentPath,1))], ...
             'Units', 'normalized', 'Color', 'w', 'FontSize', 10);
        text(0.05, 0.75, ['Path error: ', num2str(distToPath, '%.3f'), 'm'], ...
             'Units', 'normalized', 'Color', 'w', 'FontSize', 10);
        
        title('Bidirectional Navigation');
        hold off;
        drawnow;
    end
    
    if replanCount >= maxReplans
        error('❌ Maximum path replan attempts reached. Navigation failed.');
    end
    
    waitfor(controlRate);
end

stopRobot(cmdPub, cmdMsg);
disp('Bidirectional navigation completed.');
disp(['Average path following error: ', num2str(mean(pathFollowingError), '%.3f'), 'm']);
disp(['Maximum path following error: ', num2str(max(pathFollowingError), '%.3f'), 'm']);
if ishandle(navigationFig)
    close(navigationFig);
end

function [v, omega] = getVelocityCommands(robotPose, path, currentWaypointIndex, phase)
    persistent lastWaypoint lastError integralError;
    
    if isempty(lastWaypoint)
        lastWaypoint = 1;
        lastError = 0;
        integralError = 0;
    end
    
    kP = 0.8;
    kI = 0.01;
    kD = 0.05;
    
    if strcmp(phase, 'toGoal')
        baseSpeed = 0.3;
        maxSpeed = 0.4;
        minSpeed = 0.05;
    else
        baseSpeed = 0.25;
        maxSpeed = 0.35;
        minSpeed = 0.05;
    end
    
    if currentWaypointIndex > size(path,1)
        currentWaypointIndex = size(path,1);
    end
    targetPoint = path(currentWaypointIndex,:);
    
    dx = targetPoint(1) - robotPose(1);
    dy = targetPoint(2) - robotPose(2);
    distanceToWaypoint = sqrt(dx^2 + dy^2);
    targetAngle = atan2(dy, dx);
    
    headingError = wrapToPi(targetAngle - robotPose(3));
    
    if currentWaypointIndex > 1
        pathVec = path(currentWaypointIndex,:) - path(currentWaypointIndex-1,:);
        robotVec = robotPose(1:2) - path(currentWaypointIndex-1,:);
        crossTrackError = (robotVec(1)*pathVec(2) - robotVec(2)*pathVec(1)) / norm(pathVec);
    else
        crossTrackError = 0;
    end
    
    if lastWaypoint ~= currentWaypointIndex
        integralError = 0;
    else
        integralError = integralError + headingError;
    end
    derivativeError = headingError - lastError;
    
    omega = kP * headingError + kI * integralError + kD * derivativeError;
    
    speedFactor = cos(headingError)^2;
    v = baseSpeed * speedFactor * min(1, distanceToWaypoint/0.5);
    
    v = max(min(v, maxSpeed), minSpeed);
    omega = max(min(omega, 1.5), -1.5);
    
    lastWaypoint = currentWaypointIndex;
    lastError = headingError;
end

function [currentIndex, distError] = updateWaypointIndex(robotPose, path, currentIndex)
    waypointThreshold = 0.3;
    maxLookahead = min(currentIndex + 5, size(path,1));
    
    distances = zeros(maxLookahead,1);
    for i = currentIndex:maxLookahead
        distances(i) = norm(robotPose(1:2) - path(i,:));
    end
    
    [minDist, closestIndex] = min(distances(currentIndex:maxLookahead));
    closestIndex = closestIndex + currentIndex - 1;
    
    if minDist < waypointThreshold && closestIndex < size(path,1)
        currentIndex = closestIndex + 1;
    else
        currentIndex = closestIndex;
    end
    
    if currentIndex > 1
        pathVec = path(currentIndex,:) - path(currentIndex-1,:);
        robotVec = robotPose(1:2) - path(currentIndex-1,:);
        distError = abs(robotVec(1)*pathVec(2) - robotVec(2)*pathVec(1)) / norm(pathVec);
    else
        distError = norm(robotPose(1:2) - path(1,:));
    end
end

function rotateToOrientation(cmdPub, cmdMsg, odomSub, targetYaw)
    angularTolerance = 0.05;
    maxAngularVel = 0.8;
    kP = 1.5;
    rate = robotics.Rate(15);
    
    disp(['Rotating to target orientation: ', num2str(targetYaw), ' radians']);
    
    odomMsg = receive(odomSub, 1);
    pose = odomMsg.Pose.Pose;
    quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
    euler = quat2eul(quat);
    currentYaw = euler(1);
    
    yawError = wrapToPi(targetYaw - currentYaw);
    
    while abs(yawError) > angularTolerance
        omega = kP * yawError;
        omega = sign(omega) * min(abs(omega), maxAngularVel);
        
        cmdMsg.Linear.X = 0;
        cmdMsg.Angular.Z = omega;
        send(cmdPub, cmdMsg);
        
        odomMsg = receive(odomSub, 30);
        pose = odomMsg.Pose.Pose;
        quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z];
        euler = quat2eul(quat);
        currentYaw = euler(1);
        
        yawError = wrapToPi(targetYaw - currentYaw);
        
        waitfor(rate);
    end
    
    stopRobot(cmdPub, cmdMsg);
    disp('Rotation completed.');
end

function [emergency, detected, distance, angle] = detectObstacles(scanData, emergencyDist, slowDist)
    detectionAngle = pi/3;
    minValidRange = 0.02;
    
    angles = linspace(scanData.AngleMin, scanData.AngleMax, numel(scanData.Ranges));
    
    frontScans = abs(angles) < detectionAngle/2;
    ranges = scanData.Ranges(frontScans);
    angles = angles(frontScans);
    
    valid = ranges > minValidRange & ranges < slowDist;
    ranges = ranges(valid);
    angles = angles(valid);
    
    if isempty(ranges)
        emergency = false;
        detected = false;
        distance = Inf;
        angle = 0;
        return;
    end
    
    [distance, idx] = min(ranges);
    angle = angles(idx);
    
    emergency = distance < emergencyDist;
    detected = distance < slowDist;
end

function [v, omega] = adjustForObstacle(v, omega, distance, angle)
    minDist = 0.05;
    maxDist = 0.75;
    
    danger = min(1, max(0, (maxDist - distance) / (maxDist - minDist)));
    
    v = v * (1 - danger * 0.8);
    
    omega = omega + 1.5 * danger * sign(angle) * (1 - abs(angle)/(pi/2));
    
    v = max(min(v, 0.3), 0);
    omega = max(min(omega, 1.5), -1.5);
end

function [v, omega] = avoidObstacleContinuous(obstacleAngle, obstacleDistance)
    baseSpeed = 0.15;
    turnGain = 1.5;
    minSafeDistance = 0.3;
    
    danger = max(0, (minSafeDistance - obstacleDistance) / minSafeDistance);
    
    v = baseSpeed * (1 - danger * 0.7);
    
    omega = turnGain * danger * sign(obstacleAngle) * (1 - abs(obstacleAngle)/(pi/2));
    
    v = max(min(v, 0.25), 0.05);
    omega = max(min(omega, 1.5), -1.5);
end

function path = aStarPathPlanning(map, start, goal)
    planner = plannerAStarGrid(map);
    startGrid = world2grid(map, start);
    goalGrid = world2grid(map, goal);
    pathIdx = plan(planner, startGrid, goalGrid);
    
    if isempty(pathIdx)
        path = [];
    else
        path = grid2world(map, pathIdx);
    end
end

function path = dijkstraPathPlanning(map, start, goal)
    startGrid = world2grid(map, start);
    goalGrid = world2grid(map, goal);
    
    gridSize = map.GridSize;
    width = gridSize(2);
    height = gridSize(1);
    
    dist = inf(height, width);
    dist(startGrid(2), startGrid(1)) = 0;
    visited = false(height, width);
    
    movements = [-1 -1; -1 0; -1 1;
                 0 -1;         0 1;
                 1 -1;  1 0;  1 1];
    
    queue = PriorityQueue();
    queue.insert([startGrid(2), startGrid(1)], 0);
    
    parent = zeros(height, width, 2);
    
    while ~queue.isEmpty()
        [current, ~] = queue.pop();
        
        if current(1) == goalGrid(2) && current(2) == goalGrid(1)
            break;
        end
        
        for m = 1:size(movements, 1)
            neighbor = current + movements(m, :);
            
            if neighbor(1) < 1 || neighbor(1) > height || neighbor(2) < 1 || neighbor(2) > width
                continue;
            end
            
            worldPos = grid2world(map, [neighbor(2), neighbor(1)]);
            if getOccupancy(map, worldPos) > 0.2 || visited(neighbor(1), neighbor(2))
                continue;
            end
            
            if norm(movements(m,:)) > 1
                moveCost = 1.4142;
            else
                moveCost = 1;
            end
            newDist = dist(current(1), current(2)) + moveCost;
            
            if newDist < dist(neighbor(1), neighbor(2))
                dist(neighbor(1), neighbor(2)) = newDist;
                parent(neighbor(1), neighbor(2), :) = current;
                queue.insert(neighbor, newDist);
            end
        end
        visited(current(1), current(2)) = true;
    end
    
    if isinf(dist(goalGrid(2), goalGrid(1)))
        path = [];
    else
        path = [];
        current = [goalGrid(2), goalGrid(1)];
        
        while ~isequal(current, [startGrid(2), startGrid(1)])
            path = [current; path];
            current = squeeze(parent(current(1), current(2), :))';
        end
        
        path = [startGrid; path];
        path = grid2world(map, path(:, [2, 1]));
    end
end

function path = replanPath(start, goal, map)
    path = aStarPathPlanning(map, start, goal);
    
    if isempty(path)
        disp('⚠ A* replanning failed, trying Dijkstra...');
        path = dijkstraPathPlanning(map, start, goal);
    end
    
    if ~isempty(path)
        path = validatePath(path, map);
    end
end

function safePath = validatePath(path, map)
    safePath = [];
    clearance = 0.15;
    
    for i = 1:size(path,1)
        if isSafeStart(path(i,:), map, clearance)
            safePath = [safePath; path(i,:)];
        else
            safePoint = findNearestSafePoint(path(i,:), map);
            safePath = [safePath; safePoint];
        end
    end
end

function point = getRandomValidPoint(map)
    occupancyMatrix = getOccupancy(map);
    freeCells = find(occupancyMatrix < 0.2 & occupancyMatrix >= 0);
    
    if isempty(freeCells)
        error('❌ No valid free cells found in the map!');
    end
    
    [rows, cols] = ind2sub(size(occupancyMatrix), freeCells);
    worldCoords = grid2world(map, [cols, rows]);
    
    for i = randperm(size(worldCoords,1))
        if isSafeStart(worldCoords(i,:), map, 0.15)
            point = worldCoords(i,:);
            return;
        end
    end
    
    error('❌ Could not find a valid start point with required clearance.');
end

function goal = getRandomValidGoal(map, start)
    maxAttempts = 1000;
    minDistance = 2.0;
    
    for attempt = 1:maxAttempts
        goal = getRandomValidPoint(map);
        if norm(start - goal) >= minDistance
            return;
        end
    end
    
    error('❌ Could not find a valid goal position far enough from start.');
end

function safe = isSafeStart(point, map, clearance)
    gridPoint = world2grid(map, point);
    radius = ceil(clearance * map.Resolution);
    
    [X, Y] = meshgrid(-radius:radius, -radius:radius);
    neighbors = gridPoint + [X(:), Y(:)];
    
    for j = 1:size(neighbors,1)
        if any(neighbors(j,:) < 1) || neighbors(j,1) > map.GridSize(2) || neighbors(j,2) > map.GridSize(1)
            safe = false;
            return;
        end
        
        if getOccupancy(map, grid2world(map, neighbors(j,:))) > 0.2
            safe = false;
            return;
        end
    end
    safe = true;
end

function safePoint = findNearestSafePoint(point, map)
    maxRadius = 15;
    gridPoint = world2grid(map, point);
    
    for r = 1:maxRadius
        [X,Y] = meshgrid(-r:r, -r:r);
        mask = (X.^2 + Y.^2) <= r^2;
        neighbors = gridPoint + [X(mask), Y(mask)];
        
        for j = 1:size(neighbors,1)
            if neighbors(j,1) >= 1 && neighbors(j,1) <= map.GridSize(2) && ...
               neighbors(j,2) >= 1 && neighbors(j,2) <= map.GridSize(1)
                
                worldPos = grid2world(map, neighbors(j,:));
                
                if isSafeStart(worldPos, map, 0.1)
                    safePoint = worldPos;
                    return;
                end
            end
        end
    end
    
    error('❌ Could not find safe point near the specified location!');
end

function stopRobot(pub, msg)
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub, msg);
    pause(0.1);
end

function sendVelocityCommand(pub, msg, v, omega)
    maxLin = 0.5;
    maxAng = 2.0;
    
    v = max(min(v, maxLin), -maxLin*0.5);
    omega = max(min(omega, maxAng), -maxAng);
    
    msg.Linear.X = v;
    msg.Angular.Z = omega;
    send(pub, msg);
end

function teleportRobot(position, yaw)
    setStateClient = rossvcclient('/gazebo/set_model_state');
    setStateMsg = rosmessage(setStateClient);
    
    setStateMsg.ModelState.ModelName = 'turtlebot3_burger';
    
    setStateMsg.ModelState.Pose.Position.X = position(1);
    setStateMsg.ModelState.Pose.Position.Y = position(2);
    setStateMsg.ModelState.Pose.Position.Z = 0.05;
    
    q = eul2quat([yaw, 0, 0]);
    
    setStateMsg.ModelState.Pose.Orientation.W = q(1);
    setStateMsg.ModelState.Pose.Orientation.X = q(2);
    setStateMsg.ModelState.Pose.Orientation.Y = q(3);
    setStateMsg.ModelState.Pose.Orientation.Z = q(4);
    
    setStateMsg.ModelState.Twist.Linear.X = 0;
    setStateMsg.ModelState.Twist.Linear.Y = 0;
    setStateMsg.ModelState.Twist.Linear.Z = 0;
    setStateMsg.ModelState.Twist.Angular.X = 0;
    setStateMsg.ModelState.Twist.Angular.Y = 0;
    setStateMsg.ModelState.Twist.Angular.Z = 0;
    
    setStateMsg.ModelState.ReferenceFrame = '';
    
    call(setStateClient, setStateMsg);
    
    pause(0.5);
end