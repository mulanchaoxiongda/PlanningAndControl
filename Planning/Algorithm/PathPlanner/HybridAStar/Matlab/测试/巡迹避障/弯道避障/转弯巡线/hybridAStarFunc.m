%%混合A*算法--全局规划可尝试的优化方案：当相对距离大于20m时，搜索150次后暂停，从closedList中取无约束启发代价最小的节点作为阶段目标点，规划出一条路径片段，并从路径片段的起点开始顺推5m/searchStep个节点，作为下次规划的起点；以一定的频率，重复以上步骤，不断更新路径片段，直到车辆距离停车位小于20mi；相对距离小于20米后，可一次规划出到停车位的路径
function [refPath, openList, closedList] = hybridAStarFunc(startPose, goalPose, scenario, paraCfg, gridMap, hCostMatrix, smooth_line)
    poseFrontPark = poseFrontParkFunc(goalPose, paraCfg); % 计算前置停车点
    
    % 初始化openList和closedList
    cost.g = 0;
    % cost.h = paraCfg.hAStar.hCost * hUnconstrainedFunc(startPose, hCostMatrix, gridMap);
    cost.h = paraCfg.hAStar.hCost * hCostFunc(startPose, goalPose, hCostMatrix, gridMap, paraCfg);
    cost.f = cost.g + cost.h;
    
    paraCfg = optSearchStrategy(paraCfg, scenario, cost.h);

    startNodeInfo = saveNodeInfoFunc(0, 0, 0, startPose, [-1000, -1000, -1000], cost, gridMap);
 
    openList   = [startNodeInfo];
    closedList = [];
    
    rsPath = [];
    
    pathPoints = []; % 路径点信息：位姿、速度、前轮转角、路径片段累计长度

    searchTimesHA = 0; % 搜索次数

    while ~isempty(openList) && searchTimesHA <= paraCfg.hAStar.maxSearchTimes % openList为空 || 搜索次数大于上限，认为Hybrid AStar失效
        searchTimesHA = searchTimesHA + 1

        [openList, minCostNode] = updateOpenList(openList, gridMap, paraCfg, closedList, hCostMatrix, goalPose, smooth_line);

        closedList = [closedList, minCostNode];
        
        paraCfg = optSearchStrategy(paraCfg, scenario, minCostNode.cost.h);
        
        if minCostNode.cost.h <= paraCfg.hAStar.disBeginRSSearch % 优化算力--距离虚拟停车位较远时，不计算RS曲线（假定距离较远时，RS曲线极大概率发生碰撞）
            [rsPath, rsCurve] = calRSCurveFunc(minCostNode.pose, poseFrontPark, scenario, paraCfg, gridMap);
        else
            continue;
        end
        
        if ~isempty(rsPath) % 搜索路径成功标志：得到无碰撞RS曲线--可优化：Dubin曲线抵达虚拟停车位，避免频繁换向 && 车辆x轴位置偏差与横摆角偏差（相对虚拟停车位）同时为小量时，直接多项式曲线或贝塞尔曲线或直线后退停车，避免频繁换向与停车转向（停车转向不一定成立，可优化后运动中完成转向）
            break;
        end
    end

    if ~isempty(rsPath)
        % 从closedList中逆向寻找路径
        pathNode = closedList(end);

        closedList(end) = [];
        
        % 路径点信息：位姿、速度、前轮转角、路径片段累计长度
        hAPath = [pathNode.pose, pathNode.vel, pathNode.deltaFront, pathNode.disSum];

        while ~(pathNode.pose(1) == startPose(1) && pathNode.pose(2) == startPose(2) && pathNode.pose(3) == startPose(3))
            for i = 1 : length(closedList) % 搜索效率很低
                if closedList(i).mapIdx.xID == pathNode.parentIdx.xID &&...
                   closedList(i).mapIdx.yID == pathNode.parentIdx.yID &&...
                   closedList(i).mapIdx.yawID == pathNode.parentIdx.yawID
                    pathNode = closedList(i);
                    hAPath = [hAPath; pathNode.pose, pathNode.vel, pathNode.deltaFront, pathNode.disSum];

                    closedList(i) = [];

                    break;
                end
            end
        end
        
        tempPath = [];
        
        [rows, cols] = size(hAPath);
        for i = 1 : rows
            for j = 1 : cols
                tempPath(i, j) = hAPath(rows + 1 - i, j);
            end
        end
        
        hAPath = tempPath;
        hAPath = specifyPathFunc(hAPath, scenario, paraCfg);
        
        unrealToRealPath = unrealToRealPathFunc(goalPose, poseFrontPark, paraCfg);
        
        refPath = [hAPath; rsPath; unrealToRealPath];
%         refPath = [hAPath];
    else
        refPath = [];
    end
end


%% 移除openList中代价值f最小的节点
function [fMinCostNode, openList] = removeNodeFunc(openList)
    fMin = inf;
    idxFMin = inf;

    for i = 1:length(openList)
        fCost = openList(i).cost.f;

        if fCost < fMin
            fMin = fCost;
            idxFMin = i;
        end
    end

    fMinCostNode = openList(idxFMin);

    openList(idxFMin) = [];
end


%% 打包节点信息--优化：剪枝ID重复计算
function nodeInfo = saveNodeInfoFunc(disSum, deltaFront, vel, pose, parentPose, cost, gridMap)
        nodeInfo.pose       = pose;       % 节点位姿
        nodeInfo.vel        = vel;        % 速度
        nodeInfo.deltaFront = deltaFront; % 前轮转角
        nodeInfo.disSum     = disSum;     % 累计弧长
        
        nodeInfo.cost       = cost;       % 代价值
        
        nodeInfo.parentPose = parentPose; % 父节点位姿
        
        mapIdx.xID   = ceil((pose(1) - gridMap.xMin) / gridMap.xyResolution);
        mapIdx.yID   = ceil((gridMap.yMax - pose(2)) / gridMap.xyResolution);       
        mapIdx.yawID = ceil((mod2pi(pose(3)) / 1.0001 - gridMap.yawMin) / gridMap.yawResolution);
        nodeInfo.mapIdx     = mapIdx;     % 节点MAP索引值
        
        mapIdx.xID   = ceil((parentPose(1) - gridMap.xMin) / gridMap.xyResolution); % 重复计算
        mapIdx.yID   = ceil((gridMap.yMax - parentPose(2)) / gridMap.xyResolution);
        mapIdx.yawID = ceil((mod2pi(parentPose(3)) / 1.0001 - gridMap.yawMin) / gridMap.yawResolution);
        nodeInfo.parentIdx  = mapIdx;     % 父节点索引值
end


%% 计算两点相对距离
function relDis = relDisFunc(pose, goal)
    relDis = abs(pose(1) - goal(1)) + abs(pose(2) - goal(2));       % 曼哈顿距离
%     relDis = ((pose(1) - goal(1))^2 + (pose(2) - goal(2))^2) ^ 0.5; % 欧几里得距离
end


%% 计算无运动约束启发值
function hUnconstrained = hUnconstrainedFunc(pose, hCostMatrix, gridMap, goalPose)
    hUnconstrained = ((pose(1) - goalPose(1))^2 + (pose(2) - goalPose(2))^2)^0.5;
%     idxCol = ceil((pose(1) - gridMap.xMin) / gridMap.xyResolution); % 列索引
%     idxRow = ceil((gridMap.yMax - pose(2)) / gridMap.xyResolution); % 行索引
%     
%     
%     hUnconstrained = hCostMatrix(idxRow, idxCol); % 次序--从左到右，从上到下
end


%% 更新openList--剪枝
function [openList, minCostNode] = updateOpenList(openList, gridMap, paraCfg, closedList, hCostMatrix, goalPose, smooth_line)
    [minCostNode, openList] = removeNodeFunc(openList); % 移除openList中f最小的节点
    
    childNodes = searchChildNodesFunc(minCostNode, gridMap, paraCfg, hCostMatrix, goalPose, smooth_line); % 拓展节点
    
    % 剪枝--closedList
    idxToBePruned = [];
    
    if ~isempty(closedList)
        for i = 1 : length(childNodes)
            isInClosedList = isInListFunc(childNodes(i), closedList, paraCfg);

            if isInClosedList == true
                idxToBePruned = [idxToBePruned, i];
            end
        end
    end
    
    while ~isempty(idxToBePruned)
        idx = idxToBePruned(end);
        childNodes(idx) = [];
        idxToBePruned(end) = [];
    end
    
    % 剪枝--openList
    idxToBePruned = [];

    for i = 1 : length(childNodes)
        [isInList, idx] = isInListFunc(childNodes(i), openList, paraCfg); 

        if isInList == true
            idxToBePruned = [idxToBePruned, i];
            
            if childNodes(i).cost.f < openList(idx).cost.f % BUG--新拓展出来的点，因为多累加了N步的前轮转角代价、前轮转角变化代价、换向代价，一般情况下g值更大，无法有效剪枝
                openList(idx) = childNodes(i);
            end
        else
            openList = [openList, childNodes(i)];
        end
    end
end


%% 拓展子节点
function childNodes = searchChildNodesFunc(minCostNode, gridMap, paraCfg, hCostMatrix, goalPose, smooth_line)
    childNodes = [];
    
    for direction = [1 -1] % 行驶方向
        for steer = -paraCfg.hAStar.maxSteer : paraCfg.hAStar.angleResolution : paraCfg.hAStar.maxSteer % 前轮转角
            disSum = minCostNode.disSum + paraCfg.hAStar.searchStep;
            vel = paraCfg.hAStar.vRef * direction;
            parentPose = minCostNode.pose;

            sizeOfRefPos = size(smooth_line(:, 1),1);
            for i = 1:sizeOfRefPos
                dist(i,1) = norm(smooth_line(i,1 : 2) - parentPose(1 : 2));
            end
            [~,idx] = min(dist);
            
            ref_kappa = smooth_line(idx, 4);

            ref_steer = atan(paraCfg.vehicle.wheelBase * ref_kappa);
            
            steer = ref_steer + steer;
            if steer > paraCfg.hAStar.maxSteer + 0.01
                continue;
            else
                if steer < -paraCfg.hAStar.maxSteer - 0.01
                    continue;
                end
            end
            
            pose = predictVehPoseFunc(minCostNode, steer, direction, paraCfg, gridMap); % 预测车辆位姿--边界检测+碰撞检测

            if ~isempty(pose)
                cost = calTotalCost(pose, minCostNode, paraCfg, steer, direction, gridMap, hCostMatrix, goalPose, smooth_line);
            
                childNode = saveNodeInfoFunc(disSum, steer, vel, pose, parentPose, cost, gridMap);
                childNodes = [childNodes, childNode];
            else
                continue;
            end
        end
    end        
end


%% 预测车辆位姿--边界检测+碰撞检测
function pose = predictVehPoseFunc(minCostNode, deltaFront, motionDir, paraCfg, gridMap)
    pose = [];
    
    x = minCostNode.pose(1);
    y = minCostNode.pose(2);
    yaw = minCostNode.pose(3);
    
    L = paraCfg.vehicle.wheelBase;
    
    numStep = ceil(paraCfg.hAStar.searchStep / paraCfg.hAStar.lineResolution) + 1;
    lineRes = paraCfg.hAStar.searchStep / (numStep - 1);
    
    disSum = 0; % 扩展长度
    
    % 运动学近似求解--缺陷：计算量大，精度低--优化：由几何关系求解精确解,参考calRSCurveFunc.m中...
    for i = 2 : numStep
        x = x + lineRes * cos(yaw) * motionDir;
        y = y + lineRes * sin(yaw) * motionDir;
        yaw = yaw + lineRes * tan(deltaFront) / L * motionDir;
        
        disSum = disSum + lineRes;
        
        if disSum > paraCfg.gridMap.xyResolution % 过检测
            disSum = 0;
            
            % 边界检测--单步扩展中间点,searchStep > gridMap.res * 1.5
            if (x > gridMap.xMin && x < gridMap.xMax) && (y > gridMap.yMin && y < gridMap.yMax)
                isOccupied = collisionDetectionFunc([x, y, yaw], paraCfg, gridMap);

                if isOccupied == true
                    return;
                end
            else
                return;
            end     
        end
        
    end
    
    % 边界检测--单步扩展终点
    if (x > gridMap.xMin && x < gridMap.xMax) && (y > gridMap.yMin && y < gridMap.yMax)
        % 碰撞检测--现有搜索步长(gridMap.res * 1.5)，允许仅对拓展节点做一次碰撞检测，有理论漏检，但实际可涵盖
        isOccupied = collisionDetectionFunc([x, y, yaw], paraCfg, gridMap);

        if isOccupied == false
            pose = [x, y, yaw];
        end
    end
end


%% 计算代价--为方便调参，且物理含义显式化，后续可尝试：searchStep * (1 * weight_front + 1 * weight_back + steer * weight_steer + steerChange * weight_steerChange + seitch * weight_switch)
function nodeCost = calTotalCost(pose, minCostNode, paraCfg, deltaFront, motionDirection, gridMap, hCostMatrix, goalPose, smooth_line)
    if motionDirection == 1
        costForward = paraCfg.hAStar.forwardCost * paraCfg.hAStar.searchStep;
        costBack   = 0;
    else
        costForward = 0;
        costBack   = paraCfg.hAStar.backCost * paraCfg.hAStar.searchStep;
    end
    
    costSteer = paraCfg.hAStar.steerCost * abs(deltaFront);
    costSteerChange = paraCfg.hAStar.steerChangeCost * abs(deltaFront - minCostNode.deltaFront);
    
    if motionDirection * minCostNode.vel < 0
        costSwitch = paraCfg.hAStar.switchCost * (paraCfg.hAStar.vRef * 2.0); % 换向代价权重 * 速度变化量--未考虑换向时间（搜索步长）
    else
        costSwitch = 0;
    end
   
%     nodeCost.g = minCostNode.cost.g + costForward * dis2obs + costBack * dis2obs + costSteer*4.0 + costSteerChange*2 + costSwitch;

    sizeOfRefPos = size(smooth_line(:, 1),1);
    for i = 1:sizeOfRefPos
        dist(i,1) = norm(smooth_line(i,1 : 2) - pose(1 : 2));   
    end
    [min_dis,idx] = min(dist);
    
    if min_dis > 0.03
        gain = 1.0 + (min_dis - 0.03) * 100;
    else
        gain = 1.0;
    end
    
%    gain = 1.0;

    nodeCost.g = minCostNode.cost.g + costForward*gain + costBack*10*gain + costSteer*1 + costSteerChange*1 + costSwitch;
    % nodeCost.h = paraCfg.hAStar.hCost * hUnconstrainedFunc(pose, hCostMatrix, gridMap);
    nodeCost.h = paraCfg.hAStar.hCost * hCostFunc(pose, goalPose, hCostMatrix, gridMap, paraCfg);
    nodeCost.f = nodeCost.g + nodeCost.h;
end


%% hCostFunc--优化：h代价--max{几何距离, RS长度}
function hCost = hCostFunc(startPose, goalPose, hCostMatrix, gridMap, paraCfg)
    hUnconstrained = hUnconstrainedFunc(startPose, hCostMatrix, gridMap, goalPose); % 无约束启发代价
     
    % 计算无碰撞启发代价
    reedsConnObj = reedsSheppConnection; % 创建RS曲线连接器对象 % reedsConnObj = reedsSheppConnection('DisabledPathTypes',{'LpRnLp'});

    reedsConnObj.MinTurningRadius = paraCfg.rs.rMin;   % 最小转弯半径
    reedsConnObj.ReverseCost = 1; % 后退代价权重

    rsCurve = connect(reedsConnObj, startPose, goalPose); % 求解RS曲线
    
    for numSegRS = 1 : 5
        if rsCurve{1, 1}.MotionLengths(numSegRS) == 0
            numSegRS = numSegRS - 1;
            
            break;
        end
    end
    
    hCollisionFreeCost = 0;
    
    for i = 1 : numSegRS
        motionLength = rsCurve{1, 1}.MotionLengths(i); % 规划路径片段长度
        
        hCollisionFreeCost = hCollisionFreeCost + motionLength;
    end
     
     hCost = hUnconstrained; % hCost = max(hCollisionFreeCost, hUnconstrained); % 没能驾驭
end


%% 是否在列表中
function [isInList, idx] = isInListFunc(nodeInfo, List, paraCfg)
    isInList = false;
    idx = 0;
    
    for i = 1 : length(List)
        if nodeInfo.mapIdx.xID == List(i).mapIdx.xID &&... % 剪枝--同一三维栅格
           nodeInfo.mapIdx.yID == List(i).mapIdx.yID &&...
           nodeInfo.mapIdx.yawID == List(i).mapIdx.yawID
            idx = i;
            isInList = true;
            
            break;
        end
    end
end


%% 角度限制在-pi~pi
function angle = mod2pi(angle)
    while angle > pi
        angle = angle - 2 * pi;
    end
    
    while angle < -pi
        angle = angle + 2 * pi;
    end
end


%% 细化Hybrid AStar搜索路径--插点
function [hAPath] = specifyPathFunc(hAPath, scenario, paraCfg)
    tempPath = [];
    
    [rows, cols] = size(hAPath);
    
    for i = 1 : rows - 1
        x = hAPath(i, 1);
        y = hAPath(i, 2);
        yaw = hAPath(i, 3);
        
        vel = hAPath(i + 1, 4);
        deltaFront = hAPath(i + 1, 5);
        
        disSum = hAPath(i, 6);
        
        tempPath = [tempPath; x, y, yaw, vel, deltaFront, disSum];

        L = paraCfg.vehicle.wheelBase;
        
        searchStep = hAPath(i + 1, 6) - hAPath(i, 6);
        
        numStep = ceil(searchStep / paraCfg.hAStar.lineResolution) + 1;
        lineRes = searchStep / (numStep - 1);
        
        motionDir = sign(vel);
        
        for i = 2 : numStep - 1
            x = x + lineRes * cos(yaw) * motionDir;
            y = y + lineRes * sin(yaw) * motionDir;
            yaw = yaw + lineRes * tan(deltaFront) / L * motionDir;
            
            disSum = disSum + lineRes;
            
            tempPath = [tempPath; x, y, yaw, vel, deltaFront, disSum];
        end
    end

    hAPath = tempPath;
end


%% 优化搜索策略
function paraCfg = optSearchStrategy(paraCfg, scenario, hUnconstrained)
    % 变搜索步长、参考车速、前轮转角分辨率--出库需要小searchStep，目前设计只考虑泊车 OR 泊车逆行轨迹实现出库
    paraCfg.hAStar.vRef = 3.0;
    paraCfg.hAStar.searchStep = scenario.costMap.CellSize * 2.414;
    paraCfg.hAStar.angleResolution = 20.0 / 57.3;

    % 变后退、换向代价权重--有效，可优化：依据车辆横摆角+车辆位置+停车位位置+相对距离/hCost，决策backCost取值
    if hUnconstrained >= 14
        paraCfg.hAStar.backCost = 20.0; % 防止远距离倒车
        paraCfg.hAStar.switchCost = 0.0;
    elseif  hUnconstrained >= paraCfg.hAStar.disBeginRSSearch
        hAStar.backCost = 10.0;
        paraCfg.hAStar.switchCost = 0.0;
    else
        paraCfg.hAStar.backCost = 1.0;
        paraCfg.hAStar.switchCost = 0.0;
    end
    
    % 变转向代价权重--取值大，路径平滑，但子节点方向扩展性差，搜索次数变大
    if hUnconstrained >= 20
        paraCfg.hAStar.steerCost = 0.5;
        paraCfg.hAStar.steerChangeCost = 2.0;
    elseif hUnconstrained >= 14
        paraCfg.hAStar.steerCost = 0.5 * 6 / 8; % steerCost-距离关系 对应 搜索步长-距离关系(逻辑没有理清，不是最优)
        paraCfg.hAStar.steerChangeCost = 2.0 * 6 / 8;
    elseif  hUnconstrained >= paraCfg.hAStar.disBeginRSSearch
        paraCfg.hAStar.steerCost = 0.5 * 4 / 8;
        paraCfg.hAStar.steerChangeCost = 2.0 * 4 / 8;
    else
        paraCfg.hAStar.steerCost = 0.5 * 4 / 8;
        paraCfg.hAStar.steerChangeCost = 2.0 * 4 / 8;
    end
    
    % 更新因变参数
    paraCfg.hAStar.yawPruneThreshold = paraCfg.hAStar.searchStep /... % 剪枝阈值--角度 
                               (scenario.vehicle.wheelBase /...
                               tan(paraCfg.hAStar.angleResolution)) * 0.95;
    paraCfg.rs.vRef = paraCfg.hAStar.vRef; % 参考速度
end


%% 计算虚拟停车位到真实停车位引导轨迹
function unrealToRealPath = unrealToRealPathFunc(goalPose, poseFrontParking, paraCfg)
    unrealToRealPath = [];

    x = poseFrontParking(1);
    y = poseFrontParking(2);
    yaw = poseFrontParking(3);
    
    vel = -0.6;
    deltaFront = 0.0;
    
    disSum = 0;

    numStep = ceil(paraCfg.disFrontParking / paraCfg.hAStar.lineResolution) + 1;
    lineRes = paraCfg.disFrontParking / (numStep - 1);
    
    for i = 2 : numStep
        x = x - lineRes * cos(yaw);
        y = y - lineRes * sin(yaw);
        yaw = yaw;
        
        disSum = disSum + lineRes;

        unrealToRealPath = [unrealToRealPath; x, y, yaw, vel, deltaFront, disSum];
    end
end


%% 计算前置停车点--垂直泊车，虚拟停车点前置；水平泊车，虚拟停车点后置或不需要
function poseFrontParking = poseFrontParkFunc(goalPose, paraCfg)
    disFrontParking = paraCfg.disFrontParking;
    
    poseFrontParking = [goalPose(1) + disFrontParking * cos(goalPose(3)), goalPose(2) + disFrontParking * sin(goalPose(3)), goalPose(3)];
end
