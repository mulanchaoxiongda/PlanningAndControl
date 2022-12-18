%% Dijkstra求解各点到目标点的最短路径长度--作为Hybrid AStar的启发代价值：可优化，提高启发项引导效率&&提高hCostMatrix求解速度
function hCostMatrix = calHeuristicItemFunc(goalPose, gridMap)
    [rows, cols] = size(gridMap.gridOccupy); % 地图行列数
    
    S = []; % 初始化S--从源节点到本节点已求得的最小距离
    
    U(:, 1) = 1 : rows * cols; % 初始化U--从源节点到本节点暂时求得的最小距离
    U(:, 4) = inf;
    for row = 1 : rows
        for col = 1 : cols
            idxU = (row - 1) * cols + col;
            U(idxU, 2) = col;
            U(idxU, 3) = row;
        end
    end
    
    goalID.xID   = ceil((goalPose(1) - gridMap.xMin) / gridMap.xyResolution); % 目标点列索引值
    goalID.yID   = ceil((gridMap.yMax - goalPose(2)) / gridMap.xyResolution); % 目标点行索引值
    goalID.idx   = (goalID.yID - 1) * cols + goalID.xID;                      % 目标点索引值--从左到右，从上到下（gridMap.occupy定义的次序）
    
    U(goalID.idx, :) = [goalID.idx, goalID.xID, goalID.yID, 0];
    
    while ~isempty(U) % Dijkstra主循环
        [gCostMin, idxGCostMin] = min(U(:, 4)); % 查找U中最小gCost成员
        
        if gCostMin == inf
            break;
        end
        
        S(end + 1, :) = U(idxGCostMin, :); % 更新S--最小gCost成员移入S
        U(idxGCostMin, :) = [];            % 更新U--最小gCost成员移出U
        
        parentID.xID = S(end, 2); % 父节点列索引
        parentID.yID = S(end, 3); % 父节点行索引
        parentID.idx = S(end, 1); % 父节点索引
        
        childNodes = findChildNodes(parentID, cols, rows, gridMap, S, goalID); % 拓展子节点
        
        while ~isempty(childNodes)
            idxChildNodes = find(U(:, 1) == childNodes(end, 1)); % 子节点在U中索引值
                       
            gCostChildNode = childNodes(end, 4) + S(end, 4); % 计算子节点gCost
            
           if gCostChildNode < U(idxChildNodes, 4)
               U(idxChildNodes, 4) = gCostChildNode; % 更新U--优化U中子节点栅格g值
           end
            
            childNodes(end, :) = [];
        end
    end
    
    hCostMatrix = ones(rows, cols) * inf; % 索引同gridMap.occupy

    while ~isempty(S)
        xID = S(end, 2); % 列索引
        yID = S(end, 3); % 行索引
        
        if xID ~= 0 && yID ~= 0
            hCostMatrix(yID, xID) = S(end, 4);
        end
        
        S(end, :) = [];
    end
    save hCostMatrix hCostMatrix; % 留存--供调试算法
end


%% 拓展临近节点
function childNodes = findChildNodes(parentID, xIDMax, yIDMax, gridMap, S, goalID)
    childNodes = [];
    
    disToGoal = (abs(goalID.xID - parentID.xID) + abs(goalID.yID - parentID.yID)) * gridMap.xyResolution; % 车辆相对虚拟停车点的曼哈顿距离
    
    if disToGoal > 14.0
        xWeight = 1.0; % 取1.0，优于取2.0 % 取2.0--设计思路：增大横向通带代价值，使子节点沿着横向通道快速拓展 && 快速完成90度转向时的子节点拓展，有效加快搜索速度；牺牲了路径的平顺性（轻度不合理转向，可以加大转向代价权重）换取的搜索速度
        yWeight = 1.0;
    else
        xWeight = 1.0;
        yWeight = 1.0;
    end
    
    for deltaRow = -1 : 1 : 1
        for deltaCol = -1 : 1 : 1
            if deltaRow ~= 0 || deltaCol ~= 0
                yID = parentID.yID + deltaRow;
                xID = parentID.xID + deltaCol;
                idx = (yID - 1) * xIDMax + xID;
                
                if xID > 0 && xID <= xIDMax && yID > 0 && yID <= yIDMax % 边界检测--过判断
                    if gridMap.gridOccupy(yID, xID) ~= 1 && ~ismember(idx, S(:, 1)) % 非障碍栅格 && 非S栅格
                        stepCost = (abs(deltaRow) * yWeight + abs(deltaCol) * xWeight) * gridMap.xyResolution; % 移动代价--伪曼哈顿距离
                        
                        childNodes = [childNodes; idx, xID, yID, stepCost];
                    end
                end
            end
        end
    end
end
