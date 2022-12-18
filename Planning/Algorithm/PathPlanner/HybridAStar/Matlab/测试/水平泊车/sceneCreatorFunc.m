%% 创建仿真场景
function scenario = sceneCreatorFunc()
    % 车辆参数-奥迪A6L
    vehicle.wheelBase  = 3.024; % 轴距
    vehicle.wid        = 1.886; % 车宽
    vehicle.rear2front = 4.150; % 后轮-前悬距离
    vehicle.rear2back  = 0.900; % 后轮-后悬距离
    vehicle.maxSteer   = 35.0 / 57.3; % 最大前轮转角（不能用满，给控制留余量）
    vehicle.minCircle  = vehicle.wheelBase/tan(vehicle.maxSteer); % 最小转弯半径
    vehicle.len        = vehicle.rear2front + vehicle.rear2back;  % 车长
    
    scenario.vehicle = vehicle;
    
    % 停车场参数 -- 水平车位7.0*2.5  倾斜车位6.0*2.5  垂直车位5.0~6.0*2.5  通道宽6.0m
    parkingLot.lenParkSpa = 2.5;  % 停车位长度--y轴
    parkingLot.widParkSpa = 7.0;  % 停车位宽度 + 0.25 * 2(相邻车位让出的宽度和)--宽度和应为栅格分辨率的倍数--x轴
    parkingLot.numPerRow  = 7;    % 每行的停车位数量
    parkingLot.numRow     = 6;    % 停车位行数--定值(偶数)
    parkingLot.widRoad    = 4;    % 通道宽度
    
    scenario.parkingLot = parkingLot;
    
    % 车辆起始、停止位置--gridMap.res = 0.25
    startPose = [23.3, 23.3, -pi];     % 初始点位置--车辆后轴中心（局部规划）,没有调通
%     startPose = [21.85 + 0.0, 22.8, -pi];     % 初始点位置--车辆后轴中心（局部规划）
    
%     startPose = [21.85 + 0.0, 23.8, -pi];     % 初始点位置--车辆后轴中心（局部规划）
%     startPose = [24.85 + 0.0, 23.8, -pi];     % 初始点位置--车辆后轴中心（局部规划）
%     startPose = [24.85 + 0.0, 22.8, -pi];     % 初始点位置--车辆后轴中心（局部规划）
    
    
    
    goalPose  = [30.50 + 1.6, 26.3, -pi]; % 停车点位置--车辆后轴中心
    
    scenario.startPose = startPose;
    scenario.goalPose  = goalPose;
    
    % 占用栅格图
    res = 0.25; % 分辨率--取值：x厘米
    
    scenario.costMap = costMapCreatorFunc(scenario, res);
end


%% 生成占用栅格地图--自动驾驶工具箱函数
function costmap = costMapCreatorFunc(scenario, res)
    resCostMap   = res;
    lenParkSpace = scenario.parkingLot.lenParkSpa;
    widParkSpace = scenario.parkingLot.widParkSpa;
    numRow       = scenario.parkingLot.numRow;
    numPerRow    = scenario.parkingLot.numPerRow;
    widRoad      = scenario.parkingLot.widRoad;
    numRoad      = numRow * 0.5; % 通道数目

    xInitPos = widRoad + 1;
    lenParkingLot = (numPerRow + 1) * resCostMap + numPerRow * widParkSpace + xInitPos * 2;
    widParkingLot = (numRow *0.5 + 1) * resCostMap + numRoad * widRoad + numRow * lenParkSpace;
    xEndPos = xInitPos + widParkSpace * numPerRow + resCostMap * numPerRow;

    numXAxis = ceil((widParkingLot / resCostMap)); % 栅格数
    numYAxis = ceil((lenParkingLot / resCostMap));

    for j = 1 : numYAxis
        costMap(1, j) = 255;
        costMap(numXAxis, j) = 255;
    end

    for i = 1 : numXAxis
        costMap(i, 1) = 255;
        costMap(i, numYAxis) = 255;
    end

    for i = floor(numXAxis * 0.75) : floor(numXAxis * 0.85)
        costMap(i, 1) = 0;
    end

    for i = floor(numXAxis * 0.15) : floor(numXAxis * 0.25)
        costMap(i, numYAxis) = 0;
    end

    for j = xInitPos / resCostMap + 1 : widParkSpace / resCostMap + 1 : xEndPos / resCostMap + 1
        for i = 2 : lenParkSpace / resCostMap + 1
            costMap(i, j) = 255;
        end

        for i = numXAxis - 1 : -1 : numXAxis - lenParkSpace / resCostMap
            costMap(i, j) = 255;
        end
        
        for k = 1 : numRow * 0.5 - 1
            for i = (1 + lenParkSpace / resCostMap) + (widRoad / resCostMap) * k + (lenParkSpace * 2 / resCostMap + 1) * (k - 1) + 1 :...
                    (1 + lenParkSpace / resCostMap) + (widRoad / resCostMap) * k + (lenParkSpace * 2 / resCostMap + 1) *  k
                costMap(i, j) = 255;
            end
        end
    end

    for j = xInitPos / resCostMap + 1 : 1 : xEndPos / resCostMap + 1
        for k = 1 : numRow * 0.5
            xInd = (1 + lenParkSpace / resCostMap) + (widRoad / resCostMap) * k + (lenParkSpace * 2 / resCostMap + 1) * (k - 1) + 1 + lenParkSpace / resCostMap;
            
            costMap(xInd, j) = 255;
        end
    end
    
    %添加停泊车（左上）
    for x = 23.2 - scenario.vehicle.len / 2 : res : 23.2 + scenario.vehicle.len / 2
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 0.5 - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
        
    %添加停泊车（左下）
    for x = 23.20 - scenario.vehicle.len / 2 : res : 23.20 + scenario.vehicle.len / 2
        for y = 21.75 - lenParkSpace - res : res : 21.75 - 0.5 - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    %添加停泊车（正下）
    for x = 30.62 - scenario.vehicle.len / 2 : res : 30.62 + scenario.vehicle.len / 2
        for y = 21.75 - lenParkSpace - res : res : 21.75 - 0.5 - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    %添加停泊车（右下）
    for x = 37.87 - scenario.vehicle.len / 2 : res : 37.87 + scenario.vehicle.len / 2
        for y = 21.75 - lenParkSpace - res : res : 21.75 - 0.5 - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    %添加停泊车（右上）
    for x = 37.87 - scenario.vehicle.len / 2 : res : 37.87 + scenario.vehicle.len / 2
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 0.5 - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end

    costMap = im2single(uint8(costMap));

    costmap = vehicleCostmap(costMap, 'CellSize', resCostMap);
end
