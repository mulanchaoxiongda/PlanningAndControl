%% 创建仿真场景
function scenario = sceneCreatorFunc()
    % 车辆参数-奥迪A6L
    vehicle.wheelBase  = 3.024;       % 轴距
    vehicle.wid        = 1.886;       % 车宽
    vehicle.rear2front = 4.150;       % 后轮-前悬距离
    vehicle.rear2back  = 0.900;       % 后轮-后悬距离
    vehicle.maxSteer   = 35.0 / 57.3; % 最大前轮转角（不能用满，给控制留余量）
    vehicle.minCircle  = vehicle.wheelBase/tan(vehicle.maxSteer); % 最小转弯半径
    
    scenario.vehicle = vehicle;
    
    % 停车场参数 -- 水平车位6.0*2.5  倾斜车位6.0*2.5  垂直车位5.0~6.0*2.5  通道宽6.0m
    parkingLot.lenParkSpa = 6.0;            % 停车位长度--y轴
    parkingLot.widParkSpa = 2.4 + 0.3 * 2;  % 停车位宽度 + 0.25 * 2(相邻车位让出的宽度和)--宽度和应为栅格分辨率的倍数--x轴
    parkingLot.numPerRow  = 12;             % 每行的停车位数量
    parkingLot.numRow     = 4;              % 停车位行数--定值(偶数)
    parkingLot.widRoad    = 6;              % 通道宽度
    
    scenario.parkingLot = parkingLot;
    
    % 车辆起始、停止位置--gridMap.res = 0.25
    startPose = [5.000, 7.50,  0];     % 初始点位置--车辆后轴中心（全局规划）
%     startPose = [25.00, 10.00,  0];     % 初始点位置--车辆后轴中心（全局规划）
%     startPose = [30.00, 27.5, -pi];     % 初始点位置--车辆后轴中心（局部规划）
    startPose = [40.00, 27.5, -pi];     % 初始点位置--车辆后轴中心（局部规划）
    goalPose  = [34.75, 34.7, -pi / 2]; % 停车点位置--车辆后轴中心
    
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
    
    % 添加停泊车（左上）
    for x = scenario.goalPose(1) - scenario.parkingLot.widParkSpa - scenario.vehicle.wid / 2 :...
            res :...
            scenario.goalPose(1) - scenario.parkingLot.widParkSpa + scenario.vehicle.wid / 2
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 1.4
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（左左上）
    for x = scenario.goalPose(1) - scenario.parkingLot.widParkSpa * 2 - scenario.vehicle.wid / 2 - res * 2 :...
            res :...
            scenario.goalPose(1) - scenario.parkingLot.widParkSpa * 2 + scenario.vehicle.wid / 2 - res * 2
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 1.4
            xIdx = ceil(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（右上）
    for x = scenario.goalPose(1) + scenario.parkingLot.widParkSpa - scenario.vehicle.wid / 2 + 2 * res :...
            res :...
            scenario.goalPose(1) + scenario.parkingLot.widParkSpa + scenario.vehicle.wid / 2 + 2 * res
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 1.4
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（右右上）
    for x = scenario.goalPose(1) + scenario.parkingLot.widParkSpa * 2 - scenario.vehicle.wid / 2 + 3 * res :...
            res :...
            scenario.goalPose(1) + scenario.parkingLot.widParkSpa * 2 + scenario.vehicle.wid / 2 + 3 * res
        for y = widParkingLot - lenParkSpace - res : res : widParkingLot - 1.4
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（右下）
    for x = scenario.goalPose(1) + scenario.parkingLot.widParkSpa - scenario.vehicle.wid / 2 + 2 * res :...
            res :...
            scenario.goalPose(1) + scenario.parkingLot.widParkSpa + scenario.vehicle.wid / 2 + 2 * res
        for y = widParkingLot - lenParkSpace - widRoad - res - scenario.vehicle.rear2front - scenario.vehicle.rear2back :...
                res :...
                widParkingLot - lenParkSpace - widRoad - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（右右下）
    for x = scenario.goalPose(1) + scenario.parkingLot.widParkSpa * 2 - scenario.vehicle.wid / 2 + 3 * res :...
            res :...
            scenario.goalPose(1) + scenario.parkingLot.widParkSpa * 2 + scenario.vehicle.wid / 2 + 3 * res
        for y = widParkingLot - lenParkSpace - widRoad - res - scenario.vehicle.rear2front - scenario.vehicle.rear2back :...
                res :...
                widParkingLot - lenParkSpace - widRoad - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（正下）
    for x = scenario.goalPose(1) - scenario.vehicle.wid / 2 + res :...
            res :...
            scenario.goalPose(1) + scenario.vehicle.wid / 2 + res
        for y = widParkingLot - lenParkSpace - widRoad - res - scenario.vehicle.rear2front - scenario.vehicle.rear2back :...
                res :...
                widParkingLot - lenParkSpace - widRoad - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end

    % 添加停泊车（左下）
    for x = scenario.goalPose(1) - scenario.parkingLot.widParkSpa - scenario.vehicle.wid / 2 :...
            res :...
            scenario.goalPose(1) - scenario.parkingLot.widParkSpa + scenario.vehicle.wid / 2
        for y = widParkingLot - lenParkSpace - widRoad - res - scenario.vehicle.rear2front - scenario.vehicle.rear2back :...
                res :...
                widParkingLot - lenParkSpace - widRoad - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    % 添加停泊车（左左下）
    for x = scenario.goalPose(1) - scenario.parkingLot.widParkSpa * 2.0 - scenario.vehicle.wid / 2 - res :...
            res :...
            scenario.goalPose(1) - scenario.parkingLot.widParkSpa * 2.0 + scenario.vehicle.wid / 2 - res
        for y = widParkingLot - lenParkSpace - widRoad - res - scenario.vehicle.rear2front - scenario.vehicle.rear2back :...
                res :...
                widParkingLot - lenParkSpace - widRoad - res
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
%     % 障碍物1
%     for x = 40.0 : res : 43.0
%         for y = 10.0 : res : 10.5
%             xIdx = round(x / res);
%             yIdx = ceil((widParkingLot - y) / res);
%             
%             costMap(yIdx, xIdx) = 255;
%         end
%     end
%     
%     % 障碍物2
%     for x = 33 : res : 33.25
%         for y = 25.5 : res : 29.5
%             xIdx = round(x / res);
%             yIdx = ceil((widParkingLot - y) / res);
%             
%             costMap(yIdx, xIdx) = 255;
%         end
%     end
%     
%     % 障碍物3
%     for x = 33.5 : res : 35
%         for y = 25.5 : res : 26.0
%             xIdx = round(x / res);
%             yIdx = ceil((widParkingLot - y) / res);
%             
%             costMap(yIdx, xIdx) = 255;
%         end
%     end
    
    costMap = im2single(uint8(costMap));

    costmap = vehicleCostmap(costMap, 'CellSize', resCostMap);
end
