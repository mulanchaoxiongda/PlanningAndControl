%% 创建仿真场景
function scenario = sceneCreatorFunc()
    % 车辆参数-奥迪A6L
    vehicle.wheelBase  = 0.75;       % 轴距
    vehicle.wid        = 0.66;       % 车宽
    vehicle.rear2front = 0.45;       % 后轮-前悬距离
    vehicle.rear2back  = 0.45;       % 后轮-后悬距离
    vehicle.maxSteer   = 65.0 / 57.3; % 最大前轮转角（不能用满，给控制留余量）
    vehicle.minCircle  = vehicle.wheelBase/tan(vehicle.maxSteer); % 最小转弯半径
    
    scenario.vehicle = vehicle;
    
    % 车辆起始、停止位置--gridMap.res = 0.25
    startPose = [3.5, 3.5,  0]; % 初始点位置--车辆后轴中心（全局规划）
    goalPose  = [4.5, 4.5,  pi / 2]; % 停车点位置--车辆后轴中心
    
%     startPose = [3.5, 5.0, -pi / 2]; % 初始点位置--车辆后轴中心（全局规划）
%     goalPose  = [3.0, 4.5, -pi]; % 停车点位置--车辆后轴中心
    
    scenario.startPose = startPose;
    scenario.goalPose  = goalPose;
    
    % 占用栅格图
    res = 0.25; % 分辨率--取值：x厘米
    
    scenario.costMap = costMapCreatorFunc(scenario, res);
end


%% 生成占用栅格地图--自动驾驶工具箱函数
function costmap = costMapCreatorFunc(scenario, res)
    resCostMap   = res;

    lenParkingLot = 8;
    widParkingLot = 8;

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
    
    for x = 0.25 : res : 3.0
        for y = 5.0 : res : 7.75
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    for x = 0.25 : res : 3.0
        for y = 0.25 : res : 2.75
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    for x = 5.25 : res : 7.75
        for y = 0.25 : res : 2.75
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    for x = 5.25 : res : 7.75
        for y = 5.0 : res : 7.75
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
    for x = 0.25 : res : 4.0
        for y = 4.0 : res : 7.75
            xIdx = round(x / res);
            yIdx = ceil((widParkingLot - y) / res);
            
            costMap(yIdx, xIdx) = 255;
        end
    end
    
%     for x = 4.0 : res : 7.75
%         for y = 0.25 : res : 4.0
%             xIdx = round(x / res);
%             yIdx = ceil((widParkingLot - y) / res);
%             
%             costMap(yIdx, xIdx) = 255;
%         end
%     end
    
    costMap = im2single(uint8(costMap));

    costmap = vehicleCostmap(costMap, 'CellSize', resCostMap);
end
