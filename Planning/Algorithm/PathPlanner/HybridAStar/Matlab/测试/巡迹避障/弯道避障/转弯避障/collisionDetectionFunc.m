%% 碰撞检测
function isOccupied = collisionDetectionFunc(poseVehicle, paraConfig, gridMap)
    isOccupied = false;
    
    % 车辆位姿--后轴中心
    x = poseVehicle(1);
    y = poseVehicle(2);
    yaw = poseVehicle(3);
    
    transMatrix = [cos(yaw) sin(yaw) 0;   % 转换矩阵--本体系到全局系
                  -sin(yaw) cos(yaw) 0;
                   0        0        1]';
    % 求解检测点
    samplingInterval = paraConfig.collisionDet.interval; % 检测点间距
    
    lenExpansion = paraConfig.collisionDet.lenExpansion; % 车身膨胀长度
    widExpansion = paraConfig.collisionDet.widExpansion; % 车身膨胀宽度
    
    xLB = -paraConfig.vehicle.rear2back - lenExpansion; % 膨胀后车辆矩形区域边界值--车体系
    xUB = paraConfig.vehicle.rear2front + lenExpansion;
    yLB = -0.5 * paraConfig.vehicle.wid - widExpansion;
    yUB = 0.5 * paraConfig.vehicle.wid + widExpansion;
    
    xSamplingNum = ceil((xUB - xLB) / samplingInterval) + 1; % 检测点序号
    ySamplingNum = ceil((yUB - yLB) / samplingInterval) + 1;
    
    xsamplingInterval = (xUB - xLB) / (xSamplingNum - 1); % 更新检测点采样间隔
    ysamplingInterval = (yUB - yLB) / (ySamplingNum - 1);
    
    xSampling = []; % 检测点坐标集合--车体系，存在重复检测
    ySampling = [];
    
    for i = 1 : xSamplingNum
        for j = 1 : ySamplingNum
            xVehSys = xLB + (i - 1) * xsamplingInterval; % 检测点坐标--车体系，后轴中心为坐标原点
            yVehSys = yLB + (j - 1) * ysamplingInterval;
            
            poseGlobal = transMatrix * [xVehSys, yVehSys, 0]'+ [x, y, 0]'; % 检测点坐标--全局系
            
            if poseGlobal(1) > gridMap.xMax || poseGlobal(1) < gridMap.xMin ||... % 边界检测
               poseGlobal(2) > gridMap.yMax || poseGlobal(2) < gridMap.yMin
                isOccupied = true;
                
                return;
            end
            
            xID = ceil((poseGlobal(1) - gridMap.xMin) / gridMap.xyResolution); % 栅格索引值
            yID = ceil((gridMap.yMax - poseGlobal(2)) / gridMap.xyResolution);

            if gridMap.gridOccupy(yID, xID) == 1 % 栅格占用检测
                isOccupied = true;
                
                return;
            end
        end
    end
end
