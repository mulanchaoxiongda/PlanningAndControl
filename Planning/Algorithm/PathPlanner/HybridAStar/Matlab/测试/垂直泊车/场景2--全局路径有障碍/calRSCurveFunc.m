%% 求解RS曲线--漏解BUG：RS曲线求解完后进行碰撞检测，若最终轨迹发生碰撞，规划会失败，但未检测的47种RS曲线可能存在无碰撞轨迹
function [rsPath, rsCurve] = calRSCurveFunc(startPose, goalPose, scenario, paraConfig, gridMap)
    reedsConnObj = reedsSheppConnection; % 创建RS曲线连接器对象 % reedsConnObj = reedsSheppConnection('DisabledPathTypes',{'LpRnLp'});

    reedsConnObj.MinTurningRadius = paraConfig.rs.rMin;   % 最小转弯半径
    reedsConnObj.ReverseCost = 1; % 后退代价权重

    rsCurve = connect(reedsConnObj, startPose, goalPose); % 求解RS曲线

    rsPath = []; % RS路径
    
    for numSegRS = 1 : 5
        if rsCurve{1, 1}.MotionLengths(numSegRS) == 0
            numSegRS = numSegRS - 1;
            
            break;
        end
    end
    
    earlyArrivalSign = 0; % 车辆早于前置点对准车位标志--若为真，则后续RS曲线可剔除，避免不必要前置点附近出现停车换向（多余动作）

    for i = 1 : numSegRS
        motionLength = rsCurve{1, 1}.MotionLengths(i);       % 规划路径片段长度
        motionType = rsCurve{1, 1}.MotionTypes{1, i};        % 规划路径片段类型--左转、右转、直行
        motionDirection = rsCurve{1, 1}.MotionDirections(i); % 规划路径片段行驶方向--前进、后退
        
        [pathPoints, startPose, earlyArrivalSign] = calPathPoints(startPose, motionType, motionDirection, motionLength, paraConfig, earlyArrivalSign); % 以线运动分辨率为间隔，求解规划路径片段的路径点--片段拼接处存在重复路径点，后续可补点实现停车换向/转向；与Hybrid AStar算法的片段衔接处理不同，HAStar衔接思路：片段拼接处删除重合点，转向不停车，后端优化算法实现平滑转向，换向停车，由速度规划实现平顺换向（补点--位置重合、时间不重合）；RS、HAStar差异处理的出发点：HAStar转向处理存在后端优化误差，由车身膨胀尺寸涵盖优化误差，确保不碰撞障碍物，RS转向在狭窄停车点附件，停车转向避免优化误差引发碰撞（也可尝试优化转向，依赖尺寸膨胀避免碰撞）
        
        if i ~= 1 % 删除重复节点
            pathPoints(1, :) = [];
        end
        
        numPathPoints = size(pathPoints(:,1), 1); % 碰撞检测--路径点间隔远小于碰撞检测点间隔，需要降采样后做检测
        
        numSplInterval = paraConfig.collisionDet.interval / paraConfig.rs.lineMotionRes; % 降采样间隔
        
        for j = 1 : numSplInterval : numPathPoints
            isOccupied = collisionDetectionFunc(pathPoints(j, 1 : 3), paraConfig, gridMap);
            
            if isOccupied == true % 屏蔽碰撞检测--调试用
                rsPath = [];
                return;
            end
        end

        if j ~= numPathPoints % 本片段终点是后续片段起点，重复检测
            j = numPathPoints;

            isOccupied = collisionDetectionFunc(pathPoints(j, 1 : 3), paraConfig, gridMap);

            if isOccupied == true % 屏蔽碰撞检测--调试用 
                rsPath = [];
                return;
            end
        end

        rsPath = [rsPath; pathPoints];
        
        if earlyArrivalSign == 1
            disp('earlyArrivalSign = 1!\n');
            
            return;
        end
    end
end


%% 路径点解算
function [pathPoints, startPose, earlyArrivalSign] = calPathPoints(startPose, motionType, motionDirection, motionLength, paraConfig, earlyArrivalSign)
    x = startPose(1);
    y = startPose(2);
    yaw = startPose(3);
    
    rMin = paraConfig.rs.rMin;
    lineRes = paraConfig.rs.lineMotionRes;
    angleRes = paraConfig.rs.angMotionRes;
    maxSteer = paraConfig.rs.maxSteer;
    
    pathPoints = [];

    if motionType == 'L'
        centerAngle = yaw - pi / 2;    
        xCenter = x - rMin * cos(centerAngle); % 转弯路径圆心坐标
        yCenter = y - rMin * sin(centerAngle);
    
        numPathPoints = ceil((motionLength / rMin) / angleRes) + 1; % 路径点个数
        angleResTemp = (motionLength / rMin) / (numPathPoints - 1); % 更新分辨率
        
        for i = 1 : numPathPoints
            x = xCenter + rMin * cos(centerAngle);
            y = yCenter + rMin * sin(centerAngle);
            yaw = centerAngle + pi / 2;
            
            centerAngle = centerAngle + angleResTemp * motionDirection;
            
            % 路径点信息：位姿、速度、前轮转角、路径片段累计长度
            pathPoint = [x, y, yaw, motionDirection * paraConfig.rs.vRef, maxSteer, (i - 1) * angleResTemp * rMin];
            
            pathPoints = [pathPoints; pathPoint];
        end
    elseif motionType == 'R'
        centerAngle = yaw + pi / 2;
        xCenter = x - rMin * cos(centerAngle); % 转弯路径圆心坐标
        yCenter = y - rMin * sin(centerAngle);
    
        numPathPoints = ceil((motionLength / rMin) / angleRes) + 1; % 路径点个数
        angleResTemp = (motionLength / rMin) / (numPathPoints - 1); % 更新分辨率
        
        for i = 1 : numPathPoints
            x = xCenter + rMin * cos(centerAngle);
            y = yCenter + rMin * sin(centerAngle);
            yaw = centerAngle - pi / 2;
            
            centerAngle = centerAngle - angleResTemp * motionDirection;
            
            pathPoint = [x, y, yaw, motionDirection * paraConfig.rs.vRef, -maxSteer, (i - 1) * angleResTemp * rMin];
            
            pathPoints = [pathPoints; pathPoint];
        end
    else
        numPathPoints = ceil(motionLength / lineRes) + 1; % 路径点个数
        lineResTemp = motionLength / (numPathPoints - 1); % 更新分辨率
        
        for i = 1: numPathPoints
            dx = (i - 1) * lineResTemp * cos(yaw) * motionDirection;
            dy = (i - 1) * lineResTemp * sin(yaw) * motionDirection;
            
            pathPoint = [x + dx, y + dy, yaw, motionDirection * paraConfig.rs.vRef, 0.0, (i - 1) * lineResTemp];
            
            pathPoints = [pathPoints; pathPoint];
        end
        
        x = x + dx;
        y = y + dy;
    end
    
    startPose = [x, y, yaw];
end
