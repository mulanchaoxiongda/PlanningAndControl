%% 配置参数
function cfg = paraConfigFunc(scenario)
    % 混合A*参数
    hAStar.maxSearchTimes    = 2000;                                      % 最大搜索次数
    hAStar.vRef              = 0.6;                                       % 参考速度
    hAStar.lineResolution    = 0.01;                                      % 线运动分辨率
    hAStar.angleResolution   = 5.0 / 57.3;                               % 前轮转角分辨率
    hAStar.forwardCost       = 1.0;                                       % 前进代价权重
    hAStar.backCost          = 1.0;                                       % 后退代价权重
    hAStar.switchCost        = 0.0;                                       % 换向代价权重
    hAStar.steerCost         = 1.0;                                       % 转向代价权重
    hAStar.steerChangeCost   = 1.0;                                       % 转向变化代价权重
    hAStar.hCost             = 1.0;                                       % 启发代价权重--建议取1.0，影响见optSearchStrategy（）
    hAStar.maxSteer          = scenario.vehicle.maxSteer - 5.0 / 57.3;    % 最大前轮转角留有5°余量
    hAStar.searchStep        = scenario.costMap.CellSize * 1.5;           % 搜索步长
    hAStar.numSteer          = hAStar.maxSteer / hAStar.angleResolution;  % 每次搜索生成的路径数--前轮转角 numSteer * 2 + 1
    hAStar.xyPruneThreshold  = scenario.costMap.CellSize;                 % 剪枝阈值
    hAStar.yawPruneThreshold = hAStar.searchStep /...                     % 剪枝阈值--角度：
                               (scenario.vehicle.wheelBase /...
                               tan(hAStar.angleResolution)) * 0.95;
    hAStar.disBeginRSSearch  = 1.0;                                       % 启用RS搜索的初始曼哈顿相对距离--建议取值7~9m
    
    cfg.hAStar = hAStar;
    
    % RS曲线参数
    rs.maxSteer      = hAStar.maxSteer;                               % 最大前轮转角留有5°余量
    rs.rMin          = scenario.vehicle.wheelBase / tan(rs.maxSteer); % 最小转弯半径
    rs.vRef          = hAStar.vRef;                                   % 参考速度
    rs.lineMotionRes = hAStar.lineResolution;                         % 线运动分辨率
    rs.angMotionRes  = rs.lineMotionRes / rs.rMin;                    % 角运动分辨率--车辆相对转向圆心角度
    
    cfg.rs = rs;
    
    % 碰撞检测参数
    collisionDet.interval     = scenario.costMap.CellSize; % 检测点间距--栅格地图分辨率--需要为rs.lineMotionRes和hAStar.lineResolution的整数倍
    collisionDet.lenExpansion = 0.1;                       % 车身膨胀长度
    collisionDet.widExpansion = 0.05;                       % 车身膨胀宽度
    
    cfg.collisionDet = collisionDet;
    
    % 停车点位置
    goalPose = scenario.goalPose; % 停车点位置--车辆前悬中心位置反算后轴中心位置
    
    cfg.goalPose = goalPose;
    
    % 虚拟停车点前置长度
    cfg.disFrontParking = 0.0; % 停车起始位置可行域最大化
    
    % 栅格地图参数
    gridMap.xyResolution  = scenario.costMap.CellSize; % 地图分辨率
    
    cfg.gridMap = gridMap;
    
	% 奥迪A6L参数
    vehicle.wheelBase  = scenario.vehicle.wheelBase;  % 轴距
    vehicle.wid        = scenario.vehicle.wid;        % 车宽
    vehicle.rear2front = scenario.vehicle.rear2front; % 后轮-前悬距离
    vehicle.rear2back  = scenario.vehicle.rear2back;  % 后轮-后悬距离
    vehicle.maxSteer   = scenario.vehicle.maxSteer;   % 最大前轮转角（不能用满，给控制留余量）
    vehicle.minCircle  = scenario.vehicle.minCircle;  % 最小转弯半径
    
    cfg.vehicle = vehicle;
end
