%% 画图函数
function visualizationFunc(scenario, paraConfig, gridMap, refPath, openList, closedList, smooth_line)
    x = refPath(:, 1);
    y = refPath(:, 2);
    theta = refPath(:, 3);
    
    sz=get(0, 'screensize');
    figure('outerposition', sz);
    
    videoFWriter = VideoWriter('Parking.mp4', 'MPEG-4');
    open(videoFWriter);
   
    plot(scenario.costMap, 'Inflation', 'off'); hold on;
    xlim([gridMap.xMin, gridMap.xMax]); ylim([gridMap.yMin, gridMap.yMax]);
    
    plot(scenario.startPose(1), scenario.startPose(2), 'go', 'MarkerSize', 12); % 绘制起始点--后轴中心
    plot(paraConfig.goalPose(1), paraConfig.goalPose(2), 'go', 'MarkerSize', 12); % 绘制停车点--后轴中心
    
    plot(smooth_line(:, 1), smooth_line(:, 2), 'r'); hold on
    
    plot(refPath(:, 1), refPath(:, 2), 'b'); % 绘制规划路径
    
    x = refPath(1, 1);
    y = refPath(1, 2);
    yaw = refPath(1, 3);
    
    [xVehBound, yVehBound] = calVehicleConner(x, y, yaw, paraConfig.vehicle); % 根据后轴中心的位姿计算车辆边框的位姿
    
    h1 = plot(xVehBound, yVehBound, 'k'); % 车辆边框
    
    h2 = plot(x, y, 'r*', 'MarkerSize', 10); % 车辆后轴中心
       
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    
    for i = 2:length(theta)
        x = refPath(i, 1);
        y = refPath(i, 2);
        yaw = refPath(i, 3);
    
        [xVehBound, yVehBound] = calVehicleConner(x, y, yaw, paraConfig.vehicle);
        
        h1.XData = xVehBound; % 更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
        h1.YData = yVehBound;
        
        h2.XData = x;         % 更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
        h2.YData = y;
        
%         pause(0.005);
    
        if mod(i-1, 20) == 0
            img = getframe(gcf);
            writeVideo(videoFWriter,img);
        end
    end
    
%     for i = 1 : length(openList) 
%         scatter(openList(i).pose(1), openList(i).pose(2), '.c');
%     end
%     
%     for i = 1 : length(closedList) 
%         scatter(closedList(i).pose(1), closedList(i).pose(2), '.g');
%     end
    
    close(videoFWriter);
end

 %% 根据后轴中心的位姿计算车辆边框的位置
function [xVehBound, yVehBound] = calVehicleConner(x, y, yaw, vehCfg)
    cornerFL = [ vehCfg.rear2front,  vehCfg.wid / 2, 0]'; % 车辆四个角坐标--车体系
    cornerFR = [ vehCfg.rear2front, -vehCfg.wid / 2, 0]';
    cornerRL = [-vehCfg.rear2back,   vehCfg.wid / 2, 0]';
    cornerRR = [-vehCfg.rear2back,  -vehCfg.wid / 2, 0]';
    
    posRearCenter = [x, y, 0]'; % 后轴中心坐标--全局系
    
    transMatrix = [cos(yaw) sin(yaw) 0;   % 转换矩阵--本体系到全局系
                  -sin(yaw) cos(yaw) 0;
                   0        0        1]';
    
    cornerFL = transMatrix * cornerFL + posRearCenter; % 坐标变化--车体系到全局系
    cornerFR = transMatrix * cornerFR + posRearCenter;
    cornerRL = transMatrix * cornerRL + posRearCenter;
    cornerRR = transMatrix * cornerRR + posRearCenter;
    
    xVehBound = [cornerFL(1), cornerFR(1), cornerRR(1), cornerRL(1), cornerFL(1)];
    yVehBound = [cornerFL(2), cornerFR(2), cornerRR(2), cornerRL(2), cornerFL(2)];
end
