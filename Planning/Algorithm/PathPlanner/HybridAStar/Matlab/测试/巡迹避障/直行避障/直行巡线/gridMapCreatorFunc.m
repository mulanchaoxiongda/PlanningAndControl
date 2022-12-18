%% 生成占用栅格地图
function gridMap = gridMapCreatorFunc(scenario, paraCfg)
    gridMap.xMin   = scenario.costMap.MapExtent(1); % 边界信息
    gridMap.xMax   = scenario.costMap.MapExtent(2);   
    gridMap.yMin   = scenario.costMap.MapExtent(3);
    gridMap.yMax   = scenario.costMap.MapExtent(4);
    gridMap.yawMin = -pi;
    gridMap.yawMax =  pi;    
    
    gridMap.xyResolution  = paraCfg.hAStar.xyPruneThreshold;  % 分辨率
    gridMap.yawResolution = paraCfg.hAStar.yawPruneThreshold; % 分辨率
    
    gridMap.gridOccupy = getCosts(scenario.costMap);   % 占用矩阵
end
