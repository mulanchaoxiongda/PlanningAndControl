function [obs_info] = MappingObsToSTGraph( ... % todo:本车和障碍车均以几何中心为基准进行检测与投影，存在原理误差，应以后轴中心为基准
    obs_traj, ref_traj, obs_para, ego_para)
    obs_info.st_overlap = [];
    obs_info.obs_type = 1; % 1:实障碍物 2：虚拟障碍物

    idx = -Inf;

    max_delta_x = ...
            ((obs_para.length / 2)^2 + (obs_para.width / 2)^2)^0.5 + ...
            ((ego_para.length / 2)^2 + (ego_para.width / 2)^2)^0.5;
    max_delta_y = max_delta_x;

    for i = 1 : 1 : length(obs_traj)
        for j = 2 : 1 : length(ref_traj)
            if j < idx % 不考虑对向行驶车辆
                continue;
            end

            delta_x = obs_traj(i).x - ref_traj(j).x;
            delta_y = obs_traj(i).y - ref_traj(j).y;

            if delta_x * cos(ref_traj(j).theta) + ...
               delta_y * sin(ref_traj(j).theta) > 0
                continue;
            else
                dis1 = ...
                        abs((obs_traj(i).x - ref_traj(j - 1).x) * ...
                        cos(ref_traj(j - 1).theta) + ...
                        (obs_traj(i).y - ref_traj(j - 1).y) * ...
                        sin(ref_traj(j - 1).theta));
                dis2 = ...
                        abs(delta_x * cos(ref_traj(i).theta) + ...
                        delta_y * sin(ref_traj(j).theta));

                match_point.x = ...
                        (ref_traj(j - 1).x * dis2 + ref_traj(j).x * dis1) / ...
                        (dis1 + dis2);
                match_point.y = ...
                        (ref_traj(j - 1).y * dis2 + ref_traj(j).y * dis1) / ...
                        (dis1 + dis2);
                match_point.theta = ...
                        (ref_traj(j - 1).theta * dis2 + ...
                        ref_traj(j).theta * dis1) / (dis1 + dis2);
                match_point.s = ...
                        (ref_traj(j - 1).s * dis2 + ref_traj(j).s * dis1) / ...
                        (dis1 +  dis2);

                shift_x = obs_traj(i).x - match_point.x;
                shift_y = obs_traj(i).y - match_point.y;

                if shift_x > max_delta_x || shift_y > max_delta_y
                    break;
                end

                dx1 = cos(match_point.theta) * ego_para.length / 2;
                dy1 = sin(match_point.theta) * ego_para.length / 2;
                dx2 = sin(match_point.theta) * ego_para.width / 2;
                dy2 = -cos(match_point.theta) * ego_para.width / 2;
                dx3 = cos(obs_traj(i).theta) * obs_para.length / 2;
                dy3 = sin(obs_traj(i).theta) * obs_para.length / 2;
                dx4 = sin(obs_traj(i).theta) * obs_para.width / 2;
                dy4 = -cos(obs_traj(i).theta) * obs_para.width / 2;

                if abs(shift_x * cos(match_point.theta) + ... % OBB-分离轴定理进行碰撞检测
                       shift_y * sin(match_point.theta)) <= ...
                   abs(dx3 * cos(match_point.theta) + ...
                       dy3 * sin(match_point.theta)) + ...
                   abs(dx4 * cos(match_point.theta) + ...
                       dy4 * sin(match_point.theta)) + ego_para.length /2 && ...
                   abs(shift_x * sin(match_point.theta) - ...
                       shift_y * cos(match_point.theta)) <= ...
                   abs(dx3 * sin(match_point.theta) - ...
                       dy3 * cos(match_point.theta)) + ...
                   abs(dx4 * sin(match_point.theta) - ...
                       dy4 * cos(match_point.theta)) + ego_para.width / 2 && ...
                   abs(shift_x * cos(obs_traj(i).theta) + ...
                       shift_y * sin(obs_traj(i).theta)) <= ...
                   abs(dx1 * cos(obs_traj(i).theta) + ...
                       dy1*sin(obs_traj(i).theta)) + ...
                   abs(dx2 * cos(obs_traj(i).theta) + ...
                       dy2 * sin(obs_traj(i).theta)) + obs_para.length / 2 && ...
                   abs(shift_x * sin(obs_traj(i).theta) - ...
                       shift_y * cos(obs_traj(i).theta)) <= ...
                   abs(dx1 * sin(obs_traj(i).theta) - ...
                       dy1 * cos(obs_traj(i).theta)) + ...
                   abs(dx2 * sin(obs_traj(i).theta) -  ...
                       dy2 * cos(obs_traj(i).theta)) + obs_para.width / 2
                    t = obs_traj(i).t;

                    delta_s = ...
                            ((obs_para.length / 2)^2 + ...
                            (obs_para.width / 2)^2)^0.5;
                    lb = match_point.s - delta_s;
                    ub = match_point.s + delta_s;
                    
                    lb = match_point.s - max_delta_x; % todo:对障碍车和本车尺寸的粗糙处理[待优化]
                    ub = match_point.s + max_delta_x;

                    obs_info.st_overlap = [obs_info.st_overlap; t, lb, ub];

                    idx  = j;
                end

                break;
            end
        end
    end
end
