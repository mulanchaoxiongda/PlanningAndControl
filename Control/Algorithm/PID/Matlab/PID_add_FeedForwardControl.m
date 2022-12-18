%% 计算控制量:速度和方向盘转角
function [v_command, delta_f_command, idx_target] = PID_add_FeedForwardControl(RefTraj, state_measured, Control_Period, L, t)
	if t>=RefTraj(end,6)
        idx_target = length(RefTraj);
        
        RefPoint = [RefTraj(idx_target,1) RefTraj(idx_target,2) 0];
        
        Ref_x = RefTraj(end,1);
        Ref_y = RefTraj(end,3);
        Ref_Psi = RefTraj(end,3);
        Ref_Cur = RefTraj(end,5);
        Ref_Vel = RefTraj(end,4);
        Ref_wz = Ref_Vel*Ref_Cur;
    else
        SizeOfRefTraj = size(RefTraj,1);
        for i = 1:1:SizeOfRefTraj
    %         dist(i,1) = norm(RefTraj(i,1:2) - state_measured(1,1:2));
            dist(i,1) = abs(t-RefTraj(i,6));
        end
        [~,idx] = min(dist);

        if idx == 1
            idx_target = idx;
        elseif idx == SizeOfRefTraj
            idx_target = idx-1;
        else
            if dist(idx-1)<dist(idx+1)
                idx_target = idx-1;
            else
                idx_target = idx;
            end
        end
        
        dist1 = dist(idx_target);
        dist2 = dist(idx_target+1);
        
        RefPoint = ([RefTraj(idx_target,1) RefTraj(idx_target,2) 0]*dist2+[RefTraj(idx_target+1,1) RefTraj(idx_target+1,2) 0]*dist1)/(dist1+dist2);
        
        Ref_x = (RefTraj(idx_target,1)*dist2+RefTraj(idx_target+1,1)*dist1)/(dist1+dist2);
        Ref_y = (RefTraj(idx_target,2)*dist2+RefTraj(idx_target+1,2)*dist1)/(dist1+dist2);    
        Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
        Ref_Cur = (RefTraj(idx_target,5)*dist2+RefTraj(idx_target+1,5)*dist1)/(dist1+dist2);
        Ref_Vel = (RefTraj(idx_target,4)*dist2+RefTraj(idx_target+1,4)*dist1)/(dist1+dist2);
        Ref_wz = Ref_Vel*Ref_Cur;
    end
    
    Kp = 30; Ki0 = 5.0; Kd = 6.0;
    
    persistent wz_i_command;
    if isempty(wz_i_command)
        wz_i_command = 0;
    end
    
    err_y_command = 0;
    
    P = [state_measured(1) state_measured(2) 0];
    Q1 = RefPoint;
    
    while Ref_Psi >= 2*pi
        Ref_Psi = Ref_Psi-2*pi;
    end
    while Ref_Psi < 0
        Ref_Psi = Ref_Psi+2*pi;
    end
    
    if Ref_Psi>=0&&Ref_Psi<pi/2
        det_x = 1;
    elseif Ref_Psi>pi/2&&Ref_Psi<pi*3/2
        det_x = -1;
    elseif Ref_Psi>pi*3/2&&Ref_Psi<pi*2
        det_x = 1;
    end
    
    Q2 = [RefPoint(1)+det_x RefPoint(2)+det_x*tan(Ref_Psi) 0];
    
    err_y_vec = cross(Q1-P,Q2-Q1)/norm(Q2-Q1);
    err_y = err_y_vec(3);
    err_psi = state_measured(3)-Ref_Psi;
    
    if abs(Ref_Cur)<=0.1
        dis_i = 0.015;
    elseif abs(Ref_Cur)<=0.2
        dis_i = 0.015+0.04*(abs(Ref_Cur)-0.1)/(0.2-0.1);
    else
        dis_i = 0.055; % Todo
    end
    
    if abs(err_y)>=dis_i % Todo
        Ki = 0;
    else
        Ki = Ki0*abs(err_y)/dis_i;
    end
    
%     Ki = Ki0;
    if abs(err_y)<=dis_i
        Ki1 = 1.0;
    elseif abs(err_y)<=dis_i+0.02
        Ki1 = 1.0-(abs(err_y)-dis_i)/0.02;
    else
        Ki1 = 0.0;
    end
    
    CurVel_compensate = state_measured(4)*Ref_Cur;
    
    wz_i_command = wz_i_command+Ki*(err_y_command-err_y)*Control_Period;
    wz_i_command = wz_i_command*Ki1;
    
    wz_command = Kp*(err_y_command-err_y)+wz_i_command-Kd*err_psi+CurVel_compensate*1.0;
    
    delta_f_command = atan(wz_command*L/state_measured(4));
    if abs(delta_f_command)>85/57.3
        delta_f_command = 85/57.3*sign(delta_f_command);
    end
    
    UnitDirectionVector_RefPoint = [cos(Ref_Psi) sin(Ref_Psi)]';
    
    RelativePosition_Robot_RefPoint = [RefPoint(1)-state_measured(1) RefPoint(2)-state_measured(2)]';
    
    err_sx = -dot(RelativePosition_Robot_RefPoint,UnitDirectionVector_RefPoint); % 近似解 % 轨迹系x轴偏差
    
    persistent err_sx_prevalue;
    if isempty(err_sx_prevalue)
        err_sx_prevalue = err_sx;
    end
    
    Kp = 6.0; Ki = 0.0; Kd = 0.5;
    v_command_PID = Kp*(0-err_sx)+Kd*(err_sx-err_sx_prevalue)/Control_Period;
    err_sx_prevalue = err_sx;
    v_command = Ref_Vel+v_command_PID;
end
