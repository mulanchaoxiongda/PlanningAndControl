%% 更新小车量测信息
function state_measured = UpdateRobotStateMeasured(state, L)
    gate_noise = 0;
    
    para_k = mod(state(1),1); % 漂移误差;若para_k取1,则为常值误差
    err_measure_x = 0.00*para_k;
    err_measure_y = -0.03*para_k;
    err_measure_psi = -2/57.3*para_k;
    err_measure_v = 0.0;
    err_measure_wz = 0;
    err_L = 0.0;
    
    gate_postion_measured = 0; % 0定位点为后轮中心1定位点为车体几何中心
    
    if gate_postion_measured==0
        state_measured(1) = state(1) + gate_noise*err_measure_x;  
        state_measured(2) = state(2) + gate_noise*err_measure_y;  
        state_measured(3) = state(3) + gate_noise*err_measure_psi;
        state_measured(4) = state(4) + gate_noise*err_measure_v;  
        state_measured(5) = state(5) + gate_noise*err_measure_wz; 
    elseif gate_postion_measured==1
        state_measured(1) = state(1) + 0.5*(L+err_L)*cos(state(3)) + gate_noise*err_measure_x; 
        state_measured(2) = state(2) + 0.5*(L+err_L)*sin(state(3)) + gate_noise*err_measure_y; 
        state_measured(3) = state(3) + gate_noise*err_measure_psi;                            
        state_measured(4) = state(4) + gate_noise*err_measure_v;                                % 简化处理，忽略角速度引起的线速度
        state_measured(5) = state(5) + gate_noise*err_measure_wz;                               
    end
    
    gate_delay = 0; % 定位信息延时开关
    
    if gate_delay==1
        persistent state_measured_storage;
        if isempty(state_measured_storage)
            state_measured_storage=0;
        end
        
        persistent counter;
        if isempty(counter)
            counter=1;
        end
        
        state_measured_storage(counter,1) = state_measured(1);
        state_measured_storage(counter,2) = state_measured(2);
        state_measured_storage(counter,3) = state_measured(3);
        state_measured_storage(counter,4) = state_measured(4);
        state_measured_storage(counter,5) = state_measured(5);
        
        counter = counter+1;
        
        N = 2; % 延迟时间N*Control_Time
        if counter>N
            state_measured(1) = state_measured_storage(counter-N,1);
            state_measured(2) = state_measured_storage(counter-N,2);
            state_measured(3) = state_measured_storage(counter-N,3);
            state_measured(4) = state_measured_storage(counter-N,4);
            state_measured(5) = state_measured_storage(counter-N,5);
        else
            state_measured(1) = state_measured_storage(1,1);
            state_measured(2) = state_measured_storage(1,2);
            state_measured(3) = state_measured_storage(1,3);
            state_measured(4) = state_measured_storage(1,4);
            state_measured(5) = state_measured_storage(1,5);
        end
    end    
end
