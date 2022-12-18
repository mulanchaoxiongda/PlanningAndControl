%% 更新小车状态量
function state_new = UpdateRobotState(wz_command, v_command,delta_f_command,state_old,dt,L)
    err_L = 0.0000001; 
    err_delta_f0 = 0.000000001/57.3;
    
    Tv = 0.10; a = (v_command-state_old(4))/Tv;                         
    
    persistent delta_f;
    if isempty(delta_f)
        delta_f = atan(state_old(5)*(L+err_L)/state_old(4))-err_delta_f0;
    end
    
    Tw = 0.15; 
    d_delta_f = (delta_f_command-delta_f)/Tw; 
    delta_f = delta_f + d_delta_f*dt;
    
    state_new(1) = state_old(1) + state_old(4)*cos(state_old(3))*dt;                      
    state_new(2) = state_old(2) + state_old(4)*sin(state_old(3))*dt;                      
    state_new(3) = state_old(3) + tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L)*dt;
    state_new(4) = state_old(4) + a*dt;                                              
    state_new(5) = tan(delta_f+err_delta_f0)*state_old(4)/(L+err_L);
end
