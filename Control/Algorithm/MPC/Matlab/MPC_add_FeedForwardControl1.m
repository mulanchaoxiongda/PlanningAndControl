%% 计算控制量:速度和方向盘转角
function [wz_command, v_command, delta_f_command, idx_target] = MPC_add_FeedForwardControl1(RefTraj, state_measured, L, t)
    Nx = 4;
    Nu = 2;
    Np = 15;
    Nc = 3;
    Nk = 2;
    
    Control_Period = 0.12;
    
    row = diag([2 2]);
    Q = kron(eye(Np),diag([8 30 1.0 0.1]));
    R = kron(eye(Nc),diag([8.0 0.2]));

    umin = [-1.0; -20/57.3];
    umax = [1.0; 20/57.3];
    
    delta_umin = [-1.0; -20/57.3]*Control_Period;
    delta_umax = [1.0; 20/57.3]*Control_Period;
    
    Gam_Min = [0; 0];
    Gam_Max = [0.5; 1.0];

    if t >= RefTraj(end,6)
        idx_target = length(RefTraj);
        
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
            dist(i,1) = abs(t - RefTraj(i,6));
        end
        [~,idx] = min(dist);

        if idx == 1
            idx_target = idx;
        elseif idx == SizeOfRefTraj
            idx_target = idx - 1;
        else
            if dist(idx - 1) < dist(idx + 1)
                idx_target = idx - 1;
            else
                idx_target = idx;
            end
        end
        
        dist1 = dist(idx_target);
        dist2 = dist(idx_target + 1);
        
        Ref_x = (RefTraj(idx_target,1) * dist2 + RefTraj(idx_target + 1,1) * dist1) / (dist1 + dist2);
        Ref_y = (RefTraj(idx_target,2) * dist2 + RefTraj(idx_target + 1,2) * dist1) / (dist1 + dist2);    
        Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
        Ref_Cur = (RefTraj(idx_target,5)*dist2+RefTraj(idx_target+1,5)*dist1)/(dist1+dist2);
        Ref_Vel = (RefTraj(idx_target,4)*dist2+RefTraj(idx_target+1,4)*dist1)/(dist1+dist2);
        Ref_wz = Ref_Vel*Ref_Cur;
    end
 
    w_bound_compesation = state_measured(4)*Ref_Cur;
    
    umin = umin-[Ref_Vel; w_bound_compesation];
    umax = umax-[Ref_Vel; w_bound_compesation];
    
    Xr = [Ref_x Ref_y Ref_Psi Ref_wz]';
    X = [state_measured(1) state_measured(2) state_measured(3) state_measured(5)]';
 
    T1 = 0.07;
    a = [1, 0, -Ref_Vel * Control_Period*sin(Ref_Psi), 0;
         0, 1,  Ref_Vel * Control_Period*cos(Ref_Psi), 0;
         0, 0,  1,                                     Control_Period;
         0, 0,  0,                                     1 - Control_Period / T1];
     
    b = [Control_Period * cos(Ref_Psi), 0;
         Control_Period * sin(Ref_Psi), 0;
         0,                             0;
         0,                             Control_Period / T1];
         
    persistent U;
    if isempty(U)
        U = [state_measured(4) - Ref_Vel, state_measured(5) - Ref_wz]';
    end    
    
    kesi = [X-Xr;U];
    
    A = cell2mat({a b; zeros(Nu,Nx) eye(Nu)});
    B = cell2mat({b; eye(Nu)});
    C = [eye(Nx), zeros(Nx,Nu)];

    PHI_cell = cell(Np,1);
    for i = 1:Np
        PHI_cell{i,1}=C*A^i;
    end
    
    PHI = cell2mat(PHI_cell);
    
    THETA_cell = cell(Np,Nc);
    for i = 1:Np
        for j = 1:Nc
            if j <= i
                THETA_cell{i,j} = C*A^(i-j)*B;
            else
                THETA_cell{i,j} = zeros(Nx,Nu);
            end
        end
    end
    THETA = cell2mat(THETA_cell);
    

    H = THETA'*Q*THETA+R;
    %H = cell2mat({THETA'*Q*THETA+R zeros(Nu*Nc,Nk); zeros(Nk,Nu*Nc) row)});
    H = (H+H')/2;
    E = PHI*kesi;
    g = (E'*Q*THETA)';
    %g = cell2mat({E'*Q*THETA 0 0});
    
    A_t = zeros(Nc,Nc);
    for i = 1:Nc
        A_t(i,1:i) = 1;
    end
    A_I = kron(A_t,eye(Nu));
    
    Ut = kron(ones(Nc,1),U);
    
    Umin = kron(ones(Nc,1),umin);
    Umax = kron(ones(Nc,1),umax);
    delta_Umin = kron(ones(Nc,1),delta_umin);
    delta_Umax = kron(ones(Nc,1),delta_umax);
    
    A_cons = [A_I;-A_I];
    %A_cons = cell2mat({A_I, zeros(Nu*Nc,Nk); -A_I, zeros(Nu*Nc,Nk)});
    b_cons = [Umax-Ut; -Umin+Ut];
    lb = delta_Umin;
    %lb = [delta_Umin; Gam_Min];
    ub = delta_Umax;
    %ub = [delta_Umax; Gam_Max];
    
    options = optimoptions('quadprog','MaxIterations',200,'TolFun',1e-6,'Display','off');
    delta_U = quadprog(H,g,A_cons,b_cons,[],[],lb,ub,[],options);

    delta_v_tilde = delta_U(1);
    delta_wz_tilde = delta_U(2);

    U(1) = kesi(5) + delta_v_tilde;
    U(2) = kesi(6) + delta_wz_tilde;

    CurVel_compensate = state_measured(4)*Ref_Cur;
    wz_command = U(2)+CurVel_compensate;
    
    if(abs(state_measured(4))<0.01)
        state_measured(4) = 0.01*sign(state_measured(4));
    end
    delta_f_command = atan(wz_command*L/state_measured(4));
    if abs(delta_f_command)>80/57.3
        delta_f_command = 80/57.3*sign(delta_f_command);
    end
    
    v_command = U(1)+Ref_Vel;
end
