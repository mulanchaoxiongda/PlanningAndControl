%% 计算控跟踪误差:位置误差和姿态误差
function [TrackingErrorOfPosition, TrackingErrorOfAttitude] = CalculateTrackingError(RefTraj, state)
	SizeOfRefTraj = size(RefTraj,1);
    for i = 1:1:SizeOfRefTraj
        dist(i,1) = norm(RefTraj(i,1:2) - state(1,1:2));   
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
    Ref_Psi = (RefTraj(idx_target,3)*dist2+RefTraj(idx_target+1,3)*dist1)/(dist1+dist2);
    P = [state(1) state(2) 0];
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
    TrackingErrorOfPosition = err_y_vec(3);
    TrackingErrorOfAttitude = state(3)-Ref_Psi;
end