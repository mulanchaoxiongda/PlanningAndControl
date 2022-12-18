function [ref_traj] = CreatPathData(simulation_time, ego_speed)
    ref_traj = [];
    ref_point.t = 0;
    ref_point.x = 0;
    ref_point.y = 0;
    ref_point.theta = 0;
    ref_point.s = 0;
    ref_traj = [ref_traj; ref_point];
    dt = 0.1;
    while ref_point.t < simulation_time
        ref_point.t = ref_point.t + dt;
        ref_point.x = ref_point.x + ego_speed * cos(ref_point.theta) * dt;
        ref_point.y = ref_point.y + ego_speed * sin(ref_point.theta) * dt;
        ref_point.theta = ref_point.theta;
        ref_point.s = ref_point.s + ego_speed * dt;
        ref_traj = [ref_traj; ref_point];
    end
end