function [] = ResultAnalysis( ...
        st_graph, obs_info, opt_st_path, ...
        closed_list, open_list, ego_para, speed_data, planning_para)
    figure('name', 'st_graph');
% % %     for i = 1 : 1 : length(st_graph.t)
% % %         for j = 1 : 1 : length(st_graph.s)
% % %             plot(st_graph.t(i), st_graph.s(j), '*b'); hold on;
% % %         end
% % %     end
    xlabel('t(s)'); ylabel('s(m)');
    axis([st_graph.t(1), st_graph.t(end), st_graph.s_dense(end), st_graph.s_dense(1)]);

    if ~isempty(obs_info(1).st_overlap)
        for i = 1 : 1 : length(obs_info)
            obs_frame_t = ...
                    [obs_info(i).st_overlap(1 : end, 1); ...
                     obs_info(i).st_overlap(end : -1 : 1, 1); ...
                     obs_info(i).st_overlap(1, 1)];

            obs_frame_s = ...
                [obs_info(i).st_overlap(1 : end, 2); ...
                 obs_info(i).st_overlap(end : -1 : 1, 3); ...
                 obs_info(i).st_overlap(1, 2)];
             
            fill(obs_frame_t, obs_frame_s, 'r'); hold on;
        end
    end
    
    max_lon_controller_err = planning_para.max_lon_control_err;
    
    plot(opt_st_path(:, 1), opt_st_path(:, 2), 'm', ...
         opt_st_path(:, 1), opt_st_path(:, 2) + max_lon_controller_err, '--m', ...
         opt_st_path(:, 1), opt_st_path(:, 2) - max_lon_controller_err, '--m'); hold on;
    plot(closed_list(:, 5), closed_list(:, 6), 'oblack'); hold on;
    plot(open_list(:, 5), open_list(:, 6), 'oyellow'); hold on;
    
    max_control_err = 2.5;
    dis_margin = 0.1;
    ds = max_control_err + dis_margin;
    
    plot(speed_data.t, speed_data.s, 'b', ...
         speed_data.t, speed_data.s + ds, '.-b', ...
         speed_data.t, speed_data.s - ds, '-b');
    title('s - t graph')
     legend('obstacle', '', '', 'heuristic s - t curve', '', '', '', '','qp s - t curve', '', '');
%     legend('heuristic s - t curve','qp s - t curve');
    
    figure('name', 'result analysis');
    subplot(3, 1, 1);
    plot(opt_st_path(:, 1), opt_st_path(:, 3) * 3.6, 'm'); grid on; hold on;
    plot(speed_data.t, speed_data.ds * 3.6, 'b');
    legend('heuristic v - t curve', 'qp v - t curve');
    xlabel('t(s)');ylabel('v(km/h)');title('speed -time');
    subplot(3, 1, 2);
	plot(opt_st_path(:, 1), opt_st_path(:, 4), 'm'); grid on; hold on;
    plot(speed_data.t, speed_data.dds, 'b');
    legend('heuristic a - t curve');
    xlabel('t(s)'); ylabel('acc(m/s/s)'); title('acc - time');
    subplot(3, 1, 3);
    plot(opt_st_path(:, 1), opt_st_path(:, 5), 'm'); grid on;
    legend('heuristic j - t curve'); title('jerk - time');
    xlabel('t(s)'); ylabel('jerk(m/s/s/s)');
    
    figure('name', 'result analysis');
    subplot(3, 1, 1);
    plot(opt_st_path(:, 2), opt_st_path(:, 3) * 3.6, 'm'); grid on; hold on;
    plot(speed_data.s, speed_data.ds * 3.6, 'b');
    legend('heuristic v - s curve', 'qp v-s curve'); title ('speed - s');
    xlabel('s(m)');ylabel('v(m/s)');
    subplot(3, 1, 2);
    plot(opt_st_path(:, 2), opt_st_path(:, 4), 'm'); grid on; hold on;
    plot(speed_data.s, speed_data.dds, 'b');
    legend('heuristic a - s curve', 'qp a - s curve'); title('acc - s');
    xlabel('s(m)'); ylabel('acc(m/s/s)');
    subplot(3, 1, 3);
    plot(opt_st_path(:, 2), opt_st_path(:, 6), 'b'); grid on;
    title('kappa - s');
    xlabel('s(m)'); ylabel('kappa(rad/m/m)');
end
