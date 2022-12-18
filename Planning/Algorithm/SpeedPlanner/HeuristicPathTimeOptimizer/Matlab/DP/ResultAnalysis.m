function [] = ResultAnalysis( ...
        st_graph, obs_info, opt_st_path, closed_list, ...
        open_list, ego_para, planning_para)
    figure('name','st graph');
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
                    obs_info(i).st_overlap(1,1)];

            obs_frame_s = ...
                    [obs_info(i).st_overlap(1 : end, 2);
                    obs_info(i).st_overlap(end : -1 : 1, 3)
                    obs_info(i).st_overlap(1, 2)];

            fill(obs_frame_t, obs_frame_s, 'r'); hold on;
        end
    end

    max_lon_controller_err = planning_para.max_lon_control_err;

    plot(opt_st_path(:, 1), opt_st_path(:, 2), 'm', ...
         opt_st_path(:, 1), opt_st_path(:, 2) + max_lon_controller_err, '--m', ...
         opt_st_path(:, 1), opt_st_path(:, 2) - max_lon_controller_err, '--m');
    plot(closed_list(:, 5), closed_list(:, 6),'oblack');
    plot(open_list(:, 5), open_list(:, 6), 'oyellow');

    figure('name', 'result analysis');
    subplot(3, 1, 1);
    plot(opt_st_path(:, 1), opt_st_path(:, 3) * 3.6, 'm'); grid on;
    xlabel('t(s)'); ylabel('v(km/h)');
    legend('heuristic v-t curve', 'dp v-t curve'); title('v-t curve');
    subplot(3, 1, 2);
    plot(opt_st_path(:, 1), opt_st_path(:, 4), 'm'); grid on;
    xlabel('t(s)'); ylabel('acc(m/s/s)');
    legend('heuristic a-t curve', 'dp v-t curve'); title('a-t curve');
    subplot(3, 1, 3);
    plot(opt_st_path(:, 1), opt_st_path(:, 5), 'm'); grid on;
    xlabel('t(s)'); ylabel('jerk(m/s/s/s)');
    legend('heuristic jerk-t curve', 'dp jerk-t curve'); title('jerk-t curve');
    
    figure('name', 'result analysis');
    subplot(3, 1, 1);
    plot(opt_st_path(:, 2), opt_st_path(:, 3) * 3.6, 'm'); grid on;
    xlabel('s(m)'); ylabel('v(km/h)');
    legend('heuristic v-s curve', 'dp v-s curve'); title('v-s curve');
    subplot(3, 1, 2);
    plot(opt_st_path(:, 2), opt_st_path(:, 4), 'm'); grid on;
    xlabel('s(m)'); ylabel('acc(m/s/s)');
    legend('heuristic a-s curve', 'dp v-s curve'); title('a-s curve');
    subplot(3, 1, 3);
    plot(opt_st_path(:, 2), opt_st_path(:, 6), 'm'); grid on;
    xlabel('s(m)'); ylabel('jerk(rad/m/)');
    legend('kappa-s curve'); title('kappa - s');
end
