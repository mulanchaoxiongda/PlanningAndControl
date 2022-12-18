function [ref_line] = MySmoothRefLine(routing_line)
    %% 重采样参考线点，关键参数[len_line:参考线总长度，len_fragment:片段长度，interval_sampling:采点间隔]
    routing_line.s = [0];
    s = 0;
    for i = 2 : 1 : length(routing_line.x)
        s = s + ((routing_line.x(i) - routing_line.x(i - 1))^2 + (routing_line.y(i) - routing_line.y(i - 1))^2)^0.5;
        routing_line.s = [routing_line.s; s];
    end

    len_line = 4.0; % 参考线总长度
    len_fragment = 1.0; % 片段长度
    interval_sampling = 0.2; % 采点间隔

    routing_line.X = []; % 重采点后的routing_line信息
    routing_line.Y = [];
    routing_line.Theta = [];
    routing_line.S = [];

    for s = 0 : interval_sampling : len_line
        X = interp1(routing_line.s, routing_line.x, s, 'linear'); % 邻近点插值Nearest，线性插值Linear，三次样条插值Spline，立方插值Pchip
        Y = interp1(routing_line.s, routing_line.y, s, 'linear');
        Theta = interp1(routing_line.s, routing_line.theta, s, 'linear');

        routing_line.X = [routing_line.X; X];
        routing_line.Y = [routing_line.Y; Y];
        routing_line.Theta = [routing_line.Theta; Theta];
        routing_line.S = [routing_line.S; s];
    end
    
    plot(routing_line.X(:), routing_line.Y(:), 'og'); hold on; axis equal;

    %% 构造cost函数，关键参数[H:阵，f:阵]
    nums_fragment = len_line / len_fragment; % 片段数量

    H = zeros(12 * nums_fragment, 12 * nums_fragment); % 初始化H矩阵
    f = zeros(12 * nums_fragment, 1);                  % 初始化f矩阵

    Weight1 = 0.5; % x向加速度项权重
    Weight2 = 0.2; % x向加加速度项权重
    Weight3 = 2000; % x向位置误差项权重

    weight1 = 0.5; % y向
    weight2 = 0.2;
    weight3 = 2000;
    
    Weight1 = 0.01; % x向加速度项权重
    Weight2 = 0.02; % x向加加速度项权重
    Weight3 = 2000; % x向位置误差项权重

    weight1 = 0.01; % y向
    weight2 = 0.02;
    weight3 = 2000;

    for i = 1 : nums_fragment
        s = len_fragment; % 每一段5次曲线的终点时刻

        Hi1 = 2 * [ 0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0        4*s       6*s^2        8*s^3        10*s^4
                    0       0       6*s^2      12*s^3      18*s^4        24*s^5
                    0       0       8*s^3      18*s^4    (144/5)*s^5     40*s^6
                    0       0      10*s^4      24*s^5      40*s^6     (400/7)*s^7 ]; % 加速度项

        hi1 = 2 * [ 0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0        4*s       6*s^2        8*s^3        10*s^4
                    0       0       6*s^2      12*s^3      18*s^4        24*s^5
                    0       0       8*s^3      18*s^4    (144/5)*s^5     40*s^6
                    0       0      10*s^4      24*s^5      40*s^6     (400/7)*s^7 ];

        Hi2 = 2 * [ 0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0         0       36*s         72*s^2        120*s^3
                    0       0         0      72*s^2       192*s^3        360*s^4
                    0       0         0     120*s^3       360*s^4        720*s^5 ]; % 加加速项

        hi2 = 2 * [ 0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0         0         0            0             0
                    0       0         0       36*s         72*s^2        120*s^3
                    0       0         0      72*s^2       192*s^3        360*s^4
                    0       0         0     120*s^3       360*s^4        720*s^5 ];

        Hi3 = 2 * [ 1       s       s^2       s^3          s^4           s^5
                    s      s^2      s^3       s^4          s^5           s^6
                   s^2     s^3      s^4       s^5          s^6           s^7
                   s^3     s^4      s^5       s^6          s^7           s^8
                   s^4     s^5      s^6       s^7          s^8           s^9
                   s^5     s^6      s^7       s^8          s^9           s^10 ]; % 误差项

        hi3 = 2 * [ 1       s       s^2       s^3          s^4           s^5
                    s      s^2      s^3       s^4          s^5           s^6
                   s^2     s^3      s^4       s^5          s^6           s^7
                   s^3     s^4      s^5       s^6          s^7           s^8
                   s^4     s^5      s^6       s^7          s^8           s^9
                   s^5     s^6      s^7       s^8          s^9           s^10 ];

        Hi = Weight1 * Hi1 + Weight2 * Hi2 + Weight3 * Hi3;
        hi = weight1 * hi1 + weight2 * hi2 + weight3 * hi3;

        tempH = zeros(12, 12);

        tempH(1 : 6, 1 : 6) = Hi;
        tempH(7 : 12, 7 : 12) = hi;

        idx1 = (i - 1)*12 + 1;
        idx2 = (i - 1)*12 + 12;

        H(idx1 : idx2, idx1 : idx2) = tempH; % H:阵

        xI = routing_line.X(1 + 5 * i);
        yI = routing_line.Y(1 + 5 * i);

        Fi = -2 * xI * [1, s, s^2, s^3, s^4, s^5] * Weight3;
        fi = -2 * yI * [1, s, s^2, s^3, s^4, s^5] * weight3;

        f(idx1 : idx2) = [Fi'; fi']; % f:阵
    end

    %% 构造等式约束
    Aequ = zeros(6 + 6 * (nums_fragment - 1) + 6, 12 * nums_fragment); % 初始化Aequ矩阵
    Bequ = zeros(6 + 6 * (nums_fragment - 1) + 6, 1);                  % 初始化Bequ矩阵

    s = 0; % 起点约束
    
    A_eq_i1 = zeros(3, 12);
    A_eq_i1(1 : 3, 1 : 6) = [ 1        s        s^2        s^3        s^4        s^5
                              0        1        2 * s      3 * s^2    4 * s^3    5 * s^4
                              0        0        2          6 * s      12 * s^2   20 * s^3 ];

    a_eq_i1 = zeros(3, 12);
    a_eq_i1(1 : 3, 7 : 12) = [ 1        s        s^2        s^3        s^4        s^5
                               0        1        2 * s      3 * s^2    4 * s^3    5 * s^4
                               0        0        2          6 * s      12 * s^2   20 * s^3 ];

	Aequ(1 : 6, 1 : 12) = [A_eq_i1; a_eq_i1];

    pos.x = routing_line.X(1);
    vel.x = cos(routing_line.Theta(1)); %%%%%%%
    acc.x = 0;
    pos.y = routing_line.Y(1);
    vel.y = sin(routing_line.Theta(1)); %%%%%%%
    acc.y = 0;

    Bequ(1 : 6, 1) = [pos.x, vel.x, acc.x, pos.y, vel.y, acc.y]';

	s = len_fragment; % 终点约束
    
    A_eq_i2 = zeros(3, 12);
    A_eq_i2(1 : 3, 1 : 6) = [ 1        s        s^2        s^3        s^4        s^5
                              0        1        2 * s      3 * s^2    4 * s^3    5 * s^4
                              0        0        2          6 * s      12 * s^2   20 * s^3 ];

    a_eq_i2 = zeros(3, 12);
    a_eq_i2(1 : 3, 7 : 12) = [ 1        s        s^2        s^3        s^4        s^5
                               0        1        2 * s      3 * s^2    4 * s^3    5 * s^4
                               0        0        2          6 * s      12 * s^2   20 * s^3 ];

    Aequ(6 + 6 * (nums_fragment - 1) + 1 : 6 + 6 * (nums_fragment - 1) + 6, 12 * (nums_fragment - 1) + 1 : 12 * nums_fragment) = [A_eq_i2; a_eq_i2];

    pos.x = routing_line.X(end);
    vel.x = cos(routing_line.Theta(end)); %%%%%%%
    acc.x = 0;
    pos.y = routing_line.Y(end);
    vel.y = sin(routing_line.Theta(end)); %%%%%%%
    acc.y = 0;

    Bequ(6 + 6 * (nums_fragment - 1) + 1 : 6 + 6 * (nums_fragment - 1) + 6, 1) = [pos.x, vel.x, acc.x, pos.y, vel.y, acc.y]';

    for i = 1 : 1 : nums_fragment - 1
        A_eq_i3 = zeros(3, 24); % 中间点连续性约束
        a_eq_i3 = zeros(3, 24);

        s = len_fragment; % x轴位置、速度、加速度连续性
        A_eq_i3(1 : 3, 1 : 6) = [ 1        s        s^2        s^3        s^4        s^5        
                                  0        1        2 * s      3 * s^2    4 * s^3    5 * s^4    
                                  0        0        2          6 * s      12 * s^2   20 * s^3 ];
        s = 0;
        A_eq_i3(1 : 3, 13 : 18) = [ 1        s        s^2        s^3        s^4        s^5        
                                    0        1        2 * s      3 * s^2    4 * s^3    5 * s^4    
                                    0        0        2          6 * s      12 * s^2   20 * s^3 ] * (-1);
    
        s = len_fragment; % y轴位置、速度、加速度连续性
        a_eq_i3(1 : 3, 7 : 12) = [ 1        s        s^2        s^3        s^4        s^5
                                   0        1        2 * s      3 * s^2    4 * s^3    5 * s^4
                                   0        0        2          6 * s      12 * s^2   20 * s^3 ];
        s = 0;
        a_eq_i3(1 : 3, 19 : 24) = [ 1        s        s^2        s^3        s^4        s^5        
                                    0        1        2 * s      3 * s^2    4 * s^3    5 * s^4    
                                    0        0        2          6 * s      12 * s^2   20 * s^3 ] * (-1);

        Aequ(6 * i + 1 : 6 * i + 6, 12 * (i - 1) + 1 : 12 * (i - 1) + 24) = [A_eq_i3; a_eq_i3];
    end

   %% 构造不等式约束
   amount_frag = len_fragment / interval_sampling; % 每个片段内采点个数

    Ainequ = zeros((2 + 2) * (nums_fragment * amount_frag - 1), 12 * nums_fragment);
    Binequ = zeros((2 + 2) * (nums_fragment * amount_frag - 1), 1);

    err_x = 0.01; % 约束边界
    err_y = 0.01;

    for i = 1 : 1 : nums_fragment
        for j = 1 : 1 : amount_frag
            A_in_is = zeros(2, 12);
            a_in_is = zeros(2, 12);

            s = j / amount_frag * len_fragment;

            A_in_is(1 : 2, 1 : 6)  = [ 1,     s,     s^2,     s^3,     s^4,     s^5    
                                      -1,    -s,    -s^2,    -s^3,    -s^4,    -s^5 ]; % x向位置
            a_in_is(1 : 2, 7 : 12) = [ 1,     s,     s^2,     s^3,     s^4,     s^5    
                                      -1,    -s,    -s^2,    -s^3,    -s^4,    -s^5 ]; % y向位置

            Ainequ(amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 1 : amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 2, 12 * (i - 1) + 1 : 12 * i) = A_in_is;
            Ainequ(amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 3 : amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 4, 12 * (i - 1) + 1 : 12 * i) = a_in_is;

            x_I = routing_line.X(amount_frag * (i - 1) + j + 1);
            B_in_is = [x_I + err_x; -x_I + err_x];

            y_I = routing_line.Y(amount_frag * (i - 1) + j + 1);
            b_in_is = [y_I + err_y; -y_I + err_y];

            Binequ(amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 1 : amount_frag * (2 + 2) * (i - 1) + (2 + 2) * (j - 1) + 4, 1) = [B_in_is; b_in_is]; %%%%%%%
        end
    end
    
    Ainequ = Ainequ(1 : (2 + 2) * nums_fragment * amount_frag - (2 + 2), 1 : 12 * nums_fragment);
    Binequ = Binequ(1 : (2 + 2) * nums_fragment * amount_frag - (2 + 2), 1 : 1);

    %% 二次规划求解器
    A = Ainequ;
    B = Binequ;
    Aeq = Aequ;
    beq = Bequ;
    
    x0 = zeros(nums_fragment * 12, 1); % 热启动
    
    for i = 1 : 1 : nums_fragment
        state_x = [routing_line.X(1 + amount_frag * (i - 1)), cos(routing_line.Theta(1 + amount_frag * (i - 1))), 0, routing_line.X(1 + amount_frag * i), cos(routing_line.Theta(1 + amount_frag * i)), 0]';
        state_y = [routing_line.Y(1 + amount_frag * (i - 1)), sin(routing_line.Theta(1 + amount_frag * (i - 1))), 0, routing_line.Y(1 + amount_frag * i), sin(routing_line.Theta(1 + amount_frag * i)), 0]';
        
        t0 = 0;
        t1 = len_fragment;
        
        T = [ t0^5            t0^4            t0^3          t0^2        t0^1       1  
              5.0*t0^4        4.0*t0^3        3.0*t0^2      2.0*t0      1          0
              20.0*t0^3       12.0*t0^2       6.0*t0^1      2.0         0          0
              t1^5            t1^4            t1^3          t1^2        t1^1       1  
              5.0*t1^4        4.0*t1^3        3.0*t1^2      2.0*t1      1          0
              20.0*t1^3       12.0*t1^2       6.0*t1^1      2.0         0          0 ];

        a_coeff = inv(T) * state_x;

        b_coeff = inv(T) * state_y;
        
        x0(1 + 12 * (i - 1) : 6 + 12 * (i - 1), 1) = a_coeff;
        x0(7 + 12 * (i - 1) : 12 + 12 * (i - 1), 1) = b_coeff;
    end

    opts = optimoptions('quadprog','Algorithm','active-set');
    ws = optimwarmstart(x0, opts);

    [ws,fval,eflag,output,lambda] = quadprog(H, f, A, B, Aeq, beq, [], [], ws);

    x = ws.X;
    
    %% 求解平滑后路径点
    ref_line.x = [routing_line.X(1)];
    ref_line.y = [routing_line.Y(1)];

    coeff = reshape(x, 12, nums_fragment);

    interval_sample = 0.01; % 路径点间距:m

    for i = 1 : 1  : nums_fragment
        a0 = coeff(1, i);
        a1 = coeff(2, i);
        a2 = coeff(3, i);
        a3 = coeff(4, i);
        a4 = coeff(5, i);
        a5 = coeff(6, i);

        b0 = coeff(7, i);
        b1 = coeff(8, i);
        b2 = coeff(9, i);
        b3 = coeff(10, i);
        b4 = coeff(11, i);
        b5 = coeff(12, i);
        
        for s = interval_sample : interval_sample : len_fragment          
            x = a0 + a1 * s + a2 * s^2 + a3 * s^3 + a4 * s^4 + a5 * s^5;
            y = b0 + b1 * s + b2 * s^2 + b3 * s^3 + b4 * s^4 + b5 * s^5;

            ref_line.x = [ref_line.x; x];
            ref_line.y = [ref_line.y; y];
        end
    end
    
    plot(ref_line.x(:), ref_line.y(:), 'b');
end
