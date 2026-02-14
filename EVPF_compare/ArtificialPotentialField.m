function APF =  ArtificialPotentialField(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange, position_accuracy, obstacle, dT, simTimeMax,Success_Num)
    x_goal_path=x_goal_path';
    y_goal_path=y_goal_path';
    % APF parameters
    APF.zeta = 1.1547; %吸引势场比例因子
    APF.eta = 0.0732;  %排斥势场比例因子
    APF.dstar = 0.3;   %当线速度减小以保证平滑到达目标时，到目标的边缘距离
    APF.Qstar = 0.5;    %障碍物的作用范围
    %% 
    
    % Parameters related to kinematic model
    APF.error_theta_max = deg2rad(45); %边距方向误差
    APF.v_max = 0.2;                  %机器人允许的最大线速度
    APF.Kp_omega = 1.5;            
    APF.omega_max = 0.5*pi;  %机器人允许的最大角速度
    %% 
   
    t = 1;
%     APF.X = zeros(1,simTimeMax);
%     APF.Y = zeros(1,simTimeMax);
    % APF.Theta = zeros(1,simTimeMax);
    APF.X(1) = x;
    APF.Y(1) = y;
    APF.Theta(1) = theta;
    % APF.GlobalPathDistance = zeros(1,simTimeMax);

    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    Obstacle_count = size(obstacle,1);
    globalPathIdx = 1;

    L=0;
    S=0;
    D_mean=0;
    d_delta=0;
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax          %遍历所有路径点  
%% 

        % Find waypoint to track
        % 1. Closest point of not reached global path to current position
        [idx, APF.GlobalPathDistance(t)] = dsearchn([x_goal_path(globalPathIdx:end) y_goal_path(globalPathIdx:end)], [x y]);
        globalPathIdx = globalPathIdx + idx -1;
        % 2. Select point of the path to  ~X m ahead from closest point
        goal_idx = globalPathIdx + globalPathTrackingRange*100; 
        if goal_idx > length(x_goal_path), goal_idx = length(x_goal_path); end
        x_goal = x_goal_path(goal_idx); 
        y_goal = y_goal_path(goal_idx); 

        tic;   %开始计时
    
        % Calculate Attractive Potential            %找到最近的临时目标点，计算与其之间的引力
        %  机器人与目标点距离不同，计算引力的方式不同，这样可以防止离目标点较远，引力过大，容易与机器人发生碰撞
        if norm([x y]-[x_goal y_goal]) <= APF.dstar
            nablaU_att =  APF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = APF.dstar/norm([x y]-[x_goal y_goal]) * APF.zeta*([x y]-[x_goal y_goal]);
        end
        % nablaU_att =  APF.zeta*([x y]-[x_goal y_goal]);
        % Find the minimum distance from the obstacle & Calculate Repulsive
        % Potential  找到距离障碍物的最小距离
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_rep = [0 0];
        for i=1:Obstacle_count    %机器人每个位置考虑了所有的全局障碍物与未知障碍物
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);%找到与机器人最近的障碍物上的那个点的索引 
%             obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE          传感器测量误差[-0.01,0.01]
            if obst_dist(i) <= APF.Qstar        %机器人与障碍物边缘距离小于障碍物斥力作用范围Q*
%               AAAA=(APF.eta*(1/APF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
                % BBBB=APF.eta*(1/APF.Qstar - 1/obst_dist(i)) * ((norm([x y]-[x_goal y_goal]))^2/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)])...
                %      +2/2*APF.eta*(1/APF.Qstar - 1/obst_dist(i))^2*([x_goal y_goal]-[x y]);
                k_rep=APF.eta;            %%（q,qg）^n
                P0=APF.Qstar;
                do=obst_dist(i);
                q=[x y];
                qo=[obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)];
                qg=[x_goal y_goal];
                a1=(q - qo)/norm(q - qo);
                a2=(q - qg)/norm(q - qg);
                A=(k_rep*(1/P0- 1/do) * 1/do^2)*(q - qo)/norm(q - qo);
                B=k_rep*(1/P0 - 1/do) * ((norm(q-qg))^2/do^2)*a1+2/2*k_rep*(1/P0 - 1/do)^2*norm(q-qg)*a2;
                nablaU_rep = nablaU_rep + B;
            end
        end
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_rep;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));  %通过人工势场确定参考方向
    
        error_theta = theta_ref - theta;   %参考方向与实际方向之间的误差    
        if abs(error_theta) <= APF.error_theta_max     %最多不超过45度
            alpha = (APF.error_theta_max - abs(error_theta)) / APF.error_theta_max;     %误差占比
            v_ref = min( alpha*norm(-nablaU), APF.v_max );   %通过人工势场确定参考速度
        else
            v_ref = 0;
        end
    
        t_i = toc;   %计算引力斥力所用时间
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i; %计算引力斥力总次数所用时间
        t_count = t_count + 1;  %计算引力斥力次数
    
        % Simple kinematic mobile robot model       %这里不太懂  简单运动学
        % Omitted dynamics.省略动力学
        omega_ref = APF.Kp_omega * error_theta;    %参考角速度
        omega_ref = min( max(omega_ref, -APF.omega_max), APF.omega_max);
        theta_pre = theta;

        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta)); 
        x_pre=x; y_pre=y;
        x = x + v_ref*cos(theta) * dT;               
        y = y + v_ref*sin(theta) * dT;
        % %% 防止单次不成功，没有DWA.travelTime
        % if (x>0.5&&y>0.5)&&(x==x_pre&&y==y_pre)
        %    APF.travelTime = (t-1)*dT;
        %    break;
        % end
        %% 
        theta1=atan2(y-y_pre, x-x_pre);
        theta1 = atan2(sin(theta1), cos(theta1));

        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+APF.GlobalPathDistance(t);

        % Archive and plot it

        APF.X(t) = x;
        APF.Y(t) = y;
%       APF.obst_dist(t)=obst_dist;
        APF.D1(t)=D1;
        APF.Theta(t) = rad2deg(theta);
        APF.Theta1(t) = rad2deg(theta1);
        APF.v_ref(t)= v_ref;
        APF.omega_ref(t)= omega_ref;
        t = t + 1;
    end
    APF.t = t-1;  %总共多少个路径点
    if t >= simTimeMax 
       APF.t = t-1;
    end
    if norm([x_goal_path(end) y_goal_path(end)] - [x y]) <= position_accuracy 
       APF.flag(Success_Num)=1;
    else 
       APF.flag(Success_Num)=0;
    end
    if APF.flag(Success_Num)==1
        APF.travelTime = (t-1)*dT; %总共多少时间间隔
        APF.MeanCalculationTime = t_sum/t_count; %平均每次计算时间
        APF.MaxCalculationTime = t_max;%用时间最多的一次
        APF.MinCalculationTime = t_min;%用时间最少的一次
        APF.L=L;
        APF.S=sqrt(S/(t-2));
        APF.D_mean=D_mean/t;
        APF.D_min=min(APF.D1);
        APF.d_delta=d_delta/t;
    else
        APF.travelTime =0; %总共多少时间间隔
        APF.MeanCalculationTime = 0; %平均每次计算时间
        APF.MaxCalculationTime = 0;%用时间最多的一次
        APF.MinCalculationTime = 0;%用时间最少的一次
        APF.L=0;
        APF.S=0;
        APF.D_mean=0;
        APF.D_min=0;
        APF.d_delta=0;
    end
end