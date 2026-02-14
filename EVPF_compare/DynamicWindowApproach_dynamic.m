function DWA = DynamicWindowApproach_dynamic(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange,  position_accuracy, obstacle,dynamic,angles, dT, simTimeMax,Success_Num)
%     position_accuracy = 0.2;  %目标不可达
     position_accuracy=0.08;    %不必要避障
    x_goal_path=x_goal_path';
    y_goal_path=y_goal_path';  
   
    DWA.heading = 0.1;
    DWA.distance = 1;
    DWA.velocity = 10;
    DWA.predictionTime = 1;
    DWA.v_accuracyPrediction = 0.005;
    DWA.omega_accuracyPrediction = 0.025*pi;
    DWA.obstacleDistance = 0.25;

    DWA.v_max = 0.2;
    DWA.a_max = 0.2;
    DWA.omega_max = 0.5*pi; 
    DWA.epsilon_max = 1.5*pi; 

    t = 1;
%     DWA.X = zeros(1,simTimeMax);
%     DWA.Y = zeros(1,simTimeMax);
    DWA.Theta = zeros(1,simTimeMax);
    DWA.X(1) = x;
    DWA.Y(1) = y;
    DWA.Theta(1) = theta;
    DWA.GlobalPathDistance = zeros(1,simTimeMax);

    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    V_current = 0;
    omega_current = 0;
    Obstacle_count = size(obstacle,1);
    globalPathIdx = 1;

    L=0;
    S=0;
    D_mean=0;
    d_delta=0;
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax  

        % Find waypoint to track
        % 1. Closest point of not reached global path to current position
        [idx, DWA.GlobalPathDistance(t)] = dsearchn([x_goal_path(globalPathIdx:end) y_goal_path(globalPathIdx:end)], [x y]);
        globalPathIdx = globalPathIdx + idx -1;
        % 2. Select point of the path to  ~X m ahead from closest point
        goal_idx = globalPathIdx + globalPathTrackingRange*100; 
        if goal_idx > length(x_goal_path), goal_idx = length(x_goal_path); end
        x_goal = x_goal_path(goal_idx); 
        y_goal = y_goal_path(goal_idx); 

        tic;
    
        % Dynamic Window
        %[Vmin Vmax omega_min omega_max]
        tmp = [0                      DWA.v_max                 -DWA.omega_max                      DWA.omega_max
               V_current-DWA.a_max*dT V_current+DWA.a_max*dT     omega_current-DWA.epsilon_max*dT   omega_current+DWA.epsilon_max*dT];
        dynamicWindow = [max(tmp(:,1)) min(tmp(:,2)) max(tmp(:,3)) min(tmp(:,4))];

        % Evaluate possible movements
        v_range = dynamicWindow(1):DWA.v_accuracyPrediction:dynamicWindow(2);
        omega_range = dynamicWindow(3):DWA.omega_accuracyPrediction:dynamicWindow(4);
        eval_function = zeros(length(v_range), length(omega_range));
        t_predition_range = 0:dT:DWA.predictionTime;
        eval_trajectory = zeros(length(v_range), length(omega_range), length(t_predition_range), 3);
        opt_max_eval_function = -inf;
        opt_idx = [0;0];
        for i = 1:length(v_range)
            for j = 1:length(omega_range)
                % Prediction
                eval_v = v_range(i);
                eval_omega = omega_range(j);
                eval_trajectory(i,j,1,1) = x;
                eval_trajectory(i,j,1,2) = y;
                eval_trajectory(i,j,1,3) = theta;
                for k=2:length(t_predition_range)
                    x_prev = eval_trajectory(i,j,k-1,1);
                    y_prev = eval_trajectory(i,j,k-1,2);
                    theta_prev = eval_trajectory(i,j,k-1,3);
                    eval_trajectory(i,j,k,1) = x_prev + dT*cos(theta_prev)*eval_v;
                    eval_trajectory(i,j,k,2) = y_prev + dT*sin(theta_prev)*eval_v;
                    eval_trajectory(i,j,k,3) = theta_prev + dT*eval_omega;
                end

                % Cost function
                robotTheta = eval_trajectory(i,j,end,3);
                goalTheta = atan2(y_goal-eval_trajectory(i,j,end,2), x_goal-eval_trajectory(i,j,end,1));
                headingReward = DWA.heading*(deg2rad(180) - abs(robotTheta - goalTheta));
             
                distanceReward = DWA.distance * 1/(1+ norm([x_goal y_goal] - [eval_trajectory(i,j,end,1) eval_trajectory(i,j,end,2)]));

                velocityReward = DWA.velocity*eval_v;

                eval_function(i,j) = headingReward+distanceReward+velocityReward;
    
                % Find the minimum distance from the obstacle
                obst_idx = zeros(1,Obstacle_count);
                obst_dist = inf*ones(1,Obstacle_count);
                for k=1:Obstacle_count
                    for l=1:2:size(eval_trajectory,3)
                        robotPose = [eval_trajectory(i,j,l,1) eval_trajectory(i,j,l,2)];
                        [tmp_min_dist_idx, tmp_minDist] = dsearchn([obstacle(k,:,1)' obstacle(k,:,2)'], robotPose);
                         tmp_minDist = tmp_minDist + 0.01*(2*rand()-1); % MEASUREMENT NOISE
                        if tmp_minDist < obst_dist(k)
                            obst_idx(k) = tmp_min_dist_idx;
                            obst_dist(k) = tmp_minDist;
                        end
                    end
                end

                if eval_function(i,j) > opt_max_eval_function && min(obst_dist) > DWA.obstacleDistance
                    opt_idx = [i;j];
                    opt_max_eval_function = eval_function(i,j);
                end
            end
        end

        % Select best candidate
        try
        v_ref = v_range(opt_idx(1));
        omega_ref = omega_range(opt_idx(2));
        catch 
            v_ref = 0;
            omega_ref = 0;
        end

        V_current = v_ref;
        omega_current = omega_ref;

        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        theta_pre = theta; 
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x_pre=x; y_pre=y;
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
    
        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+DWA.GlobalPathDistance(t);

        % Archive and plot it

%         DWA.obst_dist()=obst_dist
        DWA.X(t) = x;
        DWA.Y(t) = y;
        DWA.Theta(t) = theta;
        DWA.D1(t)=D1;
        DWA.Theta(t) = theta;
        DWA.v_ref(t)= v_ref;
        DWA.omega_ref(t)= omega_ref;
        t = t + 1;
    end
        DWA.t = t-1;
    if t >= simTimeMax 
       DWA.t = t-1;
    end
    if norm([x_goal_path(end) y_goal_path(end)] - [x y]) <= position_accuracy 
       DWA.flag(Success_Num)=1;
    else 
       DWA.flag(Success_Num)=0;
    end
    if DWA.flag(Success_Num)==1  
        DWA.travelTime = (t-1)*dT;
        DWA.MeanCalculationTime = t_sum/t_count;
        DWA.MaxCalculationTime = t_max;
        DWA.MinCalculationTime = t_min;
        DWA.L=L;
        DWA.S=sqrt(S/(t-2));
        DWA.D_mean=D_mean/t;
        DWA.D_min=min(DWA.D1);
        DWA.d_delta=d_delta/t;
    else
        DWA.travelTime =0; %总共多少时间间隔
        DWA.MeanCalculationTime = 0; %平均每次计算时间
        DWA.MaxCalculationTime = 0;%用时间最多的一次
        DWA.MinCalculationTime = 0;%用时间最少的一次
        DWA.L=0;
        DWA.S=0;
        DWA.D_mean=0;
        DWA.D_min=0;
        DWA.d_delta=0;
    end
end