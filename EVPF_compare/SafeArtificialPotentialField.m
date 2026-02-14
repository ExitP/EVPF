function SAPF = SafeArtificialPotentialField(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange,  position_accuracy, obstacle, dT, simTimeMax,Success_Num)
    % position_accuracy=0.08;
    x_goal_path=x_goal_path';
    y_goal_path=y_goal_path';  
    % SAPF parameters
    SAPF.zeta = 1.1547;
    SAPF.eta = 0.0732;
    SAPF.dstar = 0.3;    
    SAPF.Qstar = 0.2; %障碍物影响范围0.3
    SAPF.dsafe = 0.1;  %安全距离0.2
    SAPF.dvort = 0.25; %涡旋距离0.35
    SAPF.alpha_th = deg2rad(5);%αth是免疫方法在角接近零时对测量噪声和顺时针、逆时针方向切换的阈值

    % Parameters related to kinematic model  不懂
    SAPF.error_theta_max = deg2rad(45); 
    SAPF.v_max = 0.2;
    SAPF.Kp_omega = 1.5;
    SAPF.omega_max = 0.5*pi; 
    t = 1;

    % SAPF.X = zeros(1,simTimeMax);
    % SAPF.Y = zeros(1,simTimeMax);
    % SAPF.Theta = zeros(1,simTimeMax);
    SAPF.X(1) = x;
    SAPF.Y(1) = y;
    SAPF.Theta(1) = theta;
    % SAPF.GlobalPathDistance = zeros(1,simTimeMax);


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
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax  

        % Find waypoint to track
        % 1. Closest point of not reached global path to current position
        [idx, SAPF.GlobalPathDistance(t)] = dsearchn([x_goal_path(globalPathIdx:end) y_goal_path(globalPathIdx:end)], [x y]);
        globalPathIdx = globalPathIdx + idx -1;
        % 2. Select point of the path to  ~X m ahead from closest point
        goal_idx = globalPathIdx + globalPathTrackingRange*100; 
        if goal_idx > length(x_goal_path), goal_idx = length(x_goal_path); end
        x_goal = x_goal_path(goal_idx); 
        y_goal = y_goal_path(goal_idx); 

        tic;
    
        % Calculate Attractive Potential
        if norm([x y]-[x_goal y_goal]) <= SAPF.dstar
            nablaU_att =  SAPF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = SAPF.dstar/norm([x y]-[x_goal y_goal]) * SAPF.zeta*([x y]-[x_goal y_goal]);
        end
        % nablaU_att =  SAPF.zeta*([x y]-[x_goal y_goal]);
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_obst = [0 0];
        for i=1:Obstacle_count
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);
            obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            %%alpha1是机器人前部与障碍物之间的夹角
            alpha1 = theta - atan2(obstacle(i,obst_idx(i),2)-y, obstacle(i,obst_idx(i),1)-x);
            alpha1 = atan2(sin(alpha1), cos(alpha1));
            %%
            if obst_dist(i) <= SAPF.Qstar &&  abs(alpha1) < deg2rad(150) %机器人与障碍物边缘距离小于障碍物斥力作用范围Q*，同时机器人前部与障碍物之间的夹角不超过150度
                nablaU_rep_Oi = (SAPF.eta*(1/SAPF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
                
                
                if obst_dist(i) <= SAPF.dsafe       
                    drel_Oi = 0;
                elseif obst_dist(i) >= 2*SAPF.dvort-SAPF.dsafe 
                    drel_Oi = 1; 
                else
                    drel_Oi = (obst_dist(i)-SAPF.dsafe) / (2*(SAPF.dvort-SAPF.dsafe));    
                end
                
                if rad2deg(alpha1) <= SAPF.alpha_th, D_alpha = +1; else, D_alpha = -1; end 

                if drel_Oi <= 0.5
                    gamma = pi * D_alpha * drel_Oi;
                else
                    gamma = pi * D_alpha * (1-drel_Oi);
                end

                R = [cos(gamma) -sin(gamma)
                     sin(gamma)  cos(gamma) ];

                nablaU_obst = nablaU_obst + (R*nablaU_rep_Oi')';   %计算总斥力势场
            end
        end
       
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= SAPF.error_theta_max
            alpha = (SAPF.error_theta_max - abs(error_theta)) / SAPF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), SAPF.v_max );
        else
            v_ref = 0;
        end
    
        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        omega_ref = SAPF.Kp_omega * error_theta;
        omega_ref = min( max(omega_ref, -SAPF.omega_max), SAPF.omega_max);
        theta_pre = theta;
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x_pre=x; y_pre=y;
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
        %% 防止单次不成功，没有DWA.travelTime
        % if (x>0.5&&y>0.5)&&(x==x_pre&&y==y_pre)
        %    SAPF.travelTime = (t-1)*dT;
        %    break;
        % end
        %%     
        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+SAPF.GlobalPathDistance(t); 
        % Archive and plot it
        
        SAPF.X(t) = x;
        SAPF.Y(t) = y;
        SAPF.D1(t)=D1;
        SAPF.Theta(t) = rad2deg(theta);
        SAPF.v_ref(t)= v_ref;
        SAPF.omega_ref(t)= omega_ref;
        t = t + 1;
    end
       SAPF.t = t-1;
    if t >= simTimeMax 
       SAPF.t = t-1;
    end
    if norm([x_goal_path(end) y_goal_path(end)] - [x y]) <= position_accuracy 
       SAPF.flag(Success_Num)=1;
    else 
       SAPF.flag(Success_Num)=0;
    end

    if SAPF.flag(Success_Num)==1
        SAPF.travelTime = (t-1)*dT;
        SAPF.MeanCalculationTime = t_sum/t_count;
        SAPF.MaxCalculationTime = t_max;
        SAPF.MinCalculationTime = t_min;    
        SAPF.L=L;
        SAPF.S=sqrt(S/(t-2));
        SAPF.D_mean=D_mean/t;
        SAPF.D_min=min(SAPF.D1);
        SAPF.d_delta=d_delta/t;
   else
        SAPF.travelTime =0; %总共多少时间间隔
        SAPF.MeanCalculationTime = 0; %平均每次计算时间
        SAPF.MaxCalculationTime = 0;%用时间最多的一次
        SAPF.MinCalculationTime = 0;%用时间最少的一次
        SAPF.L=0;
        SAPF.S=0;
        SAPF.D_mean=0;
        SAPF.D_min=0;
        SAPF.d_delta=0;
    end
end