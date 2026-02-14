function VAPF = VortexArtificialPotentialField(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange,  position_accuracy, obstacle, dT, simTimeMax,Success_Num)
    % position_accuracy = 0.2; %目标不可达
    position_accuracy = 0.03; %不必要臂章
    x_goal_path=x_goal_path';
    y_goal_path=y_goal_path';  
    % VAPF parameters
    VAPF.zeta = 1.1547;
    VAPF.eta = 0.0732;
    VAPF.dstar = 0.3;
    VAPF.Qstar = 0.5;

    % Parameters related to kinematic model
    VAPF.error_theta_max = deg2rad(45);
    VAPF.v_max = 0.2;
    VAPF.Kp_omega = 1.5;
    VAPF.omega_max = 0.5*pi; 

    t = 1;
%     VAPF.X = zeros(1,simTimeMax);
%     VAPF.Y = zeros(1,simTimeMax);
    % VAPF.Theta = zeros(1,simTimeMax);
    VAPF.X(1) = x;
    VAPF.Y(1) = y;
    VAPF.Theta(1) = theta;
    % VAPF.GlobalPathDistance = zeros(1,simTimeMax);


    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    Obstacle_count = size(obstacle,1);
    
    isVortex = false;
    lastSign = -1;
    globalPathIdx = 1;


    L=0;
    S=0;
    D_mean=0;
    d_delta=0;
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax  

        % Find waypoint to track
        % 1. Closest point of not reached global path to current position
        [idx, VAPF.GlobalPathDistance(t)] = dsearchn([x_goal_path(globalPathIdx:end) y_goal_path(globalPathIdx:end)], [x y]);
        globalPathIdx = globalPathIdx + idx -1;
        % 2. Select point of the path to  ~X m ahead from closest point
        goal_idx = globalPathIdx + globalPathTrackingRange*100; 
        if goal_idx > length(x_goal_path), goal_idx = length(x_goal_path); end
        x_goal = x_goal_path(goal_idx); 
        y_goal = y_goal_path(goal_idx); 

        tic;
    
        % Calculate Attractive Potential
        if norm([x y]-[x_goal y_goal]) <= VAPF.dstar
            nablaU_att =  VAPF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = VAPF.dstar/norm([x y]-[x_goal y_goal]) * VAPF.zeta*([x y]-[x_goal y_goal]);
        end
        % nablaU_att =  VAPF.zeta*([x y]-[x_goal y_goal]);
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_obst = [0 0];
        for i=1:Obstacle_count
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);
%             obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            alpha = theta - atan2(obstacle(i,obst_idx(i),2)-y, obstacle(i,obst_idx(i),1)-x);
            alpha = atan2(sin(alpha), cos(alpha));
            if obst_dist(i) <= VAPF.Qstar &&  abs(alpha) < deg2rad(150)
                nablaU_rep_Oi = (VAPF.eta*(1/VAPF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
                
                if isVortex == false 
                    if t > 10
                        if norm( [VAPF.X(t-10) VAPF.Y(t-10)] - [x y]) < 0.02
                            lastSign = lastSign*-1;
                            isVortex = true;
                        end
                    end
                else
                    goalTheta = atan2(y_goal-y, x_goal-x) - theta;
                    if abs(goalTheta) < deg2rad(10)
                        isVortex = false;
                    end
                end

                gamma = 0;
                if isVortex, gamma = lastSign*pi/2; end

                R = [cos(gamma) -sin(gamma)
                     sin(gamma)  cos(gamma) ];

                nablaU_obst = nablaU_obst + (R*nablaU_rep_Oi')';
            end
        end
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= VAPF.error_theta_max
            alpha = (VAPF.error_theta_max - abs(error_theta)) / VAPF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), VAPF.v_max );
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
        omega_ref = VAPF.Kp_omega * error_theta;
        omega_ref = min( max(omega_ref, -VAPF.omega_max), VAPF.omega_max);
        theta_pre = theta;        
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x_pre=x; y_pre=y;
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
        %% 防止单次不成功，没有DWA.travelTime
        % if (x>0.5&&y>0.5)&&(x==x_pre&&y==y_pre)
        %    VAPF.travelTime = (t-1)*dT;
        %    break;
        % end
        %% 
        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+VAPF.GlobalPathDistance(t);    
        % Archive and plot it

        VAPF.X(t) = x;
        VAPF.Y(t) = y;
        VAPF.Theta(t) = theta;
        VAPF.D1(t)=D1;
        VAPF.Theta(t) = rad2deg(theta);
        VAPF.v_ref(t)= v_ref;
        VAPF.omega_ref(t)= omega_ref;
        t = t + 1;
    end       
        VAPF.t = t-1;
    if t >= simTimeMax 
       VAPF.t = t-1;
    end
    if norm([x_goal_path(end) y_goal_path(end)] - [x y]) <= position_accuracy 
       VAPF.flag(Success_Num)=1;
    else 
       VAPF.flag(Success_Num)=0;
    end
    if VAPF.flag(Success_Num)==1
        VAPF.travelTime = (t-1)*dT;
        VAPF.MeanCalculationTime = t_sum/t_count;
        VAPF.MaxCalculationTime = t_max;
        VAPF.MinCalculationTime = t_min;    
        VAPF.L=L;
        VAPF.S=sqrt(S/(t-2));
        VAPF.D_mean=D_mean/t;
        VAPF.D_min=min(VAPF.D1);
        VAPF.d_delta=d_delta/t;
   else
        VAPF.travelTime =0; %总共多少时间间隔
        VAPF.MeanCalculationTime = 0; %平均每次计算时间
        VAPF.MaxCalculationTime = 0;%用时间最多的一次
        VAPF.MinCalculationTime = 0;%用时间最少的一次
        VAPF.L=0;
        VAPF.S=0;
        VAPF.D_mean=0;
        VAPF.D_min=0;
        VAPF.d_delta=0;
    end
end
