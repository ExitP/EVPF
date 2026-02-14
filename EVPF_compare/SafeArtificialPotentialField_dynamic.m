
function SAPF = SafeArtificialPotentialField_dynamic(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange,  position_accuracy, obstacle,dynamic, angles,dT, simTimeMax)
%     cmdpub = rospublisher('/cmd_vel', rostype.geometry_msgs_Twist);
%     cmdmsg = rosmessage(cmdpub);
    % SAPF parameters
    x_goal_path=x_goal_path';
    y_goal_path=y_goal_path';

    SAPF.zeta = 1.1547;
    SAPF.eta = 0.0732;
    SAPF.dstar = 0.3;
    SAPF.Qstar = 0.5;
    SAPF.dsafe = 0.2;  %安全距离0.2
%     Sp2(1)=0.2;
    SAPF.dvort = 0.35; %涡旋距离0.35
    SAPF.alpha_th = deg2rad(5);%αth是免疫方法在角接近零时对测量噪声和顺时针、逆时针方向切换的阈值

    % Parameters related to kinematic model  不懂
    SAPF.error_theta_max = deg2rad(45); 
    SAPF.v_max = 0.26;
    SAPF.Kp_omega = 1.5;   %什么参数
    SAPF.omega_max = 1.82; %0.5*pi

    t = 1;
    t1=1;
    t2=50;
    t3=1;
    t4=0;
    t_num=1; 

    L=0;
    S=0;
    D_mean=0;
    d_delta=0;
%     D_min=0;

    SAPF.X = zeros(1,simTimeMax);
    SAPF.Y = zeros(1,simTimeMax);
    SAPF.Theta = zeros(1,simTimeMax);
    SAPF.X(1) = x;
    SAPF.Y(1) = y;
    SAPF.Theta(1) = theta;
    SAPF.GlobalPathDistance = zeros(1,simTimeMax);

  
    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;
    globalPathIdx = 1;


%% 动态障碍物位置初始化
    
    Obstacle_count = size(obstacle,1);
%     dynamic.center_pre=dynamic.center;  %dynamic.center_pre用来计算MSD
%     dynamic.obstacle_pre=dynamic.obstacle(1,:,:);
    globalPathIdx = 1;
    L=0;
    S=0;
    D_mean=0;
    d_delta=0;
    dynamic.velocity = [0.21, 0]; % 初始速度
    dynamic.center=[4.23,4.23]-dynamic.velocity*0.01*250;
    SAPF.D1=0;
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax  
      %% 画动态障碍物
       dynamic.center_pre=dynamic.center;
       dynamic.center=dynamic.center+dynamic.velocity*0.01*t1;
       dynamic.obstacle(t,:,:)=[dynamic.radius*cos(angles)+dynamic.center(1) dynamic.radius*sin(angles)+dynamic.center(2)];
       dynamic.obstacle1(t,:,:)=[dynamic.radius1*cos(angles)+dynamic.center(1) dynamic.radius1*sin(angles)+dynamic.center(2)];
       obstacle1=[obstacle;dynamic.obstacle1(t,:,:)];
%        O_o1=[O_o,dynamic.center'];
%        Ro1=[Ro,dynamic.radius];
%        if  mod(t_num,60)==0 
%        prev_obstacle=plot(dynamic.obstacle(t_num,:,1), dynamic.obstacle(t_num,:,2),'b');hold on;
%        end
       Obstacle_count1 = size(obstacle1,1);
        % Find waypoint to track
        % 1. Closest point of not reached global path to current position
        [idx, SAPF.GlobalPathDistance(t)] = dsearchn([x_goal_path(globalPathIdx:end) y_goal_path(globalPathIdx:end)], [x y]);
        globalPathIdx = globalPathIdx + idx -1;
        % 2. Select point of the path to  ~X m ahead from closest point
        goal_idx = globalPathIdx + globalPathTrackingRange*100; 
        if goal_idx > length(x_goal_path), goal_idx = length(x_goal_path); end
        x_goal = x_goal_path(goal_idx); 
        y_goal = y_goal_path(goal_idx); 
        

        % Calculate Attractive Potential  %，计算与最近的临时目标点之间的引力
        if norm([x y]-[x_goal y_goal]) <= SAPF.dstar
            nablaU_att =  SAPF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = SAPF.dstar/norm([x y]-[x_goal y_goal]) * SAPF.zeta*([x y]-[x_goal y_goal]);
        end
    
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count1);
        obst_dist = zeros(1,Obstacle_count1);
        nablaU_obst = [0 0];
       
        for i=1:Obstacle_count1
  
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle1(i,:,1)' obstacle1(i,:,2)'], [x y]);%找到第i个障碍物上与机器人最近的点

  
%           obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            %%alpha1是机器人前部与障碍物之间的夹角
            alpha1 = theta - atan2(obstacle1(i,obst_idx(i),2)-y, obstacle1(i,obst_idx(i),1)-x);
            alpha1 = atan2(sin(alpha1), cos(alpha1));
            
            if obst_dist(i) <= SAPF.Qstar &&  abs(alpha1) < deg2rad(150) %机器人与障碍物边缘距离小于障碍物斥力作用范围Q*，同时机器人前部与障碍物之间的夹角不超过150度
                nablaU_rep_Oi = (SAPF.eta*(1/SAPF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle1(i,obst_idx(i),1) obstacle1(i,obst_idx(i),2)]);
                 
                if obst_dist(i) <= SAPF.dsafe       
                    drel_Oi = 0;
                elseif obst_dist(i) >= 2*SAPF.dvort-SAPF.dsafe 
                    drel_Oi = 1; 
                else
                    drel_Oi = (obst_dist(i)-SAPF.dsafe) / (2*(SAPF.dvort-SAPF.dsafe));    
                end               
%                 if obst_dist(i) <= SAPF.Sp2(t3)       
%                     drel_Oi = 0;
%                 elseif obst_dist(i) >= 2*SAPF.dvort-SAPF.Sp2(t3) 
%                     drel_Oi = 1; 
%                 else
%                     drel_Oi = (obst_dist(i)-SAPF.Sp2(t3)) / (2*(SAPF.dvort-SAPF.Sp2(t3)));    
%                 end
                
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

        SAPF.Fa(t,:)=nablaU_att;
        SAPF.Fv(t,:)=nablaU_obst;
        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
        SAPF.F(t,:)=nablaU;
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= SAPF.error_theta_max
            alpha = (SAPF.error_theta_max - abs(error_theta)) / SAPF.error_theta_max;             
            v_ref =alpha*norm(-nablaU);
            v_ref1 = min( alpha*norm(-nablaU), SAPF.v_max );
        else
            v_ref = 0;
            v_ref1=0;
        end
               
        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        omega_ref = SAPF.Kp_omega * error_theta;
        omega_ref1 = min( max(omega_ref, -SAPF.omega_max), SAPF.omega_max);
%% 

%         cmdmsg.Linear.X = v_ref; 
%         cmdmsg.Angular.Z = omega_ref;
%         send(cmdpub,cmdmsg);
%         pause(0.095);
%% 

        theta_pre = theta;
        theta = theta + omega_ref1 * dT;
        theta = atan2(sin(theta), cos(theta));
        x_pre=x; y_pre=y;
        x = x + v_ref1*cos(theta) * dT;
        y = y + v_ref1*sin(theta) * dT;
        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+SAPF.GlobalPathDistance(t);
 %% 画路径并计算最小安全距离
%         plot(x, y, 'r.', 'LineWidth', 2);
%         drawnow;
%         pause(0.01);
%         hold on;
%         if t4>0  %机器人运动到动态障碍物范围内
         [Sp1,Sp2]=MSD([x_pre;y_pre],[x;y],v_ref,dynamic,dT);
         SAPF.distance(t)=obst_dist(Obstacle_count1);
         SAPF.distance1(t)=norm(dynamic.center'-[x;y]);
         SAPF.Sp1(t)=Sp1;
         SAPF.Sp2(t)=Sp2;
%         end



 %% 赋值

%         dynamic.obstacle_pre=dynamic.obstacle(t,:,:); 
        t3=t; 
        SAPF.D1(t)=D1;
        t = t + 1;
        SAPF.X(t) = x;
        SAPF.Y(t) = y;
        SAPF.Theta(t) = theta;
        SAPF.v_ref(t)= v_ref;
        SAPF.omega_ref(t)= omega_ref;

        
    end
%     cmdmsg.Linear.X = 0; 
%     cmdmsg.Angular.Z = 0;
%     send(cmdpub,cmdmsg);
    SAPF.t = t;
    SAPF.travelTime = (t-1)*dT;
    SAPF.MeanCalculationTime = t_sum/t_count;
    SAPF.MaxCalculationTime = t_max;
    SAPF.MinCalculationTime = t_min;
    SAPF.L=L;
    SAPF.S=sqrt(S/(t-2));
    SAPF.D_mean=D_mean/t;
    SAPF.D_min=min(SAPF.D1);
    SAPF.d_delta=d_delta/t;
end