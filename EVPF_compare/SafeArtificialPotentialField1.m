function SAPF = SafeArtificialPotentialField1(x, y, theta, x_goal_path, y_goal_path, globalPathTrackingRange,  position_accuracy, obstacle,O_o,Ro,dynamic, angles,dT, simTimeMax,path,xo,dxo,ddxo)
    last_xrp = xo(:,1);	last_dxrp = dxo(:,1); last_ddxrp = ddxo(:,1); 

    % SAPF parameters
    SAPF.zeta = 1.1547;
    SAPF.eta = 0.0732;
    SAPF.dstar = 0.3;
    SAPF.Qstar = 0.5;
    SAPF.dsafe = 0.2;  %安全距离0.2
    Sp2(1)=0.2;
    SAPF.dvort = 0.35; %涡旋距离0.35
    SAPF.alpha_th = deg2rad(5);%αth是免疫方法在角接近零时对测量噪声和顺时针、逆时针方向切换的阈值

    % Parameters related to kinematic model  不懂
    SAPF.error_theta_max = deg2rad(60); 
    SAPF.v_max = 0.26;
    SAPF.Kp_omega = 1.5;   %什么参数
    SAPF.omega_max = 1.82; %0.5*pi

    t = 1;
    t1=1;
    t2=50;
    t3=1;
    t4=0;

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

    dynamic.O_o=[];
    dynamic.R_o=[];
%% 动态障碍物位置初始化
    
    Obstacle_count = size(obstacle,1);
    dynamic.center_pre=dynamic.center;  %dynamic.center_pre用来计算MSD
%     dynamic.obstacle_pre=dynamic.obstacle(1,:,:);
    while norm([x_goal_path(end) y_goal_path(end)] - [x y]) > position_accuracy && t < simTimeMax  
      %% 画动态障碍物
     if exist('prev_obstacle', 'var') && ishandle(prev_obstacle)
        delete(prev_obstacle);
     end
      if t4==0   %没到4.23~4.77区间，只考虑静态障碍物
           obstacle1=obstacle;
           O_o1=O_o;
           Ro1=Ro;
      else  %超过4.23~4.77区间，只考虑一个动态障碍物
            dynamic.center=dynamic.center+dynamic.velocity*dT*t1;
            dynamic.obstacle(t,:,:)=[dynamic.radius*cos(angles)+dynamic.center(1) dynamic.radius*sin(angles)+dynamic.center(2)];
            obstacle1=[obstacle;dynamic.obstacle(t,:,:)];
            O_o1=[O_o;dynamic.center];
            Ro1=[Ro,dynamic.radius];
      end
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
        if y_goal>4.23 && y_goal<4.77
            O_o1=O_o;
            Ro1=Ro;
            dynamic.radius1=4*dynamic.radius;

            t4=t4+1;
            if t4==1                       %在不与静态障碍物碰撞情况下，才考虑动态障碍物，并把障碍物设置成阻碍机器人运动
            dynamic.center=[x_goal y_goal];
            dynamic.center_pre=dynamic.center-dynamic.velocity*dT*t1;
            else
            dynamic.center=dynamic.center+dynamic.velocity*dT*t1;
            end

            %在4.23~4.77区间内,考虑当前及假设的动态障碍物
            dynamic.obstacle(t,:,:)=[dynamic.radius*cos(angles)+dynamic.center(1) dynamic.radius*sin(angles)+dynamic.center(2)];%当前的障碍物
            t2=10;
%             for mm=t+1:t+t2
%             dynamic.center1=dynamic.center+dynamic.velocity*dT*mm;%构造等效的障碍物
%             O_o1=[O_o1;dynamic.center1];
%             Ro1=[Ro1,dynamic.radius1];
% %             dynamic.O_o(mm)=[dynamic.O_o;dynamic.center1];
% %             dynamic.R_o(mm)=[dynamic.R_o,4*dynamic.radius];
%             dynamic.obstacle(mm,:,:)=[dynamic.radius1*cos(angles)+dynamic.center1(1) dynamic.radius1*sin(angles)+dynamic.center1(2)];
%             end
%             obstacle1=[obstacle;dynamic.obstacle(t:t+t2,:,:)];

            obstacle1=[obstacle;dynamic.obstacle(t,:,:)];
            O_o1=[O_o;dynamic.center];
            Ro1=[Ro,dynamic.radius];

            prev_obstacle=fill(dynamic.obstacle(t,:,1), dynamic.obstacle(t,:,2),[0.6 0.7 0.7]);
            set(prev_obstacle,'edgealpha',0,'facealpha',0.5);%透明
            hold on;
        end
      
            Obstacle_count1 = size(obstacle1,1);


%         Calculate Attractive Potential  %，计算与最近的临时目标点之间的引力
%          if norm([x y]-[x_goal y_goal]) <= SAPF.dstar
%             nablaU_att1 =  SAPF.zeta*([x y]-[x_goal y_goal]);
%             nablaU_att2=0;
%             for j=goal_idx:length(x_goal_path)  %最近点到终点的吸引力
%                 nablaU_att2_j = 0.9./(100+norm([x y]-[x_goal_path(j) y_goal_path(j)])^2)*nablaU_att1;
%                 nablaU_att2 = nablaU_att2+nablaU_att2_j ;
%             end
%         else 
%             nablaU_att1 = SAPF.dstar/norm([x y]-[x_goal y_goal]) * SAPF.zeta*([x y]-[x_goal y_goal]);
%             nablaU_att2=0;
%             for j=goal_idx:length(x_goal_path)
%                 nablaU_att2_j = 0.9./(100+norm([x y]-[x_goal_path(j) y_goal_path(j)])^2)*nablaU_att1;
%                 nablaU_att2 = nablaU_att2+nablaU_att2_j ;
%             end
%         end
%         nablaU_att=nablaU_att1+nablaU_att2;  



        [F_va,F_va1,F_va2,F_va3,F_vr,obst_dist] = ds_virtual_force_VPF([x_goal y_goal],[x y],goal_idx,x_goal_path,y_goal_path,O_o,Ro,obstacle,theta);

        nablaU_att=F_va;
        nablaU_obst=F_vr;
        SAPF.Fa(t,:)=F_va;
        SAPF.Fv(t,:)=F_vr;


        % Find the minimum distance from the obstacle & Calculate Repulsive Potential

%         nablaU_obst = [0 0];
       

%             

        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
        SAPF.F(t,:)=nablaU;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= SAPF.error_theta_max
            alpha = (SAPF.error_theta_max - abs(error_theta)) / SAPF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), SAPF.v_max );
        else
            v_ref = 0;
        end
%         [SAPF.xrp(:,t), SAPF.dxrp(:,t), SAPF.ddxrp(:,t)] = ds_impedance_control3([x_goal y_goal]'...
%         ,dxo(:,goal_idx),ddxo(:,goal_idx),nablaU',last_xrp,last_dxrp,dT);          %调节阻抗控制参数得到引力+斥力的整形轨迹
%         last_xrp = SAPF.xrp(:,t); last_dxrp = SAPF.dxrp(:,t);	last_ddxrp =SAPF.ddxrp(:,t); 
%         x_pre=x; y_pre=y;
%         x=SAPF.xrp(1,t);  y=SAPF.xrp(2,t);
%% 

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



        %评价指标
        L1=norm([x,y]-[x_pre,y_pre]);
        L=L+L1;
        S1=(theta_pre-theta)^2;
        S=S+S1;
        D1=min(obst_dist);
        D_mean=D_mean+D1;
        d_delta=d_delta+SAPF.GlobalPathDistance(t);
 %% 画路径并计算最小安全距离
        plot(x, y, 'r.', 'LineWidth', 2);
        drawnow;
        pause(0.01);
        hold on;
        if t4>0  %机器人运动到动态障碍物范围内
        [Sp1,Sp2]=MSD(x_pre,y_pre,x,y,v_ref,dynamic,dT);
%         SAPF.distance(t)=obst_dist(Obstacle_count+1);
        SAPF.Sp1(t)=Sp1;
        SAPF.Sp2(t)=Sp2;
        end


 %% 赋值
        dynamic.center_pre=dynamic.center;
%         dynamic.obstacle_pre=dynamic.obstacle(t,:,:); 
        t3=t; 
        SAPF.D1(t)=D1;
        t = t + 1;
        SAPF.X(t) = x;
        SAPF.Y(t) = y;
        SAPF.Theta(t) = theta;
        SAPF.v_ref(t)= v_ref;
        SAPF.omega_ref(t)= omega_ref;

%% 更新动态障碍物

%         dynamic.center=dynamic.center_pre+dynamic.velocity*dT*t1;
%         dynamic.obstacle(t,:,:)=[dynamic.radius*cos(angles)+dynamic.center(1) dynamic.radius*sin(angles)+dynamic.center(2)];
%         
% %       for 
%         dynamic.center1=dynamic.center_pre+dynamic.velocity*dT*t2;
%         dynamic.obstacle(t+t2,:,:)=[3*dynamic.radius*cos(angles)+dynamic.center1(1) 3*dynamic.radius*sin(angles)+dynamic.center1(2)];
        
    end

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