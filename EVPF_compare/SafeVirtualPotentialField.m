function SVPF = SafeVirtualPotentialField(xo,dxo,ddxo,O_o,Ro,theta,Success_Num)

K_o = 1*ones(1, size(O_o,2));
tstep = 0.01;
dt = tstep;
total_time = (size(xo,2)-1)*dt;


last_xrp = xo(:,1);	last_dxrp = dxo(:,1); last_ddxrp = ddxo(:,1); %
xrp = xo; dxrp = dxo; ddxrp = ddxo;

dXo = zeros(1,total_time/tstep+1);    %轨迹初始化
ddXo = zeros(1,total_time/tstep+1);

dXrp = zeros(1,total_time/tstep+1);
ddXrp = zeros(1,total_time/tstep+1);
SVPF.theta(1)=0;

t_max = -inf;
t_min = inf;
t_count = 0;
t_sum = 0;

L=0;
S=0;
D_mean=0;
d_delta=0;
error_theta_max = deg2rad(45); 
v_max = 0.26;
Kp_omega = 1.5;   %什么参数
omega_max = 1.82; %0.5*pi
Q_star=0.5;

for t=0:tstep:total_time
    %% 虚拟力(势场)修正轨迹
    

    tic;


    [F_vap,F_va1p,F_va2p,F_va3p, F_vrp,obst_dist] =  ds_virtual_force_SVPF(Ro,xo(:,round(t/dt+1)),last_xrp,O_o,K_o,xo,t,dt,tstep,total_time);
    t_i = toc;
    t_max = max(t_max, t_i);
    t_min = min(t_min, t_i);
    t_sum = t_sum + t_i; %计算引力斥力总次数所用时间
    t_count = t_count + 1;  %计算引力斥力次数


    SVPF.Fa(round(t/dt+1),:)=F_va3p;
    SVPF.Fv(round(t/dt+1),:)=F_vrp;
    F_vp = (F_va3p+F_vrp);   %势场力=引力+斥力
    SVPF.F(round(t/dt+1),:)=F_vp;

    theta_ref = atan2(-F_vp(2), -F_vp(1));
    error_theta = theta_ref - theta;

    if (abs(error_theta) <= error_theta_max)  && norm(F_vp) ~=0
        alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
        v_ref = min( alpha*norm(-F_vp), v_max );
    else
        v_ref = v_max;
    end

    omega_ref = Kp_omega * error_theta;
    omega_ref = min( max(omega_ref, -omega_max), omega_max);

    theta_pre = theta;
    % theta = theta + omega_ref * dt;
    % theta = atan2(sin(theta), cos(theta));



    [xrp(:,round(t/dt+1)), dxrp(:,round(t/dt+1)), ddxrp(:,round(t/dt+1))] = ds_impedance_control(xo(:,round(t/dt+1))...
        ,dxo(:,round(t/dt+1)),ddxo(:,round(t/dt+1)),F_vp,last_xrp,last_dxrp,dt);          %调节阻抗控制参数得到引力+斥力的整形轨迹

    [idx,SVPF.GlobalPathDistance(round(t/dt+1))] = dsearchn(xo(:,round(t/dt+1):end)', xrp(:,round(t/dt+1))' );
%% 

    theta=atan2(xrp(2,round(t/dt+1))-last_xrp(2),xrp(1,round(t/dt+1))-last_xrp(1));
    theta = atan2(sin(theta), cos(theta));%限制在-pi,pi范围内
    % ********************
    % K_o = 1*ones(1, size(O_o,2));
    ddd = O_o-xrp(:,round(t/dt+1));
    for iii = 1:18
        ddd1(iii) = norm(ddd(iii));
        K_o(iii) = exp(-0.3*(ddd1(iii)-Ro(iii)));
    end

%% 

    L1=norm(xrp(:,round(t/dt+1))-last_xrp);
    L=L+L1;
    S1=(theta_pre-theta)^2;
    S=S+S1;
    if t_count>1
    SVPF.theta(t_count)=rad2deg(theta);
    end
    D1=min(obst_dist);
    D_mean=D_mean+D1;
    d_delta=d_delta+SVPF.GlobalPathDistance(round(t/dt+1));
    SVPF.xrp(:,t_count)=xrp(:,round(t/dt+1));
    % theta_ref=atan2(SVPF.xrp(2,t_count),SVPF.xrp(1,t_count));
    % theta_ref = atan2(sin(theta_ref), cos(theta_ref)); 
    % SVPF.theta_ref(t_count)=rad2deg(theta_ref);
    SVPF.D1(t_count)=D1;
    SVPF.v_ref(t_count)=v_ref;
    SVPF.omega_ref(t_count)=omega_ref;
    % SVPF.theta(t_count)=theta;

%   [Sp1,Sp2]=MSD(x_pre,y_pre,x,y,v_ref,dynamic,dT);


    last_xrp = xrp(:,round(t/dt+1)); last_dxrp = dxrp(:,round(t/dt+1));	last_ddxrp = ddxrp(:,round(t/dt+1));
    dXo(:,t_count) = norm(dxo(:,round(t/dt+1)));ddXo(:,round(t/dt+1)) = norm(ddxo(:,round(t/dt+1)));  
    SVPF.dXrp(:,t_count) = norm(last_dxrp); SVPF.ddXrp(:,t_count) = norm(last_ddxrp);


end
SVPF.flag(Success_Num)=1;
SVPF.t_count  = t_count;
SVPF.t  = t_count;
SVPF.travelTime = (t_count-1)*dt;
SVPF.MeanCalculationTime = t_sum/t_count;
SVPF.MaxCalculationTime = t_max;
SVPF.MinCalculationTime = t_min;
SVPF.L=L;
SVPF.S=sqrt(S/(t_count-2));
SVPF.D_mean=D_mean/t_count;
SVPF.D_min=min(SVPF.D1);
SVPF.d_delta=d_delta/t_count;

end


% %% 画力
% 
% figure(12)
% t1=1:total_time/tstep+1;
% subplot(2,1,1);
% plot(t1,Fa(:,1),'LineWidth',2);
% hold on;
% plot(t1,Fv(:,1),'LineWidth',2);
% hold on;
% plot(t1,F(:,1),'LineWidth',2);
% subplot(2,1,2);
% plot(t1,Fa(:,2),'LineWidth',2);
% hold on;
% plot(t1,Fv(:,2),'LineWidth',2);
% hold on;
% plot(t1,F(:,2),'LineWidth',2);
% legend('Fa','Fv','F');
% set(figure(12),'Position',[1000,200,500,400]);
% %% 画最小安全距离
% % figure(13)
% % t=1:t_count;
% % %     plot(t,SAPF.Sp1,'b-');
% % %     hold on;
% % plot(t,Sp2,'r-','LineWidth',2);
% % hold on;
% % % plot(t,distance,'b-','LineWidth',2);  %动态障碍物的距离  
% % %     axis([0, 600, 0, 1]); % 设置坐标轴范围
% % title('Minimum safety distance');
% % 
% % saveFig('Minimum_safety_distance',1);
% % set(figure(13),'Position',[1000,200,500,400]);
%  %% 画线速度和角速度
% figure(14)
% t=1:t_count;
% plot(t,omega_ref1,'r-','LineWidth',2);
% hold on;
% plot(t,v_ref1,'b-','LineWidth',2);    
% title('Linear velocity and angular velocity');
% legend('angular velocity','Linear velocity');
% saveFig('Linear velocity and angular velocity',1);
% set(figure(14),'Position',[1000,200,500,400]);
% 
% %% 
% figure(15)
% t=1:t_count;
% plot(t,dXrp,'-k','linewidth',2);
% title('Robot-real-velocity');