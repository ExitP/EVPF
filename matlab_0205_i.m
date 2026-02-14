 % function [VPF,SVPF,APF,DWA,VAPF,SAPF]=ypf1(Success_Num)
    clc;clear;close all; %每次调用这个循环就清除
     Success_Num=1;

    % Initial position and orientation 
    x =0.5;
    y = 0.5;
    theta = 0;      %方向0

    % Goal position
    x_goal = 9.5;
    y_goal = 9.5;
    position_accuracy = 0.03;%位置准确度     分辨率？
    globalPathTrackingRange = 0.3;%全局路径跟踪范围
    
    % Sampling period采样时间
    dT = 0.1;
    
    % Generate obstacles   %15个全局已知障碍物
    load('matlab_0205_1.mat')
    

%% 调用PSO规划全局路径
model=CreateModel(x,y,x_goal,y_goal,center,radius);
% [ipoint,~,xo,dxo,ddxo]=pso1(model);
path=xo;
    % Add obstacles close to the global path  全局路径上设置3个随机障碍物
    

    %% 画全局路径和全局障碍物


%% 设置动态障碍物

%       dynamic.center=[-1, 4.5];
%       dynamic.velocity = [0.21, 0]; % 初始速度
%       dynamic.radius=0.3;
%       dynamic.obstacle(1,:,:) =[dynamic.radius*cos(angles)+dynamic.center(1) dynamic.radius*sin(angles)+dynamic.center(2)];
% %       dynamic.O_o(1,:)=dynamic.center;
% %       dynamic.Ro(1)=dynamic.radius;

    %% 
    % Simulation
    simTimeMax = 1800;
    DWA = DynamicWindowApproach(x, y, theta, path(1,:), path(2,:), globalPathTrackingRange, position_accuracy, obstacle, dT, simTimeMax,Success_Num);    
    APF = ArtificialPotentialField(x, y, theta, path(1,:), path(2,:), globalPathTrackingRange, position_accuracy, obstacle,dT, simTimeMax,Success_Num);  
    VAPF = VortexArtificialPotentialField(x, y, theta, path(1,:), path(2,:), globalPathTrackingRange, position_accuracy, obstacle, dT, simTimeMax,Success_Num);   
    SAPF = SafeArtificialPotentialField(x, y, theta, path(1,:), path(2,:), globalPathTrackingRange, position_accuracy, obstacle, dT, simTimeMax,Success_Num);
    VPF = VirtualPotentialField(xo,dxo,ddxo,O_o',Ro,theta,Success_Num);
    EVPF = SafeVirtualPotentialField(xo,dxo,ddxo,O_o',Ro,theta,Success_Num);

    % Plot it
      Size_Font = 18;
      screen_size = get(0, 'ScreenSize');
      figure(1);
      plot(path(1,:),path(2,:),':r','linewidth',2);hold on;
%       'Color',[0.6350 0.0780 0.1840]
      plot(APF.X(1:APF.t), APF.Y(1:APF.t), 'Color',[0 0.4470 0.7410], 'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      plot(DWA.X(1:DWA.t), DWA.Y(1:DWA.t), 'Color',[0.8500 0.3250 0.0680], 'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      plot(VAPF.X(1:VAPF.t), VAPF.Y(1:VAPF.t), 'Color',[0.4660 0.6740 0.1880], 'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      plot(SAPF.X(1:SAPF.t), SAPF.Y(1:SAPF.t), 'Color',[0.6350 0.0780 0.1840], 'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      % plot(VPF.xrp(1,1:VPF.t),VPF.xrp(2,1:VPF.t),'Color', [0.6350 0.0780 0.1840],'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      plot(SVPF.xrp(1,1:SVPF.t),SVPF.xrp(2,1:SVPF.t),'-k', 'LineWidth', 2);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;   
      plot(x,y,'o','MarkerSize', 10);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;
      plot(x_goal, y_goal,'*','MarkerSize', 10);set(gca,'linewidth',2,'fontsize',Size_Font,'fontname','Times New Roman');hold on;

%   plot(ipoint.x,ipoint.y,'ro');
    for i=1:15
        h1 = fill(obstacle(i,:,1), obstacle(i,:,2),[0.5 0.7 0.8]);
        fill(obstacle_P(i,:,1), obstacle_P(i,:,2),[0.5 0.7 0.8]);
        set(h1,'edgealpha',0,'facealpha',0.5);%透明
    end 
    for i=16:18
        h2 = fill(obstacle(i,:,1), obstacle(i,:,2),[0.5 0.7 0.9]);
        fill(obstacle_P(i,:,1), obstacle_P(i,:,2),[0.3 0.7 0.8]);
        set(h2,'edgealpha',0,'facealpha',0.5);%透明
    end
    grid on;
    axis equal;
    legend('Global path','APF', 'DWA','VAPF','SAPF','SVPF','Location', 'southeast');
%     legend('Global path','APF', 'DWA','VAPF','SVPF','Location', 'southeast');
    xlabel('$x$(m)', 'Interpreter', 'latex','Fontname', 'Times New Roman', 'FontSize',Size_Font);
    ylabel('$y$(m)', 'Interpreter', 'latex','Fontname', 'Times New Roman', 'FontSize',Size_Font);
%     saveFig('path',1);
%     set(figure(1),'Position',[300,100,700,600]);
    set(figure(1),'Position',[300,100,1000,800]);
