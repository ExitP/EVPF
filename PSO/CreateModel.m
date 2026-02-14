%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPAP115
% Project Title: Path Planning using PSO in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function model=CreateModel(x,y,x_goal,y_goal,center,radius)


%% 两个障碍物

%     xobs=[0.56 0.45];
%     yobs=[0.45 0.35];
%     robs=[0.03 0.04];  
%% 复杂障碍物环境

%     xobs=[ 2.5  0.5 4.3 -1 1.8];
%     yobs=[3  4.8 4.5 2 0.8];
%     robs=[0.6  1.2 1 1 0.7]; 
%% 简单障碍物环境

%     xobs=[3.5 2.7 0.3];
%     yobs=[2.8 5.5 2.2];
%     robs=[0.6 0.6 0.6];

%% 三个障碍物

%     xobs=[1.5 4.0 1.2];
%     yobs=[4.5 3.0 1.5];
%     robs=[1.5 1.0 0.8];

%% 随机障碍物环境

xs=x;
ys=y;
xt=x_goal;
yt=y_goal;
xobs=center(:,1);
yobs=center(:,2);
robs=radius;
% n=length(radius);
n=3; 
    xmin=-10;
    xmax= 10;
    
    ymin=-10;
    ymax= 10;
    
    model.xs=xs;
    model.ys=ys;
    model.xt=xt;
    model.yt=yt;
    model.xobs=xobs;
    model.yobs=yobs;
    model.robs=robs;
    model.n=n;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
%     theta=linspace(0,2*pi,100);
%     for k=1:numel(xobs)
%         fill(xobs(k)+robs(k)*cos(theta),yobs(k)+robs(k)*sin(theta),[0.5 0.7 0.8]);  %画三个圆形障碍物
%         hold on;
%     end
%     grid on;
%     axis equal;
end