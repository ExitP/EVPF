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

function PlotSolution(sol,model,ipoint)

    xs=model.xs;
    ys=model.ys;
    xt=model.xt;
    yt=model.yt;
    xobs=model.xobs;
    yobs=model.yobs;
    robs=model.robs;
    
    XS=sol.XS;
    YS=sol.YS;
    xx=sol.xx;
    yy=sol.yy;
    dxx=sol.dxx;
    dyy=sol.dyy;    
    ddxx=sol.ddxx;
    ddyy=sol.ddyy;    
    theta=linspace(0,2*pi,100);
    figure(1)
    for k=1:numel(xobs)
        fill(xobs(k)+robs(k)*cos(theta),yobs(k)+robs(k)*sin(theta),[0.5 0.7 0.8]);
        hold on;
    end
    plot(xx,yy,'k','LineWidth',2);     %画线
%     hold on;
%     plot(dxx,dyy,'r','LineWidth',2);
%     hold on;
%     plot(ddxx,ddyy,'b','LineWidth',2);    
%     plot(XS,YS,'ro');                   %画小圆圈
%     hold on;
    plot(xs,ys,'bs','MarkerSize',12,'MarkerFaceColor','y');   %画起始点 
    hold on;
    plot(xt,yt,'kp','MarkerSize',16,'MarkerFaceColor','g');   %画目标点
    hold on;
    grid on;
    axis equal;
    
%% 用原来的解

%   for t=1:1:1000
%       plot(xx(t),yy(t),'r.','linewidth',0.01);    
%      drawnow;
%      pause(0.005);
%      hold on;
%   end
 %% 用控制点 
% tt=linspace(0,1,1000);%生成0-1之间100个等间距点的行向量
% xx=spline(ipoint,XS,tt);  %以tt为步长，TS为x，XS为y，进行三次样条数据插值得到
% yy=spline(TS,YS,tt);    %对yy三次样条数据插值



%     tt=linspace(0,1,100);
%     figure(3)
%     subplot(3,1,1);
%     plot(tt,xx,'k','LineWidth',2);     %画线
%     hold on; 
%     plot(tt,yy,'k','LineWidth',2);
%     subplot(3,1,2);
%     plot(tt,dxx,'k','LineWidth',2);     %画线
%     hold on;
%     plot(tt,dyy,'k','LineWidth',2);
%     subplot(3,1,3);
%     plot(tt,ddxx,'k','LineWidth',2);     %画线
%     hold on;
%     plot(tt,ddyy,'k','LineWidth',2);    
end