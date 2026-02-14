function [F_va,F_va1,F_va2,F_va3,F_vr,obst_dist,Vor]=ds_virtual_force_SVPF_v1(Ro,X_o,X_r,O_o,K_o,dynamic,VR,xo,t,dt,tstep,total_time)
% ,R_a,eta_va,R_c,b0,R_c1,eta_vr
a0 = 300;
alpha1 = 1;

% VAFP  
R_a1 = X_o-X_r;
eta_va1 = a0*(0.01+norm(R_a1)).^alpha1;
F_va = eta_va1*R_a1./(0.01+norm(R_a1)^(2));   %与轨迹之间的吸引力

%% 考虑下一时刻原始轨迹的引力

alpha2=0.9;% alpha2=0.00005;%后面时刻引力权重因子
F_va_1=0;
% for m=(t/dt+1):(t/dt+6)
for m=(t/dt+1):(total_time/tstep)
    if m>total_time/tstep
       break; 
    end
    R_a2=xo(:,round(m+1))-X_r;
    F_va_1m = alpha2./(100+norm(R_a2)^(2)).*F_va;  %350,200,80
    F_va_1 =F_va_1+F_va_1m;
end

%% 考虑前一时刻原始轨迹的引力
%求点X_r到原始轨迹的距离

alpha3=0.9;% alpha3=0.00005; %前面时刻引力权重因子
F_va_2=0;
% for m=(t/dt+1):(t/dt-4)
for m=(t/dt+1):-1:(2)
    if m<2
       break; 
    end
    R_a3=xo(:,round(m-1))-X_r;
    F_va_2m = alpha3./(100+norm(R_a3)^(2)).*F_va*(norm(R_a1)/norm(R_a3));  %根据三角形关系求得垂直于原始轨迹方向上的分力
%   F_va_2m = alpha3./(0.01+norm(R_a3)^(2)).*F_va;
    F_va_2 =F_va_2+F_va_2m;
end
%% 总的引力
%F_va  当前时刻引力 
F_va1=F_va+F_va_1;%当前时刻引力+后面时刻引力
F_va2=F_va+F_va_2;%当前时刻引力+前面时刻引力
% F_va3=F_va+F_va_1+F_va_2;%当前时刻引力+前面时刻引力+后面时刻引力
F_va3=F_va;%当前时刻引力+前面时刻引力+后面时刻引力



% VRF
SVPF.kv=0;    %速度斥力势场系数
beta = 2.5; %8  1.4 2.5
F_vr = [0;0];
F_vr2=[0,0];
length1=size(O_o,2);
obst_idx = zeros(1,length1);
obst_dist = zeros(1,length1);
for i = 1:length1                   %
    [obst_idx(i),obst_dist(i)]= dsearchn([O_o(1,i) O_o(2,i)], X_r');%找到第i个障碍物与机器人最近
    obst_dist(i)=abs(obst_dist(i));
    R_c = X_o-O_o(:,i);   
%      if norm(R_c)<=Ro(i)
        b0 = 100;
        cvr = 0.01;%Ro(i); %
        R_c1 = exp(-(norm(R_c))^2/(2*(Ro(i))^2));%/(sqrt(2*pi)*Ro);
        eta_vr = R_c1*b0.*((cvr+(Ro(i)).^2)./(cvr+(norm(R_c)).^4)).^beta;  %3
        F_vr1 = K_o*eta_vr.*R_c./(cvr+norm(R_c));
      if i<=15    %加入速度斥力势场
          F_vr2=[0 0];
      else       
        Vor=dynamic.velocity-VR';
        eor=Vor/norm(Vor);
        er=(X_o'-dynamic.center)/norm(X_o'-dynamic.center); 
        e=norm(er-eor);
        cos_alpha=(2-e^2)/2;
        alpha=acos(cos_alpha);
        alpha=atan2(sin(alpha), cos(alpha)); 
%         if abs(rad2deg(alpha))<90
           F_vr2=SVPF.kv*cos_alpha*eor;
%         else
%            F_vr2=[0 0]; 
%         end 
      end
        F_vr = F_vr + F_vr1+F_vr2';                    %与障碍物之间的斥力合力
%      end
 end
end

