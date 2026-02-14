function [F_va, F_vr,obst_dist]=ds_virtual_force_VPF(Ro,X_o,X_r,O_o,K_o,Q_star)
% ,R_a,eta_va,R_c,b0,R_c1,eta_vr
a0 = 300;
alpha = 1;
% VAFP  
R_a = X_o-X_r;
eta_va = a0*(0.01+norm(R_a)).^alpha;
F_va = eta_va*R_a./(0.01+norm(R_a)^(2));   %与轨迹之间的吸引力

% VRF
beta = 2.8; %8  1.4 2.5 2.8 2.85
F_vr = [0;0];
length1=size(O_o,2);
obst_idx = zeros(1,length1);
obst_dist = zeros(1,length1);
for i = 1:length1                   %
    [obst_idx(i),obst_dist(i)]= dsearchn([O_o(1,i) O_o(2,i)], X_r');%找到第i个障碍物与机器人最近
    obst_dist(i)=abs(obst_dist(i)-Ro(i));
    R_c = X_o-O_o(:,i);   
%      if norm(R_c)<=Ro(i)
        b0 = 100;
        cvr = 0.01;%Ro(i); %
        R_c1 = exp(-(norm(R_c))^2/(2*(Ro(i))^2));%/(sqrt(2*pi)*Ro);
        eta_vr = R_c1*b0.*((cvr+(Ro(i)).^2)./(cvr+(norm(R_c)).^3)).^beta;
        F_vr1 = K_o(i)*eta_vr.*R_c./(cvr+norm(R_c));
        F_vr = F_vr + F_vr1;                    %与障碍物之间的斥力合力
%      end
end

