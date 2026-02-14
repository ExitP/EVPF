function [xr,dxr,ddxr] = ds_impedance_control(xo,dxo,ddxo,Fv,last_xr,last_dxr,dt,O_o,Ro,next_xo)
%% 阻抗参数设定
Md = diag([1 1]);
Bd = 100*diag([0.5 0.5]);  
Kd = 2300*diag([1.8 1.8]);
%% 虚拟力(势场)修正轨迹
for i=1:2
    ddxr(i,:) = ddxo(i,:)+Md(i,i)^(-1)*(Bd(i,i)*(dxo(i,:)-last_dxr(i,:))+Kd(i,i)*(xo(i,:)-last_xr(i,:))+Fv(i,:));
    dxr(i,:) = last_dxr(i,:)+ddxr(i,:)*dt;
    xr(i,:) = last_xr(i,:)+dxr(i,:)*dt;
end

end