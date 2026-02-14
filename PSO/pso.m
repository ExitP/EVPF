function path=pso(model)

% clc;
% clear;
% close all;

%% Problem Definition

% model=CreateModel();

% model.n=3;  % number of Handle Points

CostFunction=@(x) MyCost(x,model);    % Cost Function    @(x)创建一个匿名函数，存储在costfunction中，调用时将调用Mycost函数，

nVar=model.n;       % Number of Decision Variables

VarSize=[1 nVar];   % Size of Decision Variables Matrix  VarSize=[1 3]

VarMin.x=model.xmin;           % Lower Bound of Variables
VarMax.x=model.xmax;           % Upper Bound of Variables
VarMin.y=model.ymin;           % Lower Bound of Variables
VarMax.y=model.ymax;           % Upper Bound of Variables


%% PSO Parameters

MaxIter=320;          % Maximum Number of Iterations

nPop=40;           % Population Size (Swarm Size)  种群大小

w=1;                % Inertia Weight   惯性权重
wdamp=0.98;         % Inertia Weight Damping Ratio  惯性权重衰减率
c1=1.5;             % Personal Learning Coefficient 个人学习系数
c2=1.5;             % Global Learning Coefficient  全局学习系数

% % Constriction Coefficient   约束系数
% phi1=2.05;
% phi2=2.05;
% phi=phi1+phi2;
% chi=2/(phi-2+sqrt(phi^2-4*phi));
% w=chi;               % Inertia Weight
% wdamp=1;             % Inertia Weight Damping Ratio
% c1=chi*phi1;         % Personal Learning Coefficient
% c2=chi*phi2;         % Global Learning Coefficient

alpha=0.1;
VelMax.x=alpha*(VarMax.x-VarMin.x);    % Maximum Velocity
VelMin.x=-VelMax.x;                    % Minimum Velocity
VelMax.y=alpha*(VarMax.y-VarMin.y);    % Maximum Velocity
VelMin.y=-VelMax.y;                    % Minimum Velocity

%% Initialization

% Create Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Sol=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
empty_particle.Best.Sol=[];

% Initialize Global Best
GlobalBest.Cost=inf;

% Create Particles Matrix
particle=repmat(empty_particle,nPop,1);%150行，1列   重复数组副本

% Initialization Loop   初始化循环
for i=1:nPop
    
    % Initialize Position  初始化位置
    if i == 1      
        % Straight line from source to destination
        xx = linspace(model.xs, model.xt, model.n+2);%先生成一个等间距行向量       生成 model.n+2 个点。这些点的间距为 (x2-x1)/(model.n+2-1)
        yy = linspace(model.ys, model.yt, model.n+2);%先生成一条直线
        particle(i).Position.x = xx(2:end-1);
        particle(i).Position.y = yy(2:end-1);
    else
        particle(i).Position=CreateRandomSolution(model);
    end
    
    % Initialize Velocity    初始化速度
    particle(i).Velocity.x=zeros(VarSize);  %返回1x3零向量
    particle(i).Velocity.y=zeros(VarSize);
    
    % Evaluation           得到代价与插值生成的解
    [particle(i).Cost, particle(i).Sol]=CostFunction(particle(i).Position);
    
    % Update Personal Best   更新个体最优
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    particle(i).Best.Sol=particle(i).Sol;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost  
        
        GlobalBest=particle(i).Best;        %i==1时，第一个粒子个体为全局最优；i>1时，若个体粒子的代价小于全局的代价，
                                               % 则进行更新，始终保持全局的代价最小
        
    end
    
end

% Array to Hold Best Cost Values at Each Iteration
BestCost=zeros(MaxIter,1);

%% PSO Main Loop

for it=1:MaxIter
    
    for i=1:nPop
        
        % x Part
        
        % Update Velocity
        particle(i).Velocity.x = w*particle(i).Velocity.x ...
            + c1*rand(VarSize).*(particle(i).Best.Position.x-particle(i).Position.x) ...
            + c2*rand(VarSize).*(GlobalBest.Position.x-particle(i).Position.x);
        
        % Update Velocity Bounds
        particle(i).Velocity.x = max(particle(i).Velocity.x,VelMin.x);
        particle(i).Velocity.x = min(particle(i).Velocity.x,VelMax.x);
        
        % Update Position
        particle(i).Position.x = particle(i).Position.x + particle(i).Velocity.x;
        
        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.x<VarMin.x | particle(i).Position.x>VarMax.x);
        particle(i).Velocity.x(OutOfTheRange)=-particle(i).Velocity.x(OutOfTheRange);
        
        % Update Position Bounds
        particle(i).Position.x = max(particle(i).Position.x,VarMin.x);
        particle(i).Position.x = min(particle(i).Position.x,VarMax.x);
        
        % y Part
        
        % Update Velocity
        particle(i).Velocity.y = w*particle(i).Velocity.y ...
            + c1*rand(VarSize).*(particle(i).Best.Position.y-particle(i).Position.y) ...
            + c2*rand(VarSize).*(GlobalBest.Position.y-particle(i).Position.y);
        
        % Update Velocity Bounds
        particle(i).Velocity.y = max(particle(i).Velocity.y,VelMin.y);
        particle(i).Velocity.y = min(particle(i).Velocity.y,VelMax.y);
        
        % Update Position
        particle(i).Position.y = particle(i).Position.y + particle(i).Velocity.y;
        
        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.y<VarMin.y | particle(i).Position.y>VarMax.y);
        particle(i).Velocity.y(OutOfTheRange)=-particle(i).Velocity.y(OutOfTheRange);
        
        % Update Position Bounds
        particle(i).Position.y = max(particle(i).Position.y,VarMin.y);
        particle(i).Position.y = min(particle(i).Position.y,VarMax.y);
        
        % Evaluation
        [particle(i).Cost, particle(i).Sol]=CostFunction(particle(i).Position);   %计算代价
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost     %如果当前代价低于最好代价，将当前信息保留至最优
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            particle(i).Best.Sol=particle(i).Sol;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost  %遍历100次迭代更新的粒子，i==1时，第一个粒子个体为全局最优；i>1时，若个体粒子的代价小于全局的代价，
                                               % 则进行更新，始终保持全局的代价最小
                GlobalBest=particle(i).Best;
            end
            
        end
        
        
    end
    
    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;  %遍历3个障碍物后，找到全局最优代价
    
    % Inertia Weight Damping
    w=w*wdamp;                          %  惯性权重衰减                      

    % Show Iteration Information
%     if GlobalBest.Sol.IsFeasible        %如果碰撞，Flag=' *'
%         Flag=' *';
%     else                              %如果不碰撞，Flag=Violation   
%         Flag=[', Violation = ' num2str(GlobalBest.Sol.Violation)];
%     end
%     disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it)) Flag]);
   
%    number=it/(MaxIter/4);
%    switch number
%        case 1
%        BestSol_1=GlobalBest.Sol; 
%        BestCost_1=BestCost(1:it);
%        case 2
%        BestSol_2=GlobalBest.Sol; 
%        BestCost_2=BestCost(1:it);
%        case 3
%        BestSol_3=GlobalBest.Sol;
%        BestCost_3=BestCost(1:it);
%        case 4
%        BestSol_4=GlobalBest.Sol;
%        BestCost_4=BestCost(1:it);
%    end
%     % Plot Solution
%     figure(1);
%     PlotSolution(GlobalBest.Sol,model);    %画生成的解
    %pause(0.01);
    
end
ipoint=GlobalBest.Position;
sol=GlobalBest.Sol;

% figure(1);
% PlotSolution(sol,model,ipoint);    %画生成的解
% title(strcat('iteration=', num2str(MaxIter)));
path=[sol.xx;sol.yy];



% plot(ipoint.x(1),ipoint.y(1),'ro');
% plot(ipoint.x(2),ipoint.y(2),'ro');
% plot(ipoint.x(3),ipoint.y(3),'ro');
% plot(ipoint.x(4),ipoint.y(4),'ro');
% plot(ipoint.x(5),ipoint.y(5),'ro');

% plot(0.5,1,'ro');
% plot(1.1,2.3,'ro');
% plot(1.8,3.8,'ro');
% plot(2.2,4.3,'ro');
% plot(3.05,5.2,'ro');
% % 用Tau理论插值得到xo,dxo,ddxo
% a=size(ipoint.x,2);  num=100;
% [xo,dxo,ddxo]=ypf_Tau(ipoint,[model.xs,model.ys],[model.xt,model.yt],a,num);
% 
% % Results
% figure(1);
% subplot(2,2,1);
% PlotSolution(BestSol_1,model);  %画生成的解
% title(strcat('iteration=', num2str(MaxIter/4)));
% 
% subplot(2,2,2);
% PlotSolution(BestSol_2,model);    %画生成的解
% title(strcat('iteration=', num2str(MaxIter/4*2)));
% 
% subplot(2,2,3);
% PlotSolution(BestSol_3,model);    %画生成的解
% title(strcat('iteration=', num2str(MaxIter/4*3)));
% 
% subplot(2,2,4);
% PlotSolution(BestSol_4,model);    %画生成的解
% title(strcat('iteration=', num2str(MaxIter)));
% 
% 
% figure(2);
% subplot(2,2,1);
% plot(BestCost_1,'LineWidth',2);
% xlabel('Iteration');
% ylabel('BestCostBw');
% grid on;
% subplot(2,2,2);
% plot(BestCost_2,'LineWidth',2);
% xlabel('Iteration');
% ylabel('BestCostBw');
% grid on;
% subplot(2,2,3);
% plot(BestCost_3,'LineWidth',2);
% xlabel('Iteration');
% ylabel('BestCostBw');
% grid on;
% subplot(2,2,4);
% plot(BestCost_4,'LineWidth',2);
% xlabel('Iteration');
% ylabel('BestCostBw');
% grid on;

