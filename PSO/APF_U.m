clc;
clear;
close all;
% 定义起点和终点
start_point = [1, 1];
end_point = [8, 8];

% 定义U型障碍物的位置和大小
obstacle_center = [5, 5];
obstacle_radius = 1.5;
obstacle_width = 2;

% 定义势场参数
attractive_force_gain = 1;
repulsive_force_gain = 10;

% 生成网格
[x, y] = meshgrid(0:0.1:10, 0:0.1:10);

% 计算吸引力势场
attractive_force = attractive_force_gain * (end_point - [x(:), y(:)]);

% 计算斥力势场（U型障碍物）
repulsive_force = zeros(size(attractive_force));
theta = atan2(y(:) - obstacle_center(2), x(:) - obstacle_center(1));
arc_mask = (theta > -pi/4) & (theta < pi/4); % 只在U型障碍物的前半部分施加斥力
repulsive_force(arc_mask, :) = repulsive_force_gain * (obstacle_radius - sqrt((x(arc_mask) - obstacle_center(1)).^2 + (y(arc_mask) - obstacle_center(2)).^2)) .* ...
    [cos(theta(arc_mask)), sin(theta(arc_mask))];

% 计算合力
total_force = attractive_force + repulsive_force;

% 可视化势场
figure;
quiver(x, y, reshape(attractive_force(:, 1), size(x)), reshape(attractive_force(:, 2), size(y)), 'b');
hold on;
quiver(x, y, reshape(repulsive_force(:, 1), size(x)), reshape(repulsive_force(:, 2), size(y)), 'r');
title('人工势场');
xlabel('X');
ylabel('Y');
legend('吸引力', '斥力');

% 模拟机器人运动
robot_pos = start_point;
trajectory = [robot_pos];
max_iterations = 100;

for i = 1:max_iterations
    % 计算机器人所受合力
    current_force = interp2(x, y, reshape(total_force(:, 1), size(x)), robot_pos(1), robot_pos(2), 'linear', 0);
    current_force = current_force + interp2(x, y, reshape(total_force(:, 2), size(y)), robot_pos(1), robot_pos(2), 'linear', 0);
    
    % 更新机器人位置
    robot_pos = robot_pos + current_force / norm(current_force);
    
    % 记录轨迹
    trajectory = [trajectory; robot_pos];
    
    % 检查是否达到终点
    if norm(robot_pos - end_point) < 0.1
        break;
    end
end

% 可视化机器人轨迹和U型障碍物
figure;
quiver(x, y, reshape(attractive_force(:, 1), size(x)), reshape(attractive_force(:, 2), size(y)), 'b');
hold on;
quiver(x, y, reshape(repulsive_force(:, 1), size(x)), reshape(repulsive_force(:, 2), size(y)), 'r');
plot(trajectory(:, 1), trajectory(:, 2), 'k', 'LineWidth', 2);

% 绘制U型障碍物
theta_arc = linspace(-pi/4, pi/4, 100);
obstacle_arc = obstacle_center + obstacle_radius * [cos(theta_arc); sin(theta_arc)];
plot(obstacle_arc(1, :), obstacle_arc(2, :), 'k', 'LineWidth', 2);

title('机器人路径和U型障碍物');
xlabel('X');
ylabel('Y');
legend('吸引力', '斥力', '机器人路径', 'U型障碍物');

