clc; clear; close all;

%% 参数设置
n = 13;                     % 节点数
max_iter = 200;            % 最大迭代次数
delta = 0.15;                % 时间间隔Δ
beta = 2;                   % 权重衰减率
epsilon = 5;                % 合作范围  分别选择3和5

%% 初始化状态（三维坐标）
% 初始位置 (x, y, z)
x0 = [-2,-0.7,0.7,1,1.3,1.6,1.3,0.7,-0.7,-1.3,-1.6,-1.3,0];
y0 = 2*[1.6,2.6,2.6,1.6,0.5,-0.5,-1.6,-3,-3,-1.6,-0.5,0.5,0];
z0 = zeros(1, n);  % 假设初始z坐标为0（可根据需求修改）

% 初始速度 (vx, vy, vz)
vx0 = [1.2, 0.1, -0.1, -0.5, -0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, 0.2, 0.05];
vy0 = [-0.2, -0.2, -0.5, -0.2, -1, 1.3, 0.2, 0.2, 1.2, 2, 0.2, -0.1, 0.15];
vz0 = 0.1 * rand(1, n);  % 随机生成z方向初始速度（示例）

% 合并为三维状态矩阵
X = [x0; y0; z0]';  % n×3矩阵
V = [vx0; vy0; vz0]';  % n×3矩阵

% 创建存储历史轨迹的三维数组 (时间步×节点×坐标)
X_history = zeros(max_iter+1, n, 3);  % 包含初始状态
X_history(1,:,:) = X;

V_history = zeros(max_iter+1, n, 3);  % 包含初始状态
V_history(1,:,:) = V;

%% 邻接矩阵A（保持不变，与空间维度无关）
A = [
    0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   -0.1, 0;
    0.1, 0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0;
    -0.1,   0.1, 0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0;
    0,   0,   0.1, 0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0;
    0,   0,   0,   0.1, 0, 0.1, 0,   0,   0,   0,   0,   0,   -0.5;
    0,   0,   0,   0,   0.1, 0, 0.1, 0,   0,   0,   0,   0,   0;
    0,   0,   0,   0,   0,   0.1, 0, -0.1, 0,   0,   0,   0,   0.5;
    0,   0,   0,   0,   0,   0,   0.1, 0, 0.1, 0,   0,   0,   0;
    0,   0,   0,   0.1,   0,   0,   0,   0.1, 0, 0.1, 0,   0,   0;
    0,   0,   0,   0,   0,   0,   0,   0,   0.1, 0, 0.1, 0,   0.5;
    0,   0,   0,   0,   0,   0,   0,   0,   0,   -0.1, 0, -0.1, 0;
    0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0.1, 0, 0;
    0,   0,   -0.5,   0,   0,   0,   0.5,   0,   0,   0.3,   0,   0,   0
];

%% 初始化三维动态轨迹图
figure(1);
h_scatter = scatter3(X(:,1), X(:,2), X(:,3), 50, 'filled', 'MarkerEdgeColor','k');
hold on;
title('三维实时运动轨迹');
xlabel('X坐标'); ylabel('Y坐标'); zlabel('Z坐标');
axis equal; grid on;
xlim([-10 10]);  
ylim([-16 12]);
zlim([-4 4]);  % 根据z方向数据调整范围

%% 主循环（含三维动画）
for t = 1:max_iter
    delta_V = zeros(n, 3);  % 存储三维速度变化
    
    for i = 1:n
        for j = 1:n
            if i == j || A(i,j) == 0  
                continue;
            end
            
            % 计算三维欧氏距离
            dx = X(j,1) - X(i,1);
            dy = X(j,2) - X(i,2);
            dz = X(j,3) - X(i,3);
            e_ij = sqrt(dx^2 + dy^2 + dz^2);  % 三维距离
            dvx = V(j,1) - V(i,1);
            dvy = V(j,2) - V(i,2);
            dvz = V(j,3) - V(i,3);
            ev_ij = sqrt(dvx^2 + dvy^2 + dvz^2);  % 三维距离
            % e_ij = e_ij + 0.5*ev_ij;
            % 计算交互权重
            if t==1
                f_ij = A(i,j);
            else
                if e_ij <= epsilon
                    f_ij = 1 / (1 + e_ij^beta);
                else
                    f_ij = -1 / (1 + e_ij^beta)-0.1 ;
                end
            end
            % 累加三维速度变化
            delta_V(i,:) = delta_V(i,:) +  abs(f_ij) * (sign(f_ij) * V(j,:) - V(i,:));
            f_ji_1(i,:)=f_ij;
        end
    end
    
    % 更新速度和位置（三维）
    V = V + delta_V * delta;  
    X = X + V * delta;  
    
    % 记录轨迹
    X_history(t+1,:,:) = X;    
    V_history(t+1,:,:) = V;
    f_ji_2(t+1,:,:) = f_ji_1;
    % 更新三维动态图
    set(h_scatter, 'XData', X(:,1), 'YData', X(:,2), 'ZData', X(:,3));
    drawnow;  
    pause(0.05); 
end


%% 修改后的三维完整轨迹图（含速度箭头）
figure(2);
hold on;
colors = lines(n);  

% ===== 新增部分：提取初始/最终状态 =====
% 获取初始状态
X_initial = squeeze(X_history(1,:,:));    % 初始位置 n×3
V_initial = squeeze(V_history(1,:,:));    % 初始速度 n×3

% 获取最终状态
X_final = squeeze(X_history(end,:,:));    % 最终位置 n×3
V_final = squeeze(V_history(end,:,:));    % 最终速度 n×3

% ===== 绘制轨迹（修改句柄管理）=====
traj_plots = gobjects(n,1);  % 预分配图形句柄
for k = 1:n
    x_traj = squeeze(X_history(:,k,1));
    y_traj = squeeze(X_history(:,k,2));
    z_traj = squeeze(X_history(:,k,3));
    traj_plots(k) = plot3(x_traj, y_traj, z_traj,...
        'Color', 'b', 'LineWidth', 1.5);
end

% ===== 绘制标记点 =====
h_start = scatter3(X_initial(:,1), X_initial(:,2), X_initial(:,3),...
    80, 'k', 'filled', '^');
h_end = scatter3(X_final(:,1), X_final(:,2), X_final(:,3),...
    80, 'r', 'filled', 'o');

% ===== 新增：绘制速度箭头 =====
scale_factor = 0.3;  % 箭头缩放系数（根据实际数据调整）

% 初始速度箭头（青色）
h_initial = quiver3(X_initial(:,1), X_initial(:,2), X_initial(:,3),...
    V_initial(:,1), V_initial(:,2), V_initial(:,3),...
    scale_factor, 'Color', [0 0.8 0.8], 'LineWidth', 1.5,...
    'MaxHeadSize', 0.5);

% 最终速度箭头（洋红色）
h_final = quiver3(X_final(:,1), X_final(:,2), X_final(:,3),...
    V_final(:,1), V_final(:,2), V_final(:,3),...
    scale_factor, 'Color', [1 0 1], 'LineWidth', 1.5,...
    'MaxHeadSize', 0.5);

% ===== 图形美化 =====
xlabel('X-axis(m)'); ylabel('Y-axis(m)'); zlabel('Z-axis(m)');
axis equal; grid on;
view(30, 30);

% 创建复合图例（只显示第一个轨迹示例）
legend([traj_plots(1), h_start, h_end, h_initial, h_final],...
    {'Agent trajectory', 'Initial position', 'Termination position', 'initial velocity', 'final velocity'},...
    'Location', 'best');

% 优化显示范围
all_x = X_history(:,:,1);
all_y = X_history(:,:,2);
all_z = X_history(:,:,3);
xlim([min(all_x(:))-1, max(all_x(:))+1])
ylim([min(all_y(:))-1, max(all_y(:))+1])
zlim([min(all_z(:))-1, max(all_z(:))+1])



%% 绘制速度图（图3）
figure(3)
set(gcf,'Position',[200 200 800 400])  % 设置窗口尺寸

% 绘制个体节点速度曲线
hold on
plot(0:max_iter, V_history(:,:,1),...
        'Color','g',...  % 浅灰色半透明
        'LineWidth',2)
hold on
plot(0:max_iter, V_history(:,:,2),...
        'Color','b',...  
        'LineWidth',2)
hold on
plot(0:max_iter, V_history(:,:,3),...
        'Color','r',...  % 浅灰色半透明
        'LineWidth',2)

% 图形美化
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')




figure(4)
plot(0:max_iter, V_history(:,:,1),...
        'Color','b',...  % 浅灰色半透明
        'LineWidth',2)
hold on;
title('X-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

figure(5)
plot(0:max_iter, V_history(:,:,2),...
        'Color','b',...  % 浅灰色半透明
        'LineWidth',2)
hold on;
title('Y-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

figure(6)
plot(0:max_iter, V_history(:,:,3),...
        'Color','b',...  % 浅灰色半透明
        'LineWidth',2)
hold on;
title('Z-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

