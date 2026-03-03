clc; clear; close all;

%% 参数设置
n = 13;                     % 原系统节点数
max_iter = 100;            % 最大迭代次数
delta = 0.3;                % 时间间隔Δ
beta = 2;                   % 权重衰减率
epsilon = 10;               % 合作范围

%% 初始化状态（三维坐标）
% 扩展系统节点数
extended_n = 2*n;  % 每个原节点扩展为2个子节点

% 初始位置 (x, y, z)
x0 = [-2,-0.7,0.7,1,1.3,1.6,1.3,0.7,-0.7,-1.3,-1.6,-1.3,-0.5,2,0.7,-0.7,-1,-1.3,-1.6,-1.3,-0.7,0.7,1.3,1.6,1.3,0.5];
y0 = 2*[1.6,2.6,2.6,1.6,0.5,-0.5,-1.6,-3,-3,-1.6,-0.5,0.5,0.1,-1.6,-2.6,-2.6,-1.6,-0.5,0.5,1.6,3,3,1.6,0.5,-0.5,-0.1];
z0 = zeros(1, extended_n); 

% 初始速度 (vx, vy, vz)
vx0 = [1.2, 0.1, -0.1, -0.5, -0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, 0.2, 0.005,-1.2, -0.1, 0.1, 0.5, 0.2, 0.3, 0.2, 0.1, -0.1, -0.2, -0.2, -0.2, -0.005];
vy0 = [-0.2, -0.2, -0.5, -0.2, -1, 1.3, 0.2, 0.2, 1.2, 2, 0.2, -0.1, 0.15,0.2, 0.2, 0.5, 0.2, 1, -1.3, -0.2, -0.2, -1.2, -2, -0.2, 0.1, -0.15];
vz0 = zeros(1, extended_n); 

% 合并为三维状态矩阵
X = [x0; y0; z0]';  % extended_n×3矩阵
V = [vx0; vy0; vz0]';  % extended_n×3矩阵

% 创建存储历史轨迹的三维数组
X_history = zeros(max_iter+1, extended_n, 3);
X_history(1,:,:) = X;
V_history = zeros(max_iter+1, extended_n, 3);
V_history(1,:,:) = V;

%% 邻接矩阵A（原系统13×13）
A = [
    0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   -0.1, 0;
    0.1, 0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0;
    -0.1,   0, 0, 0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0.5;
    0,   0,   0, 0, 0, 0.1,   0,   0,   0,   0,   0,   0,   0;
    0,   0,   0,   0.1, 0, -0.1, 0,   0,   0,   0,   0,   0,   0.5;
    0,   0,   0,   0,   0, 0, 0.1, 0,   0,   0,   0,   0,   0;
    0,   0,   0,   0,   0,   0, 0, 0.1, 0,   -0.5,   0,   0,   0.5;
    0,   0,   0,   0,   0,   0,   0.1, 0, 0.1, 0,   0,   0,   0;
    0,   0,   0,   0.1,   0,   0,   0,   0.1, 0, 0.1, 0,   0,   0;
    0,   0,   0,   0,   0,   0,   0,   0,   0.1, 0, 0.1, 0,   0.5;
    0,   0,   0,   0,   0,   0,   0,   0,   0,   -0.1, 0, 0.1, 0;
    0.1, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0.1, 0, 0.5;
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
];

%% 初始化三维动态轨迹图
figure(1);
h_scatter = scatter3(X(:,1), X(:,2), X(:,3), 50, 'filled', 'MarkerEdgeColor','k');
hold on;
title('三维实时运动轨迹');
xlabel('X坐标'); ylabel('Y坐标'); zlabel('Z坐标');
axis equal; grid on;
xlim([-5 5]);  
ylim([-8 6]);
zlim([-2 2]);

%% 主循环（含三维动画）
for t = 1:max_iter
    % 初始化扩展邻接矩阵B（对称矩阵）
    B = zeros(extended_n, extended_n);
    
    % 构建对称连接拓扑
    for i = 1:n       % 原节点索引
        for j = 1:n   % 原节点索引
            if i == j || A(i,j) == 0  
                continue;
            end
            
            % 获取扩展节点索引
            i_odd = 2*i-1;  % 奇数节点
            i_even = 2*i;    % 偶数节点
            j_odd = 2*j-1;
            j_even = 2*j;
            
            % 计算原节点间的三维距离
            pos_i = mean(X([i_odd, i_even], :));  % 原节点位置取均值
            pos_j = mean(X([j_odd, j_even], :));
            dx = pos_j(1) - pos_i(1);
            dy = pos_j(2) - pos_i(2);
            dz = pos_j(3) - pos_i(3);
            e_ij = norm([dx, dy, dz]);
            
            % 计算交互权重
            if e_ij <= epsilon
                f_ij = 1 / (1 + e_ij^beta);
                % 直连模式
                % B(i_odd, j_odd) = f_ij;
                 B(j_odd, i_odd) = f_ij;  % 对称连接
                % B(i_even, j_even) = f_ij;
                 B(j_even, i_even) = f_ij;
            else
                f_ij = 1 / (1 + e_ij^beta) - 1; % 防止负权重
                % 交叉连接模式
                % B(i_odd, j_even) = (f_ij);
                 B(j_even, i_odd) = (f_ij);
                 % B(i_even, j_odd) = (f_ij);
                 B(j_odd, i_even) = (f_ij);
            end
        end  
    end

    %%判断是否与领导者有链接
    if B(:,13) == 0
           B(:,13) = 0;
        else   
           B(:,13)=0.3;
    end
    if B(:,26) == 0
           B(:,26) = 0;
        else
           B(:,26)=0.3;
    end
    % 计算速度变化量
    delta_V = zeros(extended_n, 3);
    for i = 1:extended_n
        for j = 1:extended_n
            if B(i,j) > 0
                % 标准一致性协议
                delta_V(i,:) = delta_V(i,:) + abs(B(i,j))*(sign(B(i,j))*V(j,:) - V(i,:));
            end
        end
    end
    
    % 更新速度和位置（显式欧拉法）
    V = V + delta_V * delta;
    X = X + V * delta;
    
    % 记录轨迹
    X_history(t+1,:,:) = X;    
    V_history(t+1,:,:) = V;
    
    % 更新三维动态图
    set(h_scatter, 'XData', X(:,1), 'YData', X(:,2), 'ZData', X(:,3));
    drawnow;  
    pause(0.05); 
end



%% 修改后的三维完整轨迹图（含速度箭头）
figure(2);
hold on;
colors = lines(extended_n);  

% ===== 新增部分：提取初始/最终状态 =====
% 获取初始状态
X_initial = squeeze(X_history(1,:,:));    % 初始位置 n×3
V_initial = squeeze(V_history(1,:,:));    % 初始速度 n×3

% 获取最终状态
X_final = squeeze(X_history(end,:,:));    % 最终位置 n×3
V_final = squeeze(V_history(end,:,:));    % 最终速度 n×3

% ===== 绘制轨迹（修改句柄管理）=====
traj_plots = gobjects(extended_n,1);  % 预分配图形句柄
for k = 1:extended_n
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
scale_factor = 0.8;  % 箭头缩放系数（根据实际数据调整）

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
hold on;
plot3(X_history(:,13,1), X_history(:,13,2), X_history(:,13,3),'g-','linewidth', 5);hold on;
hold on;
plot3(X_history(:,26,1), X_history(:,26,2), X_history(:,26,3),'g-','linewidth', 5);hold on;

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