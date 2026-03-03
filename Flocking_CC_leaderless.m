clc; clear; close all;


n = 13;                     
max_iter = 200;            
delta = 0.15;                
beta = 2;                   
epsilon = 5;                



x0 = [-2,-0.7,0.7,1,1.3,1.6,1.3,0.7,-0.7,-1.3,-1.6,-1.3,0];
y0 = 2*[1.6,2.6,2.6,1.6,0.5,-0.5,-1.6,-3,-3,-1.6,-0.5,0.5,0];
z0 = zeros(1, n);  


vx0 = [1.2, 0.1, -0.1, -0.5, -0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, 0.2, 0.05];
vy0 = [-0.2, -0.2, -0.5, -0.2, -1, 1.3, 0.2, 0.2, 1.2, 2, 0.2, -0.1, 0.15];
vz0 = 0.1 * rand(1, n);  


X = [x0; y0; z0]';  
V = [vx0; vy0; vz0]';  


X_history = zeros(max_iter+1, n, 3);  
X_history(1,:,:) = X;

V_history = zeros(max_iter+1, n, 3);  
V_history(1,:,:) = V;


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


figure(1);
h_scatter = scatter3(X(:,1), X(:,2), X(:,3), 50, 'filled', 'MarkerEdgeColor','k');
hold on;
title('三维实时运动轨迹');
xlabel('X坐标'); ylabel('Y坐标'); zlabel('Z坐标');
axis equal; grid on;
xlim([-10 10]);  
ylim([-16 12]);
zlim([-4 4]);  


for t = 1:max_iter
    delta_V = zeros(n, 3);  
    
    for i = 1:n
        for j = 1:n
            if i == j || A(i,j) == 0  
                continue;
            end
            
            
            dx = X(j,1) - X(i,1);
            dy = X(j,2) - X(i,2);
            dz = X(j,3) - X(i,3);
            e_ij = sqrt(dx^2 + dy^2 + dz^2);  
            dvx = V(j,1) - V(i,1);
            dvy = V(j,2) - V(i,2);
            dvz = V(j,3) - V(i,3);
            ev_ij = sqrt(dvx^2 + dvy^2 + dvz^2);  
            
            
            if t==1
                f_ij = A(i,j);
            else
                if e_ij <= epsilon
                    f_ij = 1 / (1 + e_ij^beta);
                else
                    f_ij = -1 / (1 + e_ij^beta)-0.1 ;
                end
            end
            
            delta_V(i,:) = delta_V(i,:) +  abs(f_ij) * (sign(f_ij) * V(j,:) - V(i,:));
            f_ji_1(i,:)=f_ij;
        end
    end
    
    
    V = V + delta_V * delta;  
    X = X + V * delta;  
    
    
    X_history(t+1,:,:) = X;    
    V_history(t+1,:,:) = V;
    f_ji_2(t+1,:,:) = f_ji_1;
    
    set(h_scatter, 'XData', X(:,1), 'YData', X(:,2), 'ZData', X(:,3));
    drawnow;  
    pause(0.05); 
end



figure(2);
hold on;
colors = lines(n);  



X_initial = squeeze(X_history(1,:,:));    
V_initial = squeeze(V_history(1,:,:));    


X_final = squeeze(X_history(end,:,:));    
V_final = squeeze(V_history(end,:,:));    


traj_plots = gobjects(n,1);  
for k = 1:n
    x_traj = squeeze(X_history(:,k,1));
    y_traj = squeeze(X_history(:,k,2));
    z_traj = squeeze(X_history(:,k,3));
    traj_plots(k) = plot3(x_traj, y_traj, z_traj,...
        'Color', 'b', 'LineWidth', 1.5);
end


h_start = scatter3(X_initial(:,1), X_initial(:,2), X_initial(:,3),...
    80, 'k', 'filled', '^');
h_end = scatter3(X_final(:,1), X_final(:,2), X_final(:,3),...
    80, 'r', 'filled', 'o');


scale_factor = 0.3;  


h_initial = quiver3(X_initial(:,1), X_initial(:,2), X_initial(:,3),...
    V_initial(:,1), V_initial(:,2), V_initial(:,3),...
    scale_factor, 'Color', [0 0.8 0.8], 'LineWidth', 1.5,...
    'MaxHeadSize', 0.5);


h_final = quiver3(X_final(:,1), X_final(:,2), X_final(:,3),...
    V_final(:,1), V_final(:,2), V_final(:,3),...
    scale_factor, 'Color', [1 0 1], 'LineWidth', 1.5,...
    'MaxHeadSize', 0.5);


xlabel('X-axis(m)'); ylabel('Y-axis(m)'); zlabel('Z-axis(m)');
axis equal; grid on;
view(30, 30);


legend([traj_plots(1), h_start, h_end, h_initial, h_final],...
    {'Agent trajectory', 'Initial position', 'Termination position', 'initial velocity', 'final velocity'},...
    'Location', 'best');


all_x = X_history(:,:,1);
all_y = X_history(:,:,2);
all_z = X_history(:,:,3);
xlim([min(all_x(:))-1, max(all_x(:))+1])
ylim([min(all_y(:))-1, max(all_y(:))+1])
zlim([min(all_z(:))-1, max(all_z(:))+1])




figure(3)
set(gcf,'Position',[200 200 800 400])  


hold on
plot(0:max_iter, V_history(:,:,1),...
        'Color','g',...  
        'LineWidth',2)
hold on
plot(0:max_iter, V_history(:,:,2),...
        'Color','b',...  
        'LineWidth',2)
hold on
plot(0:max_iter, V_history(:,:,3),...
        'Color','r',...  
        'LineWidth',2)


grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')




figure(4)
plot(0:max_iter, V_history(:,:,1),...
        'Color','b',...  
        'LineWidth',2)
hold on;
title('X-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

figure(5)
plot(0:max_iter, V_history(:,:,2),...
        'Color','b',...  
        'LineWidth',2)
hold on;
title('Y-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

figure(6)
plot(0:max_iter, V_history(:,:,3),...
        'Color','b',...  
        'LineWidth',2)
hold on;
title('Z-axis')
grid on
xlabel('Time')
ylabel('Velocity (m/s)')
legend('Location','northwest')

