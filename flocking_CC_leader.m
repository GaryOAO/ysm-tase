clc; clear; close all;


n = 13;                     
max_iter = 100;            
delta = 0.3;                
beta = 2;                   
epsilon = 10;               



extended_n = 2*n;  


x0 = [-2,-0.7,0.7,1,1.3,1.6,1.3,0.7,-0.7,-1.3,-1.6,-1.3,-0.5,2,0.7,-0.7,-1,-1.3,-1.6,-1.3,-0.7,0.7,1.3,1.6,1.3,0.5];
y0 = 2*[1.6,2.6,2.6,1.6,0.5,-0.5,-1.6,-3,-3,-1.6,-0.5,0.5,0.1,-1.6,-2.6,-2.6,-1.6,-0.5,0.5,1.6,3,3,1.6,0.5,-0.5,-0.1];
z0 = zeros(1, extended_n); 


vx0 = [1.2, 0.1, -0.1, -0.5, -0.2, -0.3, -0.2, -0.1, 0.1, 0.2, 0.2, 0.2, 0.005,-1.2, -0.1, 0.1, 0.5, 0.2, 0.3, 0.2, 0.1, -0.1, -0.2, -0.2, -0.2, -0.005];
vy0 = [-0.2, -0.2, -0.5, -0.2, -1, 1.3, 0.2, 0.2, 1.2, 2, 0.2, -0.1, 0.15,0.2, 0.2, 0.5, 0.2, 1, -1.3, -0.2, -0.2, -1.2, -2, -0.2, 0.1, -0.15];
vz0 = zeros(1, extended_n); 


X = [x0; y0; z0]';  
V = [vx0; vy0; vz0]';  


X_history = zeros(max_iter+1, extended_n, 3);
X_history(1,:,:) = X;
V_history = zeros(max_iter+1, extended_n, 3);
V_history(1,:,:) = V;


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


figure(1);
h_scatter = scatter3(X(:,1), X(:,2), X(:,3), 50, 'filled', 'MarkerEdgeColor','k');
hold on;
title('三维实时运动轨迹');
xlabel('X坐标'); ylabel('Y坐标'); zlabel('Z坐标');
axis equal; grid on;
xlim([-5 5]);  
ylim([-8 6]);
zlim([-2 2]);


for t = 1:max_iter
    
    B = zeros(extended_n, extended_n);
    
    
    for i = 1:n       
        for j = 1:n   
            if i == j || A(i,j) == 0  
                continue;
            end
            
            
            i_odd = 2*i-1;  
            i_even = 2*i;    
            j_odd = 2*j-1;
            j_even = 2*j;
            
            
            pos_i = mean(X([i_odd, i_even], :));  
            pos_j = mean(X([j_odd, j_even], :));
            dx = pos_j(1) - pos_i(1);
            dy = pos_j(2) - pos_i(2);
            dz = pos_j(3) - pos_i(3);
            e_ij = norm([dx, dy, dz]);
            
            
            if e_ij <= epsilon
                f_ij = 1 / (1 + e_ij^beta);
                
                
                 B(j_odd, i_odd) = f_ij;  
                
                 B(j_even, i_even) = f_ij;
            else
                f_ij = 1 / (1 + e_ij^beta) - 1; 
                
                
                 B(j_even, i_odd) = (f_ij);
                 
                 B(j_odd, i_even) = (f_ij);
            end
        end  
    end

    
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
    
    delta_V = zeros(extended_n, 3);
    for i = 1:extended_n
        for j = 1:extended_n
            if B(i,j) > 0
                
                delta_V(i,:) = delta_V(i,:) + abs(B(i,j))*(sign(B(i,j))*V(j,:) - V(i,:));
            end
        end
    end
    
    
    V = V + delta_V * delta;
    X = X + V * delta;
    
    
    X_history(t+1,:,:) = X;    
    V_history(t+1,:,:) = V;
    
    
    set(h_scatter, 'XData', X(:,1), 'YData', X(:,2), 'ZData', X(:,3));
    drawnow;  
    pause(0.05); 
end




figure(2);
hold on;
colors = lines(extended_n);  



X_initial = squeeze(X_history(1,:,:));    
V_initial = squeeze(V_history(1,:,:));    


X_final = squeeze(X_history(end,:,:));    
V_final = squeeze(V_history(end,:,:));    


traj_plots = gobjects(extended_n,1);  
for k = 1:extended_n
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


scale_factor = 0.8;  


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
hold on;
plot3(X_history(:,13,1), X_history(:,13,2), X_history(:,13,3),'g-','linewidth', 5);hold on;
hold on;
plot3(X_history(:,26,1), X_history(:,26,2), X_history(:,26,3),'g-','linewidth', 5);hold on;


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
