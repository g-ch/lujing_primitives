clear;

d = 3; % sample point length
Fov_half = [35, 20]; %field of view of the camera
Angle_h = [-180, -150, -120, -90, -70, -50, -30, -20, -10, 0, 10, 20, 30, 50, 70, 90, 120, 150];
Angle_v = [-40, -20, -10, 0, 10, 20, 40];
k1_xy = 4; % Goal directed coefficient
k1_z = 4; % Goal directed coefficient
k2_xy = 2; % Rotation coefficient
k2_z = 2; % Rotation coefficient
k3 = pi*0.1; % FOV coefficient
kk_h = 1; % FOV horisontal cost coefficient
kk_v = 1; % FOV vertical cost coefficient
v_max_ori = 5; % m/s, just reference
delt_t = 0.05; %time interval between two control points

% For the real limitation of the UAV in control procedure
a_limit = 3;
v_limit = 5;

Angle_h = Angle_h * pi / 180;
Angle_v = Angle_v * pi / 180;
Fov_half = Fov_half * pi / 180;
I = length(Angle_h);
J = length(Angle_v);
F_cost = zeros(I, J);  
% Set FOV cost, inside 0, outside increase linearly
for i = 1:I
    for j = 1:J
        if abs(Angle_h(i)) < Fov_half(1) && abs(Angle_v(j)) < Fov_half(2)
            continue;
        else
            delt_h_angle = min(abs(Angle_h(i)-Fov_half(1)), abs(Angle_h(i)+Fov_half(1)));
            delt_v_angle = min(abs(Angle_v(j)-Fov_half(2)), abs(Angle_v(j)+Fov_half(2)));
            F_cost(i,j) = (kk_h*delt_h_angle + kk_v*delt_v_angle)/(270/180*pi); % vertical max error + horizontal max error = 270бу
        end
    end
end

p_goal = [10, 10, 2];
p0 = [0, 0, 2];
v0 = [0, 0, 0];
a0 = [1, 1, 0]; 
ob = [2, 2, 0];
ob_r = 0.5;
yaw0 = pi/4;
theta_h_last = 0;
theta_v_last = 0;

theta=0:2*pi/3600:2*pi;
Circle1=ob(1)+ob_r*cos(theta);
Circle2=ob(2)+ob_r*sin(theta);
plot(Circle1,Circle2,'m','Linewidth',1);
hold on;
plot(p0(1), p0(2),'b*');
hold on;
% Velocity direction
quiver(p0(1),p0(2),p0(1)+v0(1),p0(2)+v0(2),'r','filled','LineWidth',2);
hold on;
% Yaw direction
quiver(p0(1),p0(2),p0(1)+cos(yaw0),p0(2)+sin(yaw0),'b','filled','LineWidth',2);

loop_times = 2000;
p_store=zeros(2000, 3);
v_store=zeros(2000, 3);
a_store=zeros(2000, 3);

for s_times = 1:loop_times
    disp(s_times);
    p_store(s_times,:)=p0;
    v_store(s_times,:)=v0;
    a_store(s_times,:)=a0;
    delt_p = p_goal - p0;
    phi_h = atan2(delt_p(2), delt_p(1)); % horizental offset angle
    phi_v = atan2(delt_p(3), sqrt(delt_p(1)*delt_p(1) + delt_p(2)*delt_p(2))); % vertical offset angle

    %calculate cost for sampled points
    cost = zeros(I*J, 4); % cost, theta_h, theta_v, FOV cost
    theta_h = 0;
    theta_v = 0;
    for i = 1:I
        for j = 1:J
            theta_h = Angle_h(i);
            theta_v = Angle_v(j);
            m = (i - 1) * J + j;
            cost(m, 1) = k1_xy*(yaw0+theta_h-phi_h)^2 + k1_z*(theta_v-phi_v)^2 + k2_xy*(theta_h-theta_h_last)^2+k2_z*(theta_v-theta_v_last)^2 + k3*F_cost(i,j);
            cost(m, 2) = theta_h;
            cost(m, 3) = theta_v;
            cost(m, 4) = (1-F_cost(i,j)) * d;
        end
    end
    % Rank by cost
    cost = sortrows(cost, 1);
    % max velocity is decreased concerning current velocity direction and goal
    % direction
    v_scale = max(dot(delt_p, v0) / norm(v0) / norm(delt_p), 0.5);
    v_max = v_max_ori * v_scale;
    
    for seq = 1:100
        [p,v,a,t,pf,vf,af] = motion_primitives(p0, v0, a0, yaw0, cost(seq,2), cost(seq,3), p_goal, cost(seq,4), v_max, delt_t);
        points_num = size(p);
        collision = 0;
        for p_seq = 1:points_num(1)
            if sqrt((p(p_seq, 1)-ob(1))^2 + (p(p_seq, 2)-ob(2))^2) < ob_r
                collision = 1;
                break;
            end
        end
        if collision == 0
            break;
        end
    end
    
    % p, v, a try to follow
    a0(1) = a(1,1) / abs(a(1,1)) * min(a_limit, abs(a(1,1)));
    a0(2) = a(1,2) / abs(a(1,2)) * min(a_limit, abs(a(1,2)));
%     a0(3) = a(1,3) / abs(a(1,3)) * min(a_limit, abs(a(1,3)));
    vT = [0,0,0];
    vT(1) = v(1,1); 
    if abs(vT(1) - v0(1)) > a0(1) * delt_t  % a limit
        vT(1) = v0(1) + a0(1) * delt_t;
    end
    if vT(1) > v_limit
        vT(1) = v_limit;
    end
    if vT(1) < -v_limit
        vT(1) = -v_limit;
    end
    vT(2) = v(1,2); 
    if abs(vT(2) - v0(2)) > a0(2) * delt_t  % a limit
        vT(2) = v0(2) + a0(2) * delt_t;
    end
    if vT(2) > v_limit
        vT(2) = v_limit;
    end
    if vT(2) < -v_limit
        vT(2) = -v_limit;
    end
    
    p0(1) = p0(1) + (v0(1)+vT(1))/2 * delt_t;
    p0(2) = p0(2) + (v0(2)+vT(2))/2 * delt_t;
    p0(3) = p(1,3);
%     p0(3) = p0(3) + (v0(3)+vT(3))/2 * delt_t;
    v0(1) = vT(1);
    v0(2) = vT(2);
    v0(3) = v(1,3);
%     v0(3) = vT(3);
    
%     p0 = p(1,:);
%     v0 = v(1,:);
%     a0 = a(1,:);

    % Yaw try to follow
    % yaw0 = atan2(v(1,2),v(1,1));
    yaw_desire = atan2(v(1,2),v(1,1));
    delt_yaw = yaw_desire - yaw0;
    if delt_yaw ~= 0 
        yaw0 = yaw0 + delt_yaw / abs(delt_yaw) * min(0.05, abs(delt_yaw));
    end
    
    theta_h_last = cost(1,2);
    theta_v_last = cost(1,3);
    
    if abs(p(1,1) - p_goal(1)) < 3.0 && abs(p(1,2) - p_goal(2)) < 3.0 % && abs(p(1,3) - p_goal(3)) < 3.0
        break
    end
    
    plot(p(1,1), p(1,2),'g.');
    hold on; 
end
plot(p(1,1), p(1,2),'b*');
grid on;

% for m = 1:1
%     [p,v,a,t,pf,vf,af] = motion_primitives(p0, v0, a0, yaw0, cost(m,2), cost(m,3), p_goal, cost(m,4), v_max, delt_t);
%     plot(p(:,1),p(:,2));
%     hold on;
% %     plot(t, p(:,3));
% %     hold on;
% end
% quiver(p0(1),p0(2),p0(1)+v0(1),p0(2)+v0(2),'r','filled','LineWidth',2);
% hold on;
% quiver(p0(1),p0(2),p0(1)+cos(yaw0),p0(2)+sin(yaw0),'b','filled','LineWidth',2);
% grid on;




