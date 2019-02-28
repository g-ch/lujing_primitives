clear;
p_goal = [6, 6, 6];
p0 = [0, 0, 0];
v0 = [1, 1, 0];
a0 = [0, 0, 0]; 
yaw0 = pi/4;
theta_h_last = 0;
theta_v_last = 0;

d = 3; % sample point length
Fov_half = [35, 20]; %field of view of the camera
Angle_h = [-180, -150, -120, -90, -70, -50, -30, -20, -10, 0, 10, 20, 30, 50, 70, 90, 120, 150];
Angle_v = [-70, -40, -20, -10, 0, 10, 20, 40, 70];
k1_xy = 2; % Goal directed coefficient
k1_z = 2; % Goal directed coefficient
k2_xy = 2; % Rotation coefficient
k2_z = 4; % Rotation coefficient
k3 = pi; % FOV coefficient
v_max = 3; % m/s, just reference
delt_t = 0.01; %time interval between two control points

Angle_h = Angle_h * pi / 180;
Angle_v = Angle_v * pi / 180;
Fov_half = Fov_half * pi / 180;
I = length(Angle_h);
J = length(Angle_v);
F_cost = zeros(I, J);  
% Set FOV cost, inside 0, outside increase linearly
for i = 1:I
    for j = 1:J
        if abs(Angle_h(i)) > Fov_half(1) && abs(Angle_v(j)) > Fov_half(2)
            F_cost(i,j) = (abs(Angle_h(i)) - Fov_half(1) + abs(Angle_v(j)) - Fov_half(2))/(270/180*pi);
        end
    end
end

delt_p = p_goal - p0;
phi_h = atan2(delt_p(1), delt_p(2)); % horizental offset angle
phi_v = atan2(delt_p(3), sqrt(delt_p(1)*delt_p(1) + delt_p(2)*delt_p(2))); % vertical offset angle

%calculate cost for sampled points
cost = zeros(I*J, 3);
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
    end
end

% Rank by cost
cost = sortrows(cost, 1);
v_scale = max(dot(delt_p, v0) / norm(v0) / norm(delt_p), 0.1);
v_max = v_max * v_scale;
disp(v_max);

for m = 1:10
    [p,v,a,t,pf,vf,af] = motion_primitives(p0, v0, a0, yaw0, cost(m,2), cost(m,3), p_goal, d, v_max, delt_t);
    plot(p(:,1),p(:,2));
    hold on;
    plot(t, p(:,3));
    hold on;
end
quiver(p0(1),p0(2),p0(1)+v0(1),p0(2)+v0(2),'r','filled','LineWidth',2);
hold on;
quiver(p0(1),p0(2),p0(1)+cos(yaw0),p0(2)+sin(yaw0),'b','filled','LineWidth',2);
grid on;




