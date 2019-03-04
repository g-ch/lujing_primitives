function [p,v,a,t,pf,vf,af] = motion_primitives(p0, v0, a0, yaw0, theta_h, theta_v, goal, d, v_max, delt_t)
disp(p0);
disp(v0);
disp(a0);

delt_x = d*cos(theta_v) * cos(theta_h+yaw0);
delt_y = d*cos(theta_v) * sin(theta_h+yaw0);
delt_z = d*sin(theta_v);
pf = p0 + [delt_x, delt_y, delt_z];

l = goal - pf;
vf = (v_max / norm(l)) * l;
vf(3) = 0; % Note: 0 maybe better, for the p curve wont go down to meet the vf
af = [0,0,0];

disp(pf)
disp(vf)

% Choose the time as running in average velocity
decay_parameter = 0.5;
T1 = 2*delt_x/(vf(1)+v0(1)) * decay_parameter;
T2 = 2*delt_y/(vf(2)+v0(2)) * decay_parameter;
T3 = 2*delt_z/(vf(3)+v0(3)) * decay_parameter;
if  T1 > 1000 % eliminate infinite value
    T1 = 0;
end
if  T2 > 1000
    T2 = 0;
end
if  T3 > 1000
    T3 = 0;
end
T = max([T1, T2, T3]);
times = floor(T / delt_t);
p = zeros(times,3);
v = zeros(times,3);
a = zeros(times,3);
t = delt_t : delt_t : times*delt_t;

% calculate optimal jerk controls by Mark W. Miller
for ii = 1:3 % x, y, z axis
    delt_a = af(ii) - a0(ii);
    delt_v = vf(ii) - v0(ii) - a0(ii)*T;
    delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;
    %   if vf is not free
    alpha = delt_a*60/T^3 - delt_v*360/T^4 + delt_p*720/T^5;
    beta = -delt_a*24/T^2 + delt_v*168/T^3 - delt_p*360/T^4;
    gamma = delt_a*3/T - delt_v*24/T^2 + delt_p*60/T^3;

    %    if vf is  free
%     alpha = -delt_a*7.5/T^3  + delt_p*45/T^5;
%     beta = delt_a*7.5/T^2 - delt_p*45/T^4;
%     gamma = -delt_a*1.5/T + delt_p*15/T^3;
    for jj = 1:times
        tt = t(jj);
        p(jj,ii) = alpha/120*tt^5 + beta/24*tt^4 + gamma/6*tt^3 + a0(ii)/2*tt^2 + v0(ii)*tt + p0(ii);
        v(jj,ii) = alpha/24*tt^4 + beta/6*tt^3 + gamma/2*tt^2 + a0(ii)*tt + v0(ii);
        a(jj,ii) = alpha/6*tt^3 + beta/2*tt^2 + gamma*tt + a0(ii);
    end
end

end



