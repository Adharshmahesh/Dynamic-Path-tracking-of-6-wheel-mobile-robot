function [log_error, log_z, log_torque, log_Md] = controller_WP(w, z)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State of the robot z = [X, Y, Vx, Vy, Yaw, Yaw_rate]
% z(1), z(2) is the X and Y position of the robot in ground coordinate 
% z(3), z(4) is the X and Y velocity of the robot in robot frame
% z(5), z(6) is the yaw angle and yaw angle rate 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Vehicle Model Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global Mc_desired Mdhat Fy_total Fxd m Md Mc;

m = 6762; % m in kg
tr = 0.547; % Tire radius in m
Iz = 13201; % Moment of Inertia in kgm/s^2
lf = 1.8788; % Perpendicular length from CG to front axle in m
lr = 1.3216; % Perpendicular length from CG to rear axle in m
lm = 0.2784; % Perpendicular length from CG to middle axle in m
lv = 1; % Length of the vehicle in m
lw = 1.948; % Distance between left and right wheels


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial Conditions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts = 0.01; % Time step in s
lpv = 0; % Waypoint tracking preview distance in m
lv = 1; % Length of the vehicle in m
[n, ~] = size(w);
i = 2;
j = 1;
t_final = 400;
log_z(j, :) = [0,z]; 
yaw_rate = 0; % Yaw rate in rad/s
error_lateral = 0; % Lateral deviation in m
error_v = 0; 
error_v_integral = 0;
error_yaw = 0; 
error_yaw_rate = 0;
log_error(j, :) = [0, 0, 0, error_v, 0, 0, error_yaw, error_lateral];
u = 0;
log_torque(j, :) = [0, 0, 0, 0, 0, 0, 0];
log_slip_angle(j,:) = [0, 0, 0, 0, 0, 0, 0];
log_Mc(j, :) = [0, 0, 0];
log_Md(j, :) = [0, 0, 0, 0, 0];
log_Fxd(j, :) = [0, 0];
Cs = 17453; % Cornering stiffness of tires
Fy = zeros(1,6); % Lateral force in tire
alpha = zeros(1,6); % Side slip angle
Fx_des = zeros(6,1); % Longitudinal force
g = 9.81; % acceleration due to gravity in m/s^2
Fz1 = (m/6)*g; % Vertical tire force of tire 1
Fz2 = (m/6)*g; % Vertical tire force of tire 2
Fz3 = (m/6)*g; % Vertical tire force of tire 3
Fz4 = (m/6)*g; % Vertical tire force of tire 4
Fz5 = (m/6)*g; % Vertical tire force of tire 5
Fz6 = (m/6)*g; % Vertical tire force of tire 6


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kvp = 7; % Proportional Control gain for speed 
Kvi = 1; % Integral Control gain for speed 
Kphi_p =7; % Proportional Control gain for yaw 
Kphi_d = 10; % Derivative Control gain for yaw 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Disturbance Moment Observer Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p= 10;
eta = 25*Iz; % Observer gain
l = p*Iz; % Observer gain
Mc = 0; %  Yaw Moment
Md = 0; % Disturbance friction moment
rhat = 0; % Estimated Yaw angular velocity
Mdhat = 0; % Estimated Disturbance friction moment

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Force limit Parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tl = 3416; % Torque Limit in Nm
fl = tl/tr; % Force Limit =  Torque Limit / Tire radius

for t=1:ts:t_final
   
   % Waypoint tracking algorithm
    
   w1 = w(i-1,:);
   w2 = w(i,:);
   d = sqrt((w2(2)-w1(2))^2+(w2(1)-w1(1))^2);
   yaw_w = atan2((w2(2)-w1(2)),(w2(1)-w1(1)));
   u = ((z(1) - w1(1))*(w2(1)-w1(1)) + (z(2)-w1(2))*(w2(2)-w1(2)))/(d^2);
  
   while u>1
      if i==n
          break;
      end
      
      i = i+1;
      w1 = w(i-1,:);
      w2 = w(i,:);
      d = sqrt((w2(2)-w1(2))^2+(w2(1)-w1(1))^2);
      yaw_w = atan2((w2(2)-w1(2)),(w2(1)-w1(1)));
      u = ((z(1) - w1(1))*(w2(1)-w1(1)) + (z(2)-w1(2))*(w2(2)-w1(2)))/(d^2);
   
   end
   
   if i==n
        break;
    end
    xd = w1(1)+ u*(w2(1) - w1(1));
    yd = w1(2)+ u*(w2(2) - w1(2));
    
    if u>0
        vd = w1(3)+ u*(w2(3) - w1(3));
    else
        vd = w1(3);
    end
    
    vd = vd*exp(-0.5*abs(error_lateral));
    
    
    if abs(z(3))<lv
        lpv = lv;
    else
        lpv = abs(z(3))*lv;
    end
    
    ipv = i;
    dr = sqrt((w(ipv,1)-xd)^2 + (w(ipv,2)-yd)^2);
    if lpv>dr
        lpv_r = lpv - dr;
        while(lpv_r>0)
            if ipv==n
                break;
            end
            ipv = ipv+1;
            dr = sqrt((w(ipv,1)-w(ipv-1,1))^2+(w(ipv,2)-w(ipv-1,2))^2);
            lpv_r = lpv_r - dr;
        end
        lpv_r = dr + lpv_r;
        yaw_wpv = atan2((w(ipv,2) - w(ipv-1,2)), (w(ipv,1) - w(ipv-1,1)));
        xpv = w(ipv-1,1) + lpv_r*cos(yaw_wpv);
        ypv = w(ipv-1,2) + lpv_r*sin(yaw_wpv);
        if ipv==n
            xpv = w(ipv,1);
            ypv = w(ipv,2);
        end
    else
        xpv = xd+lpv*cos(yaw_w);
        ypv = yd+lpv*sin(yaw_w);
    end
    
    error_lateral = sqrt((xd-z(1))^2 + (yd-z(2))^2)*sign(yd-z(2));
    if abs(error_lateral)>15
        break;
    end
    yaw_previous = yaw_rate;
    yaw_rate = atan2((ypv-z(2)), (xpv-z(1))); 
    
    yaw = z(5);
    if yaw>=0
        if yaw_rate>(yaw - pi)
            error_yaw = yaw_rate - yaw;
        else
            error_yaw = yaw_rate + 2*pi - yaw;
        end
    else
        if yaw_rate>(yaw+pi)
            error_yaw = (yaw_rate - 2*pi) - yaw;
        else
            error_yaw = yaw_rate - yaw; 
        end
    end
    
    if yaw_previous>=0
        if yaw_rate<(yaw_previous - pi)
            error_lateral_desired = yaw_rate + 2*pi - yaw_previous;
        else
            error_lateral_desired = yaw_rate - yaw_previous;
        end
    else
        if yaw_rate>(yaw_previous + pi)
            error_lateral_desired = (yaw_rate - 2*pi) - yaw_previous;
        else
            error_lateral_desired = yaw_rate - yaw_previous;
        end
    end
    
    yd_dot = error_lateral_desired/ts;
    error_yaw_rate = -z(6);
    error_v = vd - z(3);
    error_v_integral = error_v_integral + error_v*ts;  
    log_error(j,:) = [t, vd, z(3), error_v, yaw_rate, z(5), error_yaw, error_lateral];
    
    % Lateral Dynamics
    
    if z(4)==0
        alpha = zeros(1,6);
    else
        alpha(1) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) + z(6)*lf*cos(z(5)) - z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) - z(6)*lf*sin(z(5)) - z(6)*lw*cos(z(5))/2));
        alpha(3) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) + z(6)*lm*cos(z(5)) - z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) - z(6)*lm*sin(z(5)) - z(6)*lw*cos(z(5))/2));
        alpha(5) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) - z(6)*lr*cos(z(5)) - z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) + z(6)*lr*sin(z(5)) - z(6)*lw*cos(z(5))/2));
        
        alpha(2) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) + z(6)*lf*cos(z(5)) + z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) - z(6)*lf*sin(z(5)) + z(6)*lw*cos(z(5))/2));
        alpha(4) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) + z(6)*lm*cos(z(5)) + z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) - z(6)*lm*sin(z(5)) + z(6)*lw*cos(z(5))/2));
        alpha(6) = -z(5) + atan2((z(3)*sin(z(5)) + z(4)*cos(z(5)) - z(6)*lr*cos(z(5)) + z(6)*lw*sin(z(5))/2), (z(3)*cos(z(5)) - z(4)*sin(z(5)) + z(6)*lr*sin(z(5)) + z(6)*lw*cos(z(5))/2));
        
    end
    
  
    for i1=1:6
        if alpha(i1) > pi
            alpha(i1) = alpha(i1) - 2*pi;
        elseif alpha(i1) < -pi
            alpha(i1) = alpha(i1) + 2*pi;
        end
    end
    
    for  i1=1:6
        if abs(alpha(i1))>0.09
            Fy(i1) = -Cs*0.09*sign(alpha(i1));
        else
            Fy(i1) = -Cs*alpha(i1);
        end
    end
    
    % Total Lateral force
    
    Fy_total = sum(Fy);
    Md = lf*(Fy(1)+Fy(2)) - lr*(Fy(5)+Fy(6));
    Mc = (Kphi_d*error_yaw_rate + Kphi_p*error_yaw)*Iz - Md; 
    
    Fxd = m*(Kvp*(error_v)+Kvi*(error_v_integral));
    log_Fxd(j,:) = [t, Fxd];
    
    % Longitudinal force in each wheel - Tire force distribution algorithm
    
    A=[2*(1/(Fz1^2 + 1)+1/(Fz5^2 + 1)),0,2*1/(Fz5^2 + 1),0;
        0,2*(1/(Fz2^2 + 1)+1/(Fz6^2 + 1)),0,2/(Fz6^2 + 1);
        2/(Fz5^2 + 1),0,2*(1/(Fz3^2 + 1)+1/(Fz5^2 + 1)),0;
        0,2/(Fz6^2 + 1),0,2*(1/(Fz4^2 + 1)+1/(Fz6^2 + 1))];
    B=[1/(Fz5^2 + 1)*Fxd-2/(Fz5^2 + 1)*Mc/lw;
        1/(Fz6^2 + 1)*Fxd+2/(Fz6^2 + 1)*Mc/lw;
        1/(Fz5^2 + 1)*Fxd-2/(Fz5^2 + 1)*Mc/lw;
        1/(Fz6^2 + 1)*Fxd+2/(Fz6^2 + 1)*Mc/lw];
    Fx_flag = inv(A)*B;
    Fx_des(1) = Fx_flag(1);
    Fx_des(2) = Fx_flag(2);
    Fx_des(3) = Fx_flag(3);
    Fx_des(4) = Fx_flag(4);
    Fx_des(5) = (Fxd/2)-(Mc/lw)-Fx_des(1)-Fx_des(3);
    Fx_des(6) = (Fxd/2)+(Mc/lw)-Fx_des(2)-Fx_des(4);
    
    % Limiting the longitudinal force
    
    if abs(Fx_des(1))>fl || abs(Fx_des(2))>fl || abs(Fx_des(3))>fl || abs(Fx_des(4))>fl || abs(Fx_des(5))>fl || abs(Fx_des(6))>fl
        ratio = fl/max(abs(Fx_des));
        for k=1:6
            Fx_des(k) = Fx_des(k)*ratio;
        end
    end
    
    Mc_desired = (lw/2)*(Fx_des(2)+Fx_des(4)+Fx_des(6)-Fx_des(1)-Fx_des(3)-Fx_des(5));
    
    % Disturbance moment Md observer
    
    rhat = rhat+(ts/Iz)*Mdhat+(ts/Iz)*Mc+ts*p*(z(6)-rhat);
    Mdhat = Mdhat+ts*eta*(z(6)-rhat);

    log_Md(j,:) = [t, Md, Mdhat, z(6), rhat];
    log_Mc(j,:) = [t, Mc, Mc_desired];
    
    log_torque(j,:) = [t, Fx_des(1)*tr, Fx_des(2)*tr, Fx_des(3)*tr, Fx_des(4)*tr, Fx_des(5)*tr, Fx_des(6)*tr];
    log_slip_angle(j,:) = [t, alpha.*180/pi];
    
    [tnew, zode] = ode45(@statespace, [0,ts], z);
    [q,~] = size(tnew);
    z = zode(q,:);
    
    j=j+1;
    log_z(j,:) = [t, z];
    
end

end