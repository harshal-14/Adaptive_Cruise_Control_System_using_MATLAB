function yaw_angle = traj_controller(current_point,target_point,v_x)
if target_point(1,1)<=current_point(1,1)
    yaw_angle = 0;
    return;
end

%% Waypoint generation
dt = 0.1;
T = abs(target_point(1,1)-current_point(1,1))/v_x;

x_f = target_point(1,1); % Final x coordinate
y_f = target_point(1,2); % Final y coordinate

%% Vehicle next state in x direction (Longitudinal)

x=current_point(1,1) + dt*v_x;
x_dot=v_x;
x_ddot=0;

nextState_x=[x;x_dot;x_ddot];

%%  Minimun jerk trajectory function for the calculation in y direction (Lateral)

a0=current_point(1,2);
a1=0;
a2=0;
syms a3 a4 a5;
[a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
    3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
    6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration

% Solving for coefficients and conversion to double precision
a3=double(a3);
a4=double(a4);
a5=double(a5);

%% Vehicle states in y direction
y=[];
y_dot=[];
y_ddot=[];

t = dt;
y       = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
y_dot   = a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4;
y_ddot  = 2*a2+6*a3*t+12*a4*t^2+20*a5*t^3;

nextState_y=[y;y_dot;y_ddot];

%% Yaw angle reference
yaw_angle=atan(y_dot/ x_dot);

end