function [state_x,state_y,yaw_angle]=traj_planner_v2(current_point,T,v_x,lane_width)
%% Inputs:

% current_point : a 1x2 array that represents vehicle's location at the
%beginning of the lane-switching trajectory

% T : total duration of the lane change trajectory

% v_x : initial longitudinal speed at the beginning of the lane-switching

% lane_width : is the distance between adjacent lane centerline


%% Waypoint generation
% frequency = 10;%Number of commands per sec (obsolete)
dt = 0.1;

x_f = current_point(1,1) + T*v_x; % Final x coordinate
y_f = current_point(1,2) + lane_width; % Final y coordinate

%% Vehicle states in x direction (Longitudinal)
x=[];
x_dot=[];
x_ddot=[];

for t=0:dt:T
x=[x, current_point(1,1) + t*v_x];
x_dot=[x_dot,v_x];
x_ddot=[x_ddot,0];
end
state_x=[x;x_dot;x_ddot];

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
for t=0:dt:T
y=[y,a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5];
y_dot=[y_dot,a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4];
y_ddot=[y_ddot,2*a2+6*a3*t+12*a4*t^2+20*a5*t^3];
end
state_y=[y;y_dot;y_ddot];

%% Calculate yaw angle kinematically
yaw_angle=zeros(length(x_dot),1); % Memory preallocation

for t=1:length(x_dot)
    yaw_angle(t)=atan(y_dot(t)/ (x_dot(t)));
end
%% Visualization of the Minimum Jerk Optimal Trajectory
%1. Position
figure
t=0:dt:T;
subplot(4,1,1);
plot(state_x(1,:),state_y(1,:));
legend('pos');
title("Minimum jerk trajectory");
xlabel("x direction[m]");
ylabel("y direction[m]");

%2. Velocity
subplot(4,1,2);
plot(t,state_x(2,:))
hold on;
plot(t, state_y(2,:));
legend('v_x','v_y');
title("Velocity");
xlabel("time[s]");
ylabel("Velocity[m/s]");

%3. Acceleration
subplot(4,1,3);
plot(t, state_x(3,:));
hold on;
plot(t, state_y(3,:));
legend('a_x','a_y');
title("Acceleration");
xlabel("time[s]");
ylabel("acceleration[m/s^2]");

%4. Yaw angle
subplot(4,1,4);
plot(t, yaw_angle);
hold on;
plot(t, yaw_angle);
legend('yaw angle');
title("yaw angle");
xlabel("time[s]");
ylabel("yaw angle[rad]");
end
