function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%DEFAULT VALUES
kp_phi = 150;
kd_phi = 13;
kp_theta = 150;
kd_theta = 13;
kp_psi = 1;
kd_psi = 1;
% Stiff hover controller
kp_x = 1;
kp_y = 1;
kp_z = 1;
kd_x = 5; 
kd_y = 5; 
kd_z = 1; 

kp_y =   5.; %1.241; %1.8;
kp_z =   65;
kp_phi = 150; % 80 % best value so far: 26
 % asdsa
kv_y =   7.;  %3.0; %10000;
kv_z =   520; %10000;
kv_phi = 13.2; % 25 %15 % best value so far 9

x = 1;
y = 2;
z = 3;

kp_lin = [kp_x kp_y kp_z]';
kd_lin = [kd_x kd_y kd_z]';
kp_ang = [kp_phi kp_theta kp_psi]';
kd_ang = [kd_phi kd_theta kd_psi]';

phi = state.rot;
phi_dot = state.omega;

pos = state.pos;
vel = state.vel;
rot = state.rot;
omega = state.omega;


traj_pos =   des_state.pos;
traj_vel =   des_state.vel;
traj_acc =   des_state.acc;
traj_yaw = des_state.yaw;
traj_yawdot = des_state.yawdot;

% FILL IN YOUR CODE HERE
ep = traj_pos - pos;
ev = traj_vel - vel;

params;
I = params.I;
invI = params.invI;
m = params.mass;
g = params.gravity;

% t_unit = traj_vel / norm(traj_vel)
% persistent t_unit_prev
% persistent prev_time
% if isempty(t_unit_prev)
%     t_unit_prev = [0 0 0]';
%     prev_time = 0.001;
% end
% n_unit = (t_unit - t_unit_prev) / (t - prev_time);
% b_unit = cross(t_unit, n_unit)
% prev_time = t
% t_unit_prev = t_unit;

% Calculate erros in position
% ep = (dot(traj_pos - pos, n_unit))*n_unit + (dot(traj_pos - pos, b_unit))*b_unit;
ep = traj_pos - pos;
ev = (traj_vel - vel);
%Desired acceleration from erros in position
des_acc = traj_acc + kd_lin.*ev + kp_lin.*ep;
% des_acc =  kd_lin.*ev + kp_lin.*ep;

% Thrust
u1 = m * (g + des_acc(z));
F = u1;

% Moment
traj_yaw = 0;
traj_yawdot = 0;
% From desired acceleration and trajectory planned yaw, calculate desired phi (roll) and theta (pitch)
phi_des = (1/g) * (des_acc(x)* sin(traj_yaw) - des_acc(y)*cos(traj_yaw));
theta_des = (1/g) * (des_acc(x)* cos(traj_yaw) + des_acc(y)*sin(traj_yaw));
des_rot = [phi_des; theta_des; traj_yaw];
des_rot_dot = [0 0 traj_yawdot]';

M = kp_ang.*(des_rot - rot) + kd_ang.*(des_rot_dot - omega)

% M = zeros(3,1);
% =================== Your code ends here ===================

end
