function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

persistent phi_c;
persistent n;

if isempty(n)
    n = 0
end

kp_y =   1.8; %1.8;
kp_z =   2000;
kp_phi = 80; % 80 % best value so far: 26
 % asdsa
kv_y =   25.;  %3.0; %10000;
kv_z =   200; %10000;
kv_phi =  55; % 25 %15 % best value so far 9

y = 1;
z = 2;

phi = state.rot;
phi_dot = state.omega;

% FILL IN YOUR CODE HERE

ep = des_state.pos - state.pos;
ev = des_state.vel - state.vel;

% ev_y = ev(y)
% ep_y = ep(y)

m = params.mass;
g = params.gravity;
Ixx = params.Ixx;

% des_state.pos = des_state.pos
% des_state.vel = des_state.vel
% des_state.acc = des_state.acc
% des_state_vel_y = des_state.vel(y)

phi_c = (-1/g) * ( des_state.acc(y) +  kv_y * ev(y) + kp_y*ep(y) );
% phi_c = (-1/g) * (    kv_y * ev(y) + kp_y*ep(y) )

% if n < 2000
%     phi_c = -20 * pi / 180.0
% else
%     phi_c = 0
% end
n = n + 1;

u1 = m * (g + des_state.acc(z) + kv_z*ev(z) + kp_z*ep(z));
u2 = Ixx * ( 0 + kv_phi*(-phi_dot) + kp_phi*(phi_c - phi));

end

