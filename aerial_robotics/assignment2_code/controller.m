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

kp_y = 1000;
kp_z = 1000;
kp_phi = 1000;

kv_y = 10000;
kv_z = 10000;
kv_phi = 10000;

y = 1;
z = 2;

phi = state.rot;
phi_dot = state.omega;

% FILL IN YOUR CODE HERE

params

ep = des_state.pos - state.pos;
ev = des_state.vel - state.vel;

m = params.mass;
g = params.gravity;
Ixx = params.Ixx;



u1 = m * (g + des_state.acc(z) + kv_z*ev(z) + kp_z*ep(z));
phi_c = (1/g) * ( des_state.acc(y) +  kv_y *ev(y) + kp_y*ep(y) );
u2 = Ixx * ( 0 + kv_phi*(-phi_dot) + kp_phi*(phi_c - phi));

end

