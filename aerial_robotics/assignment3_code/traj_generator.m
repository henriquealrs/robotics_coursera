function [ desired_state ] = traj_generator(t, state, waypoints_)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

%% Fill in your code here
function [M, b] = GetMatrices(x)
    n_coefs = 8;
    N = length(x);
    n_pols = N - 1;
    M0 = zeros(n_pols, n_pols * n_coefs);
    j = 1;

    for i = 1:n_coefs:n_pols * n_coefs
        i
        M0(j, i) = 1;
        j = j + 1;
    end

    M1 = zeros(n_pols, n_pols * n_coefs);

    for i = 1:n_pols
        init_col = 1 + n_coefs * (i - 1);
        end_col = init_col + n_coefs - 1;
        M1(i, init_col:end_col) = 1;
    end

    M0
    M1

    m_ders_0 = (eye(n_coefs) * (factorial(0:n_coefs - 1))') .* eye(n_coefs);
    m_ders_1 = zeros(n_coefs);
    row = ones(1, n_coefs);

    for k = 0:(n_coefs - 1)
        m_ders_1(k + 1, :) = row;
        row(k + 1:end) = row(k + 1:end) .* (0:n_coefs - k - 1)
    end

    m_ders_0
    inv(m_ders_0)
    m_ders_1
    inv(m_ders_1)

    M = zeros(n_coefs * n_pols);
    M(1:n_pols, :) = M0;
    M(n_pols + 1:2 * n_pols, :) = M1;

    filled_constraints = 2 * n_pols;
    M(filled_constraints + 1, 1:n_coefs) = m_ders_0(2, :);
    filled_constraints = filled_constraints + 1;
    M(filled_constraints + 1, end - n_coefs + 1:end) = m_ders_1(end - (num_coefs - 2), :);
    filled_constraints = filled_constraints + 1;

    for n_der = 2:3

        for i = 1:n_pols
            M(filled_constraints + 1, 1 + (n_coefs * (i - 1)):n_coefs * i) = m_ders_0(n_der+1, :);
            filled_constraints = filled_constraints + 1

            M(filled_constraints + 1, 1 + (n_coefs * (i - 1)):n_coefs * i) = m_ders_1(n_der+1, :);
            filled_constraints = filled_constraints + 1
        end
    end
    n_der = 4;
    for i = 1:n_pols
        M(filled_constraints + 1, 1 + (n_coefs * (i - 1)):n_coefs * i) = m_ders_0(n_der+1, :);
        filled_constraints = filled_constraints + 1
    end
    for i = 1 : n_pols/2
        M(filled_constraints + 1, 1 + (n_coefs * (i - 1)):n_coefs * i) = m_ders_1(n_der+1, :);
        filled_constraints = filled_constraints + 1
    end

    b0 = x(1:end -1);
    b1 = x(2:end);
    % M = [M0; M1;]
    b = [b0; b1; zeros(n_coefs * n_pols - 2 * n_pols, 1)];
    M
    det_M = det(M)

end

persistent waypoints traj_time d0 num_points num_coefs T curr_p;
persistent T_final dt;
if nargin > 2
    % T_final = 15;
    % d = waypoints(:, 2:end) - waypoints(:, 1:end - 1);
    % d0 = 2 * sqrt(d(1, :) .^ 2 + d(2, :) .^ 2 + d(3, :) .^ 2);
    % traj_time = [0, cumsum(d0)];
    waypoints = waypoints_;
    num_points = size(waypoints, 2);
    num_coefs = 8;
    T = 0;
    curr_p = 1;
    X = waypoints(1, :)'; 
    Y = waypoints(2, :)'; 
    Z = waypoints(3, :)';
    dt =  T_final / num_points; 
    [M b] = GetMatrices(X)
    size(M)
    inv(M)
    alpha = inv(M) * b
else
    i = floor(t/dt);
    persistent x_f v_f acc_f
    if i == 0
        x_0 = 0;
        x_f = 0;
        v_0 = 0;
        v_f = 0;
        acc_0 = 0;
    else
        x_0 = x_f;
    end
end
%



desired_state.pos = zeros(3,1);
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
desired_state.yaw = 0;

end


