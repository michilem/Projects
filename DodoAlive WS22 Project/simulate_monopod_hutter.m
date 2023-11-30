%% Simulate monopod dynamics
% Init
clear; clc; close all;

% Toggle simulation behavior
export_graphs = false;

% flight controller parameters
global i_pos;
global i_vel;
global i_foot;
global iteration_counter;
global control_timer;
global prev_iteration_t;
global angle_of_attack;

i_pos = 0;
i_vel = 0;
i_foot = [0;0];
iteration_counter = 0;
control_timer = 0;
prev_iteration_t = 0;
angle_of_attack = 0;

global slip_length;

% State vector: x = [x; y, q1; q2; dx; dy; dq1; dq2]
% Control vector: u = [u1; u2]

x_0 = [0, 1.5, deg2rad(-45), deg2rad(-90), 1.0, 0, 0, 0];
x_full = x_0;
t_full = 0;

%% Tunable parameters
% Set desired hopping heights (foot) and corresponding times
t_max = 30; % time duration
t_d = [10 25 45];
d_counter = 1;
poses_des = [10, 25, -5]; % desired base position
pos_des = poses_des(1);
l_desired = 0.9; % desired flight phase length

%%

t_step = 0.025;
t_span = 0:t_step:t_max;

l_rest = get_l_rest();
slip_length = l_rest;

% Event functions
tolerance = odeset('RelTol',1e-3,'AbsTol',1e-4);
opts_flight1 = odeset(odeset('Events', @guard_high), tolerance);
opts_flight2 = odeset(odeset('Events', @guard_TD), tolerance);
opts_stance1 = odeset(odeset('Events', @guard_low), tolerance);

while true
    % Flight phase 2: From high-point to touchdown
    [t_out_f2, x_out_f2, te_f2, xe_f2, ie_f2] = ...
        ode45(@(t, x) get_dyn_flight(t, x, ...
        flight_controller(t, x, pos_des, l_desired)), ...
        t_span, x_full(end, :), opts_flight2);

    x_full = [x_full; x_out_f2(2:end, :)];  % remove first entry of x_out since it is already in last entry of x_full
    t_full = [t_full; t_out_f2(2:end)];

    t_span = t_full(end):t_step:t_max;

    if ie_f2
        fprintf("t = %.2fs: Touchdown", t_full(end))
    end

    if t_full(end) >= t_max - t_step
        break
    end
    
    %_calculate_new_slip_length
    q = x_full(end, 1:4);
    dq = x_full(end, 5:8);

    base_to_foot_pos =  get_pos_com(q') - get_foot_pos(q');
    dl =  get_delta_l(q, dq);
    slip_length = norm(base_to_foot_pos, 2) + dl;

    % flight to stance jump function:  calculate new dq after impact
    % Impact
    x_full(end, 5:8) = get_impact(xe_f2')';
    new_dq = x_full(end, 5:8);

    fprintf(" & Impact\n")
    if t_full(end) >= t_max - t_step
        break
    end
    
    % Stance phase 1: From touchdown to low-point
    [t_out_s1, x_out_s1, te_s1, xe_s1, ie_s1] = ...
        ode45(@(t, x) get_dyn_stance(t, x, ...
        stance_controller(t, x)), ...
        t_span, x_full(end, :), opts_stance1);

    x_full = [x_full; x_out_s1(2:end, :)];
    t_full = [t_full; t_out_s1(2:end)];

    t_span = t_full(end):t_step:t_max;

    if ie_s1
        fprintf("t = %.2fs: Low-point reached\n", t_full(end))
    end

    if t_full(end) >= t_max - t_step
        break
    end
    
    %Set desired hopping height corresponding to current time
    if t_full(end) >= t_d(d_counter)
        d_counter = min(d_counter + 1, length(t_d));
        pos_des = poses_des(d_counter);
    end
    
    % Stance phase 2: From low-point to lift-off
    opts_stance2 = odeset('Events', @(t,x) guard_LO(t, x));

    [t_out_s2, x_out_s2, te_s2, xe_s2, ie_s2] = ...
        ode45(@(t, x) get_dyn_stance(t, x, ...
        stance_controller(t, x)), ...
        t_span, x_full(end, :), opts_stance2);

    x_full = [x_full; x_out_s2(2:end, :)];
    t_full = [t_full; t_out_s2(2:end)];

    t_span = t_full(end):t_step:t_max;

    if ie_s2
        fprintf("t = %.2fs: Lift-Off\n", t_full(end))
    end

    if t_full(end) >= t_max - t_step
        break
    end
    
    % Initialize control timer of fligh phase controller.
    control_timer = 0;

    % Flight phase 1: From lift-off to high-point
    [t_out_f1, x_out_f1, te_f1, xe_f1, ie_f1] = ...
        ode45(@(t, x) get_dyn_flight(t, x, ...
        flight_controller(t, x, pos_des, l_desired)), ...
        t_span, x_full(end, :), opts_flight1);
    
    x_high_prev = xe_f1;
    
    x_full = [x_full; x_out_f1(2:end, :)];
    t_full = [t_full; t_out_f1(2:end)];

    t_span = t_full(end):t_step:t_max;

    if ie_f1
        fprintf("t = %.2fs: High-point reached\n", t_full(end))
    end

    if t_full(end) >= t_max - t_step
        break
    end
    get_E_sys([q';dq'])
end

% Plot trajectory
monoplot(t_full, x_full, t_d, poses_des, export_graphs)

