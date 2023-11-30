function u = flight_controller(t, x, des_pos, l0)
    load('monopod_parameters.mat', 'K_l', 'K_phi', 'D_l', 'D_phi', 'l_UL', 'l_LL', 'enable_trq_limit', 'trq_max', 'm_B', 'm_LL', 'm_UL')
    
    function v = limit_value_to_max_abs(value, max_abs)
        v = min(max_abs, max(-max_abs, value));
    end

    % global parameters to store temporal relate variables
    global iteration_counter;
    global prev_iteration_t;
    global i_pos;
    global i_vel;
    global i_foot;
    global angle_of_attack;
    global control_timer;
    
    %% Tunable parameters
    max_vel = 1.5;
    max_angle_of_attack = 18;
    
    % control period of aoa controller
    angle_of_attack_control_period = 0.25;
    
    % integral term scale
    i_scale = 2e1;
    
    % velocity pid controller
    pos_k_p = 0.50;
    pos_k_i = 0.08;
    i_max_control_pos = 0.8;
    
    % angle of attacl pid controller
    vel_k_p = 10.;
    vel_k_i = 0.1;
    vel_k_c = 0.27; % com velocity damping term
    i_max_control_vel = 0.4;
    
    % foot position pid controller
    foot_k_p = 10;
    foot_k_i = 0.1;
    foot_k_d = 1.5;
    i_max_control_foot = 0.2;
    

    %% controller

    % State variables
    q = x(1:4);
    dq = x(5:8);
    
    iteration_counter = iteration_counter + 1;
    time_diff = t - prev_iteration_t;

    %pos_com = get_pos_com(q);

    %% position_controller
    %W_X_B = get_W_X_B(q);
    %cur_pos = W_X_B * [pos_com(1);pos_com(2);0;1]; % world com position

    pos_error = des_pos - q(1);

    i_pos = i_pos + pos_error * time_diff * i_scale; % error integration
    i_pos = limit_value_to_max_abs(i_pos, i_max_control_pos);
    
    vel_des = pos_k_p * pos_error + pos_k_i * i_pos;
    vel_des = limit_value_to_max_abs(vel_des, max_vel);

    %% velocity_controller
    %vel_com = get_jac_com(q) * dq;
    %cur_vel = vel_com(1); % com x velocity
    cur_vel = dq(1);
    vel_error = (cur_vel - vel_des);
    
    % Integral Part of PID
    i_vel = i_vel + vel_error * time_diff * i_scale;

    i_vel = limit_value_to_max_abs(i_vel, i_max_control_vel);
    
    % control desired angle of attack every (angle_of_attack_control_period) seconds to avoid change too frequently.
    control_timer = control_timer - time_diff;
    if control_timer <= 0
        control_timer = control_timer + angle_of_attack_control_period;
        % PID
        angle_of_attack = vel_k_p * vel_error + vel_k_i * i_vel + vel_k_c * cur_vel;
        angle_of_attack = limit_value_to_max_abs(angle_of_attack, max_angle_of_attack);
    end
    
    %% pose_controller
    aoa_rad = deg2rad(angle_of_attack);
    des_x = sin(aoa_rad) * l0;
    des_y = -cos(aoa_rad) * l0;
    pos_foot_des = [des_x ; des_y]; 

    pos_foot = get_foot_pos(q);

    foot_pos_error = pos_foot_des - pos_foot;
    
    i_foot = i_foot + foot_pos_error * time_diff * i_scale;

    i_foot(1) = limit_value_to_max_abs(i_foot(1), i_max_control_foot);
    i_foot(2) = limit_value_to_max_abs(i_foot(2), i_max_control_foot);
    
    % local space jacobian of the foot
    J_cq = get_J_cq(q);
    foot_vel = J_cq * dq;

    control_force = foot_k_p * foot_pos_error + foot_k_i * i_foot - foot_k_d * (foot_vel);
    
    f = J_cq' * control_force;
    u = [0;0;f(3); f(4)];

    prev_iteration_t = t;
end
