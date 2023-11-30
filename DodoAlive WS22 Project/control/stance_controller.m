function u = stance_controller(t, x)
    load('monopod_parameters.mat', 'K_l', 'K_phi', 'D_l', 'D_phi', 'l_UL', 'l_LL', 'enable_trq_limit', 'trq_max', 'm_B', 'm_LL', 'm_UL')
    
    % Some parameters
    global slip_length;
    global prev_iteration_t;

    prev_iteration_t = t;

    q = x(1:4);
    dq = x(5:8);
    
    robot_mass = m_B + m_UL + m_LL;
    
    % Calculate constrained com jacobian numerically
    all_stars = get_all_stars_num(q, dq);
    J_star = all_stars{1};
    lambda_star = all_stars{2};
    mu_star = all_stars{3};
    p_star = all_stars{4};
    
    % slip force calculation
    distance_foot_com = get_pos_com(q) - get_foot_pos(q);
    foot_new_length = norm(distance_foot_com, 2);
    
    if foot_new_length < 1e-8
        vec = [0; 0];
    else
        vec = distance_foot_com / foot_new_length;
    end

    spring_compression = slip_length - foot_new_length;
    spring_compression_vector = spring_compression * vec;
    
    % Transform desired slip force to joint actuations : tau.
    slip_force = K_l * spring_compression_vector + robot_mass * [0; -9.81];
    tau = transpose(J_star) * (lambda_star * ((1/robot_mass) * slip_force) + (mu_star + p_star));
    
    u = [0;0;tau(1);tau(2)];
end
