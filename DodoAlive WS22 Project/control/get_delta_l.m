function delta_l = get_delta_l(q,dq)
    %GET_DELTA_L_TORQUE Summary of this function goes here
    load('monopod_parameters', 'K_l', 'm_B', 'm_LL', 'm_UL');

    robot_mass = m_B + m_LL + m_UL;

    J_cog = get_jac_com(q');

    N_s = get_N_s([q';dq']); % 8 x 1

    matrix = transpose(J_cog)*J_cog - transpose(N_s)*transpose(J_cog)*J_cog*N_s;
    delta_E = 0.5 * robot_mass * dq * matrix * dq';

    delta_l = sqrt(abs((2 / K_l) * delta_E));

end

