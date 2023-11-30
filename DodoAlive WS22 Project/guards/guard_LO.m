function [value, isterminal, direction] = guard_LO(t, x)
    % Lift-off occurs when constraint forces in y-direction equal zero
    global slip_length;
    dy = x(6);
    y = x(2);
    if dy >= 0
        foot_pos = get_pos_com(x(1:4)) - get_foot_pos(x(1:4));
        foot_distance = norm(foot_pos, 2);
        value = slip_length - foot_distance;
    else
        value = 1;
    end
    isterminal = 1;
    direction  = -1;
end
