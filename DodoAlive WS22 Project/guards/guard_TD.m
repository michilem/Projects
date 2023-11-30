function [value, isterminal, direction] = guard_TD(t, x)
    % Touchdown occurs when the foot y-coordinate equals zero
    if x(6) <= 0
        value = get_c(x(1:4));
    else
        value = 1;
    end
    isterminal = 1;
    direction  = -1;
end
