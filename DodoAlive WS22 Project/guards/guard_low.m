function [value, isterminal, direction] = guard_low(t, x)
    % Low point is reached when y velocity changes from negative to positive
    dy = x(6);
    value      = dy;
    isterminal = 1;
    direction  = 1;
end
