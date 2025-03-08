function x_bounded = bound(x, xmin, xmax)
    % Ensure xmin is less than xmax
    if xmin > xmax
        error('Lower bound must be less than upper bound');
    end
    % Apply bounding
    x_bounded = min(max(x, xmin), xmax);
end
