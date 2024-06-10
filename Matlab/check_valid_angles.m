% Performs validation check for all four angles, ensuring they are inside
% the valid ranges specified by [lb, ub] bounds
function [valid_angles, angles_info] = check_valid_angles(angles)
    % Define lower and upper bounds for joint angles (in radians)
    lb = [-deg2rad(60), -deg2rad(55), -deg2rad(27.5), -deg2rad(31)]; % Lower bounds
    ub = [deg2rad(60), deg2rad(90), deg2rad(135), deg2rad(97.5)]; % Upper bounds

    % Initialize angles_info with default values
    angles_info = struct('value', num2cell(angles), 'lb', num2cell(lb), 'ub', num2cell(ub));

    % Initialize valid_angles with the original angles
    valid_angles = angles;

    for i = 1:length(angles)
        if ~isreal(angles(i)) || angles(i) < lb(i) || angles(i) > ub(i)
            disp(['Invalid value detected in theta_', num2str(i), ': ', num2str(angles(i)), ...
                  ' (deg: ', num2str(rad2deg(angles(i))), ')']);
            disp(['Valid range for theta_', num2str(i), ': [', num2str(rad2deg(lb(i))), ', ', num2str(rad2deg(ub(i))), ']']);
            valid_angles(i) = NaN;  % Set invalid angles to NaN
        end
    end
end