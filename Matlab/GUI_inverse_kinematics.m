% Solves the inverse kinematics problem for given phalanges lengths, end
% effector's (fingertip) position and orientation (x, y, z, total angle with respect to x-axis in x-z plane)
% and returns to the GUI the 4 angles solutions, along with some additional
% information about their validity check, calling internally check_valid_angles().
function [theta_MCP_aa_sol, theta_MCP_fe_sol, theta_PIP_sol, theta_DIP_sol, angles_info] = GUI_inverse_kinematics(lengths, x, y, z, angle)
    L1 = lengths(1);
    L2 = lengths(2);
    L3 = lengths(3);
    x_eff = x;
    y_eff = y;
    z_eff = z;
    theta_eff = angle;

    % Calculate the angles derived mathematically in closed form solutions
    % MCPaa
    theta_MCP_aa_sol = atan2(y_eff, x_eff);
    
    % MCPfe
    numerator1_MCP_fe = z_eff - L3*sin(theta_eff);
    denominator1_MCP_fe = sqrt(x_eff^2 + y_eff^2) - L3*cos(theta_eff);
    numerator2_MCP_fe = x_eff^2 + y_eff^2 + z_eff^2 + L3^2 + L1^2 - L2^2 - 2*L3*(sqrt(x_eff^2 + y_eff^2)*cos(theta_eff) + z_eff*sin(theta_eff));
    denominator2_MCP_fe = 2*L1*(sqrt(x_eff^2 + y_eff^2 + z_eff^2 + L3^2 - 2*L3*(sqrt(x_eff^2 + y_eff^2)*cos(theta_eff) + z_eff*sin(theta_eff))));

    % Ensure the fraction is within the range of [-1, 1]
    fraction_MCP_fe = numerator2_MCP_fe / denominator2_MCP_fe;
    fraction_MCP_fe = max(min(fraction_MCP_fe, 1), -1);

    theta_MCP_fe_sol = atan2(numerator1_MCP_fe, denominator1_MCP_fe) - acos(fraction_MCP_fe);

    % PIP
    numerator_PIP = x_eff^2 + y_eff^2 + z_eff^2 + L3^2 - 2*L3*(sqrt(x_eff^2 + y_eff^2)*cos(theta_eff) + z_eff*sin(theta_eff)) - L1^2 - L2^2;
    denominator_PIP = 2*L1*L2;

    % Ensure the fraction is within the range of [-1, 1]
    fraction_PIP = numerator_PIP / denominator_PIP;
    fraction_PIP = max(min(fraction_PIP, 1), -1);

    theta_PIP_sol = acos(fraction_PIP);

    % DIP
    theta_DIP_sol = theta_eff - theta_MCP_fe_sol - theta_PIP_sol;

    % Check for complex values and set to NaN if found
    if ~isreal(theta_MCP_fe_sol)
        disp('Complex value detected in theta_MCP_fe_sol');
        disp(['Numerator1: ', num2str(numerator1_MCP_fe)]);
        disp(['Denominator1: ', num2str(denominator1_MCP_fe)]);
        disp(['Numerator2: ', num2str(numerator2_MCP_fe)]);
        disp(['Denominator2: ', num2str(denominator2_MCP_fe)]);
        disp(['theta_MCP_fe_sol: ', num2str(theta_MCP_fe_sol)]);
        theta_MCP_fe_sol = NaN;
    end

    if ~isreal(theta_PIP_sol)
        disp('Complex value detected in theta_PIP_sol');
        disp(['Numerator: ', num2str(numerator_PIP)]);
        disp(['Denominator: ', num2str(denominator_PIP)]);
        disp(['theta_PIP_sol: ', num2str(theta_PIP_sol)]);
        theta_PIP_sol = NaN;
    end

    if ~isreal(theta_DIP_sol)
        disp('Complex value detected in theta_DIP_sol');
        disp(['theta_DIP_sol: ', num2str(theta_DIP_sol)]);
        theta_DIP_sol = NaN;
    end

    % Validate the angles
    angles = [theta_MCP_aa_sol, theta_MCP_fe_sol, theta_PIP_sol, theta_DIP_sol];
    [valid_angles, angles_info] = check_valid_angles(angles);
    
    % Assign back the validated angles
    theta_MCP_aa_sol = valid_angles(1);
    theta_MCP_fe_sol = valid_angles(2);
    theta_PIP_sol = valid_angles(3);
    theta_DIP_sol = valid_angles(4);
end

