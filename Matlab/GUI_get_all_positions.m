% Returns to the GUI the positions for all 3 joints (MCP, PIP, DIP) 
% and for the end effector (fingertip), using a predetermined modified DH
% parameters table and the general transformation matrix and given as
% argument the lengths of the 3 phalanges and the angles that must be
% calculated first during inverse kinematics solution. The GUI uses this
% function to visualize explicitly each phalange in 3D plot.
function [MCP_aa_fe_position, PIP_position, DIP_position, eff_position] = GUI_get_all_positions(lengths, angles)
    % Extract angles
    theta_MCP_aa = angles(1);
    theta_MCP_fe = angles(2);
    theta_PIP = angles(3);
    theta_DIP = angles(4);
    L1 = lengths(1);
    L2 = lengths(2);
    L3 = lengths(3);
    
    % Use sym to create symbolic variables
    alpha = sym('alpha');
    r = sym('r');
    theta = sym('theta');
    d = sym('d');
    
    % MDH parameters
    mdh_table = [0, 0, theta_MCP_aa, 0; pi/2, 0, theta_MCP_fe, 0; 0, L1, theta_PIP, 0; 0, L2, theta_DIP, 0];

    % Define transformation matrix components
    T_cur_prev = [cos(theta), -sin(theta), 0, r; ...
                  sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha); ...
                  sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha); ...
                  0, 0, 0, 1];

    % Calculate transformation matrices for each joint
    T_1_0 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(1, :));
    T_2_1 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(2, :));
    T_3_2 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(3, :));
    T_4_3 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(4, :));

    % Define the transformation matrix for the end-effector
    T_eff_4 = [1, 0, 0, L3; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];

    % Calculate the total transformation matrix
    T_total = T_1_0 * T_2_1 * T_3_2 * T_4_3 * T_eff_4; % unused

    % Extract the positions for all links
    Link_0__MCP_aa_fe = T_1_0 * T_2_1; % MCP_aa_fe is always at (0,0,0)
    Link_MCP_aa_fe__PIP = Link_0__MCP_aa_fe * T_3_2;
    Link_PIP__DIP = Link_MCP_aa_fe__PIP * T_4_3;
    Link_DIP__eff = Link_PIP__DIP * T_eff_4;
    MCP_aa_fe_position = double(Link_0__MCP_aa_fe(1:3, 4));
    PIP_position = double(Link_MCP_aa_fe__PIP(1:3, 4));
    DIP_position = double(Link_PIP__DIP(1:3, 4));
    eff_position = double(Link_DIP__eff(1:3, 4));

    % Check for complex values
    if ~isreal(MCP_aa_fe_position)
        disp('Complex value detected in MCP_aa_fe_position');
        disp(MCP_aa_fe_position);
    end
    if ~isreal(PIP_position)
        disp('Complex value detected in PIP_position');
        disp(PIP_position);
    end
    if ~isreal(DIP_position)
        disp('Complex value detected in DIP_position');
        disp(DIP_position);
    end
    if ~isreal(eff_position)
        disp('Complex value detected in eff_position');
        disp(eff_position);
    end
end