% Returns as symbolic expression the complete transformation matrix for a
% 4DOF robotic finger, for a predetermined modified DH parameters table and
% a given general transformation matrix between two consecutive links.
function T_simplify = get_forward_transformation_matrix()
    theta_MCP_aa = sym('theta_MCP_aa');
    theta_MCP_fe = sym('theta_MCP_fe');
    theta_PIP = sym('theta_PIP');
    theta_DIP = sym('theta_DIP');
    L1 = sym('L1');
    L2 = sym('L2');
    L3 = sym('L3');
    alpha = sym('alpha');
    r = sym('r');
    theta = sym('theta');
    d = sym('d');
    
    % set base reference frame x0, y0, z0 same as first link's/joint's MCP_fe
    % (same as PIP and DIP)
    mdh_table = [0, 0, theta_MCP_aa, 0; pi/2, 0, theta_MCP_fe, 0; 0, L1, theta_PIP, 0; 0, L2, theta_DIP, 0];

    T_cur_prev = [cos(theta), -sin(theta), 0, r; ...
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha); ...
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha); ...
        0, 0, 0, 1];
    T_1_0 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(1, :));
    T_2_1 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(2, :));
    T_3_2 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(3, :));
    T_4_3 = subs(T_cur_prev, [alpha, r, theta, d], mdh_table(4, :));
    T_eff_4 = [1, 0, 0, L3; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T_total = T_1_0 * T_2_1 * T_3_2 * T_4_3 * T_eff_4; % T_total -> T_eff_0
    T_simplify = simplify(T_total, 'Steps', 150);
end