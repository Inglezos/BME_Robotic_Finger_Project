% Calculates and returns the jacobian matrix for the positions and angles
% of a 4DOF robotic finger's fingertip (end effector), which are 
% the linear and angular velocities, depending on the passed argument
% 'option' ("linear" or "angular"). The positions of the fingertip are
% extracted from the total transformation matrix denotes as Q, which is
% passed as argument.
function J = get_jacobian(Q, option)
    % Define the symbolic variables for the joint angles and lengths
    syms theta_MCP_aa theta_MCP_fe theta_PIP theta_DIP real
    syms L1 L2 L3 positive
    
    % Position of the fingertip
    x = Q(1, 4);
    y = Q(2, 4);
    z = Q(3, 4);
    
    % Define the joint angles vector
    theta = [theta_MCP_aa, theta_MCP_fe, theta_PIP, theta_DIP];
    
    % Compute the linear velocity Jacobian matrix
    J_v = jacobian([x; y; z], theta);
    
    % The rotation axes for each joint relative to the base frame
    z0 = [0; 0; 1];  % z axis for theta_MCP_aa
    z1 = [0; -1; 0];  % -y axis for theta_MCP_fe
    z2 = [0; -1; 0];  % -y axis for theta_PIP
    z3 = [0; -1; 0];  % -y axis for theta_DIP
    
    % Combine to form the angular velocity Jacobian
    J_omega = [z0, z1, z2, z3];
    
    if option == "linear"
        J = J_v;
    elseif option == "angular"
        J = J_omega;
    else
        % Full Jacobian matrix
        J = [J_v; J_omega];
    end
end

