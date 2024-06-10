classdef Robotic_Finger_GUI_App < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        SetLengthsButton          matlab.ui.control.Button
        L3EditField               matlab.ui.control.NumericEditField
        L3EditFieldLabel          matlab.ui.control.Label
        L2EditField               matlab.ui.control.NumericEditField
        L2EditFieldLabel          matlab.ui.control.Label
        L1EditField               matlab.ui.control.NumericEditField
        L1EditFieldLabel          matlab.ui.control.Label
        ThetaValuesTextArea       matlab.ui.control.TextArea
        ThetaValuesTextAreaLabel  matlab.ui.control.Label
        ViewDropDown              matlab.ui.control.DropDown
        ViewLabel                 matlab.ui.control.Label
        AngleEditField            matlab.ui.control.NumericEditField
        AngleEditFieldLabel       matlab.ui.control.Label
        SliderAngle               matlab.ui.control.Slider
        SliderAngleLabel          matlab.ui.control.Label
        ZEditField                matlab.ui.control.NumericEditField
        ZEditFieldLabel           matlab.ui.control.Label
        YEditField                matlab.ui.control.NumericEditField
        YEditFieldLabel           matlab.ui.control.Label
        XEditField                matlab.ui.control.NumericEditField
        XEditFieldLabel           matlab.ui.control.Label
        CalculateButton           matlab.ui.control.Button
        SliderZ                   matlab.ui.control.Slider
        SliderZLabel              matlab.ui.control.Label
        SliderY                   matlab.ui.control.Slider
        SliderYLabel              matlab.ui.control.Label
        SliderX                   matlab.ui.control.Slider
        SliderXLabel              matlab.ui.control.Label
        UIAxes                    matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        FingerLength = [5, 4, 3]; % Lengths of the finger segments (phalanges) (by default value)
        InitialJointAngles = [0, 0, 0, 0]; % Initial joint angles for a fully extended finger
        LastValidState % Store last valid state values and joint angles
        RepresentationType = 'Phalanges'; % Default representation type
    end
    
    methods (Access = private)    
        % Function to draw the robotic finger
        function drawFinger(app, jointAngles)
            if strcmp(app.RepresentationType, 'Phalanges')
                drawComplexFinger(app, jointAngles);
            end
        end

        % Function to draw the robotic finger with 3 links/phalanges
        function drawComplexFinger(app, jointAngles)
            base = [0, 0, 0]; % Base position at the origin
            lengths = app.FingerLength;

            % Get the positions of each joint
            [joint1, joint2, joint3, fingertip] = GUI_get_all_positions(lengths, jointAngles);
        
            % Plot each segment
            cla(app.UIAxes);
            hold(app.UIAxes, 'on');
            plot3(app.UIAxes, [base(1), joint1(1)], [base(2), joint1(2)], [base(3), joint1(3)], 'b', 'LineWidth', 2);
            plot3(app.UIAxes, [joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'b', 'LineWidth', 2);
            plot3(app.UIAxes, [joint2(1), joint3(1)], [joint2(2), joint3(2)], [joint2(3), joint3(3)], 'b', 'LineWidth', 2);
            plot3(app.UIAxes, [joint3(1), fingertip(1)], [joint3(2), fingertip(2)], [joint3(3), fingertip(3)], 'b', 'LineWidth', 2);
            
            % Plot joints with different colors
            plot3(app.UIAxes, joint1(1), joint1(2), joint1(3), 'ro', 'MarkerFaceColor', 'r'); % Red dot for joint 1
            plot3(app.UIAxes, joint2(1), joint2(2), joint2(3), 'bo', 'MarkerFaceColor', 'b'); % Blue dot for joint 2
            plot3(app.UIAxes, joint3(1), joint3(2), joint3(3), 'co', 'MarkerFaceColor', 'c'); % Cyan dot for joint 3
            plot3(app.UIAxes, fingertip(1), fingertip(2), fingertip(3), 'go', 'MarkerFaceColor', 'g'); % Green dot for fingertip
        
            hold(app.UIAxes, 'off');
        
            % Set axis properties for better visualization
            grid(app.UIAxes, 'on');
            axis(app.UIAxes, 'equal');
            xlim(app.UIAxes, [-20, 20]);
            ylim(app.UIAxes, [-20, 20]);
            zlim(app.UIAxes, [-20, 20]);
            xlabel(app.UIAxes, 'X');
            ylabel(app.UIAxes, 'Y');
            zlabel(app.UIAxes, 'Z');

            % Set specific tick marks for X-axis to avoid cluttering
            app.UIAxes.XTick = -20:5:20;
            app.UIAxes.YTick = -20:5:20;
            app.UIAxes.ZTick = -20:5:20;
        end

        % Function to update the finger position from sliders
        function updateFingerFromSliders(app)
            x = app.SliderX.Value;
            y = app.SliderY.Value;
            z = app.SliderZ.Value;
            angle = app.SliderAngle.Value;

            % Convert angle to radians
            angle = deg2rad(angle);

            % Constrain the input to be within the reachable space
            max_length = sum(app.FingerLength);
            if sqrt(x^2 + y^2 + z^2) > max_length
                % Reset sliders to previous values
                app.SliderX.Value = app.LastValidState.x;
                app.SliderY.Value = app.LastValidState.y;
                app.SliderZ.Value = app.LastValidState.z;
                app.SliderAngle.Value = rad2deg(app.LastValidState.angle);
                app.XEditField.Value = app.LastValidState.x;
                app.YEditField.Value = app.LastValidState.y;
                app.ZEditField.Value = app.LastValidState.z;
                app.AngleEditField.Value = rad2deg(app.LastValidState.angle);

                % Show warning popup
                uialert(app.UIFigure, 'The specified position is beyond the reachable space of the finger.', 'Input Error', 'Icon', 'warning');
                return;
            end

            app.XEditField.Value = x;
            app.YEditField.Value = y;
            app.ZEditField.Value = z;
            app.AngleEditField.Value = angle;

            lengths = app.FingerLength;
            [theta1, theta2, theta3, theta4, angles_info] = GUI_inverse_kinematics(lengths, x, y, z, angle);

            % Check for invalid angles and alert the user
            invalid_angles = isnan([theta1, theta2, theta3, theta4]);
            if any(invalid_angles)
                alert_msg = 'Invalid solution for the following angles: ';
                if invalid_angles(1)
                    alert_msg = sprintf('%sTheta1 (MCPaa): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(1).value), rad2deg(angles_info(1).lb), rad2deg(angles_info(1).ub));
                end
                if invalid_angles(2)
                    alert_msg = sprintf('%sTheta2 (MCPfe): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(2).value), rad2deg(angles_info(2).lb), rad2deg(angles_info(2).ub));
                end
                if invalid_angles(3)
                    alert_msg = sprintf('%sTheta3 (PIP): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(3).value), rad2deg(angles_info(3).lb), rad2deg(angles_info(3).ub));
                end
                if invalid_angles(4)
                    alert_msg = sprintf('%sTheta4 (DIP): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(4).value), rad2deg(angles_info(4).lb), rad2deg(angles_info(4).ub));
                end
                uialert(app.UIFigure, alert_msg, 'Solution Error', 'Icon', 'error');

                % Revert to previous valid state
                app.SliderX.Value = app.LastValidState.x;
                app.SliderY.Value = app.LastValidState.y;
                app.SliderZ.Value = app.LastValidState.z;
                app.SliderAngle.Value = rad2deg(app.LastValidState.angle);
                app.XEditField.Value = app.LastValidState.x;
                app.YEditField.Value = app.LastValidState.y;
                app.ZEditField.Value = app.LastValidState.z;
                app.AngleEditField.Value = rad2deg(app.LastValidState.angle);
        
                % Use LastValidState to draw the finger
                drawFinger(app, app.LastValidState.jointAngles);
                return;
            end
        
            % Update last valid state
            app.LastValidState.x = x;
            app.LastValidState.y = y;
            app.LastValidState.z = z;
            app.LastValidState.angle = angle;
            app.LastValidState.jointAngles = [theta1, theta2, theta3, theta4];

            app.ThetaValuesTextArea.Value = sprintf('Theta1 (rad): %.6f\nTheta1 (deg): %.6f\n\nTheta2 (rad): %.6f\nTheta2 (deg): %.6f\n\nTheta3 (rad): %.6f\nTheta3 (deg): %.6f\n\nTheta4 (rad): %.6f\nTheta4 (deg): %.6f', theta1, rad2deg(theta1), theta2, rad2deg(theta2), theta3, rad2deg(theta3), theta4, rad2deg(theta4));
            drawFinger(app, [theta1, theta2, theta3, theta4]);
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Enable 3D rotation
            rotate3d(app.UIAxes, 'on');

            % Initialize the finger in the fully extended position
            app.XEditField.Value = sum(app.FingerLength);
            app.YEditField.Value = 0;
            app.ZEditField.Value = 0;
            app.AngleEditField.Value = 0;
            app.SliderX.Value = sum(app.FingerLength);
            app.SliderY.Value = 0;
            app.SliderZ.Value = 0;
            app.SliderAngle.Value = 0;

            % Initialize the L1, L2, and L3 fields with current FingerLength values
            app.L1EditField.Value = app.FingerLength(1);
            app.L2EditField.Value = app.FingerLength(2);
            app.L3EditField.Value = app.FingerLength(3);

            % Set LastValidState to initial values
            app.LastValidState = struct('x', sum(app.FingerLength), 'y', 0, 'z', 0, 'angle', 0, ...
                                        'jointAngles', app.InitialJointAngles);

            % Draw the finger in the initial position
            drawFinger(app, app.InitialJointAngles);
        end

        % Value changed function: SliderX
        function SliderXValueChanged(app, event)
            app.XEditField.Value = app.SliderX.Value;
        end

        % Value changed function: SliderY
        function SliderYValueChanged(app, event)
            app.YEditField.Value = app.SliderY.Value;            
        end

        % Value changed function: SliderZ
        function SliderZValueChanged(app, event)
            app.ZEditField.Value = app.SliderZ.Value;            
        end

        % Value changed function: SliderAngle
        function SliderAngleValueChanged(app, event)
            app.AngleEditField.Value = app.SliderAngle.Value;            
        end

        % Button pushed function: CalculateButton
        function CalculateButtonPushed(app, event)
            % Callback for calculate button
            x = app.XEditField.Value;
            y = app.YEditField.Value;
            z = app.ZEditField.Value;
            angle = app.AngleEditField.Value;

            % Convert angle to radians
            angle = deg2rad(angle);

            % Constrain the input to be within the reachable space
            max_length = sum(app.FingerLength);
            if sqrt(x^2 + y^2 + z^2) > max_length
                % Reset sliders to previous values
                app.SliderX.Value = app.LastValidState.x;
                app.SliderY.Value = app.LastValidState.y;
                app.SliderZ.Value = app.LastValidState.z;
                app.SliderAngle.Value = rad2deg(app.LastValidState.angle);
                app.XEditField.Value = app.LastValidState.x;
                app.YEditField.Value = app.LastValidState.y;
                app.ZEditField.Value = app.LastValidState.z;
                app.AngleEditField.Value = rad2deg(app.LastValidState.angle);

                % Show warning popup
                uialert(app.UIFigure, 'The specified position is beyond the reachable space of the finger.', 'Input Error', 'Icon', 'warning');
                return;
            end

            lengths = app.FingerLength;
            [theta1, theta2, theta3, theta4, angles_info] = GUI_inverse_kinematics(lengths, x, y, z, angle);

            % Check for invalid angles and alert the user
            invalid_angles = isnan([theta1, theta2, theta3, theta4]);
            if any(invalid_angles)
                alert_msg = 'Invalid solution for the following angles: ';
                if invalid_angles(1)
                    alert_msg = sprintf('%sTheta1 (MCPaa): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(1).value), rad2deg(angles_info(1).lb), rad2deg(angles_info(1).ub));
                end
                if invalid_angles(2)
                    alert_msg = sprintf('%sTheta2 (MCPfe): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(2).value), rad2deg(angles_info(2).lb), rad2deg(angles_info(2).ub));
                end
                if invalid_angles(3)
                    alert_msg = sprintf('%sTheta3 (PIP): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(3).value), rad2deg(angles_info(3).lb), rad2deg(angles_info(3).ub));
                end
                if invalid_angles(4)
                    alert_msg = sprintf('%sTheta4 (DIP): %.6f (deg), Valid Range: [%.6f, %.6f] (deg)\n', alert_msg, rad2deg(angles_info(4).value), rad2deg(angles_info(4).lb), rad2deg(angles_info(4).ub));
                end
                uialert(app.UIFigure, alert_msg, 'Solution Error', 'Icon', 'error');
                
                % Revert to previous valid state
                app.SliderX.Value = app.LastValidState.x;
                app.SliderY.Value = app.LastValidState.y;
                app.SliderZ.Value = app.LastValidState.z;
                app.SliderAngle.Value = rad2deg(app.LastValidState.angle);
                app.XEditField.Value = app.LastValidState.x;
                app.YEditField.Value = app.LastValidState.y;
                app.ZEditField.Value = app.LastValidState.z;
                app.AngleEditField.Value = rad2deg(app.LastValidState.angle);
        
                % Use LastValidState to draw the finger
                drawFinger(app, app.LastValidState.jointAngles);
                return;
            end
        
            % Update last valid state
            app.LastValidState.x = x;
            app.LastValidState.y = y;
            app.LastValidState.z = z;
            app.LastValidState.angle = angle;
            app.LastValidState.jointAngles = [theta1, theta2, theta3, theta4];

            app.ThetaValuesTextArea.Value = sprintf('Theta1 (rad): %.6f\nTheta1 (deg): %.6f\n\nTheta2 (rad): %.6f\nTheta2 (deg): %.6f\n\nTheta3 (rad): %.6f\nTheta3 (deg): %.6f\n\nTheta4 (rad): %.6f\nTheta4 (deg): %.6f', theta1, rad2deg(theta1), theta2, rad2deg(theta2), theta3, rad2deg(theta3), theta4, rad2deg(theta4));
            drawFinger(app, [theta1, theta2, theta3, theta4]);
        end

        % Value changed function: ViewDropDown
        function ViewDropDownValueChanged(app, event)
            app.RepresentationType = app.ViewDropDown.Value;
            drawFinger(app, app.InitialJointAngles);
            
        end

        % Value changed function: XEditField
        function XEditFieldValueChanged(app, event)
            app.SliderX.Value = app.XEditField.Value;            
        end

        % Value changed function: YEditField
        function YEditFieldValueChanged(app, event)
            app.SliderY.Value = app.YEditField.Value;            
        end

        % Value changed function: ZEditField
        function ZEditFieldValueChanged(app, event)
            app.SliderZ.Value = app.ZEditField.Value;            
        end

        % Value changed function: AngleEditField
        function AngleEditFieldValueChanged(app, event)
            app.SliderAngle.Value = app.AngleEditField.Value;            
        end

        % Button pushed function: SetLengthsButton
        function SetLengthsButtonPushed(app, event)
            % Update lengths from user input
            app.FingerLength = [app.L1EditField.Value, app.L2EditField.Value, app.L3EditField.Value];
        
            % Reinitialize the finger in the fully extended position
            app.XEditField.Value = sum(app.FingerLength);
            app.YEditField.Value = 0;
            app.ZEditField.Value = 0;
            app.AngleEditField.Value = 0;
            app.SliderX.Value = sum(app.FingerLength);
            app.SliderY.Value = 0;
            app.SliderZ.Value = 0;
            app.SliderAngle.Value = 0;
            
            % Draw the finger in the initial position
            drawFinger(app, app.InitialJointAngles);
            
            % Set initial valid state
            app.LastValidState = struct('x', sum(app.FingerLength), 'y', 0, 'z', 0, 'angle', 0, ...
                                        'theta1', 0, 'theta2', 0, 'theta3', 0, 'theta4', 0);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [90 90 1312 926];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Robotic Finger GUI')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.XTick = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
            app.UIAxes.Position = [381 185 909 717];

            % Create SliderXLabel
            app.SliderXLabel = uilabel(app.UIFigure);
            app.SliderXLabel.HorizontalAlignment = 'right';
            app.SliderXLabel.Position = [90 787 44 22];
            app.SliderXLabel.Text = 'SliderX';

            % Create SliderX
            app.SliderX = uislider(app.UIFigure);
            app.SliderX.Limits = [-20 20];
            app.SliderX.ValueChangedFcn = createCallbackFcn(app, @SliderXValueChanged, true);
            app.SliderX.Position = [155 796 150 3];

            % Create SliderYLabel
            app.SliderYLabel = uilabel(app.UIFigure);
            app.SliderYLabel.HorizontalAlignment = 'right';
            app.SliderYLabel.Position = [92 672 44 22];
            app.SliderYLabel.Text = 'SliderY';

            % Create SliderY
            app.SliderY = uislider(app.UIFigure);
            app.SliderY.Limits = [-20 20];
            app.SliderY.ValueChangedFcn = createCallbackFcn(app, @SliderYValueChanged, true);
            app.SliderY.Position = [157 681 150 3];

            % Create SliderZLabel
            app.SliderZLabel = uilabel(app.UIFigure);
            app.SliderZLabel.HorizontalAlignment = 'right';
            app.SliderZLabel.Position = [94 564 43 22];
            app.SliderZLabel.Text = 'SliderZ';

            % Create SliderZ
            app.SliderZ = uislider(app.UIFigure);
            app.SliderZ.Limits = [-20 20];
            app.SliderZ.ValueChangedFcn = createCallbackFcn(app, @SliderZValueChanged, true);
            app.SliderZ.Position = [158 573 150 3];

            % Create CalculateButton
            app.CalculateButton = uibutton(app.UIFigure, 'push');
            app.CalculateButton.ButtonPushedFcn = createCallbackFcn(app, @CalculateButtonPushed, true);
            app.CalculateButton.Position = [185 276 100 23];
            app.CalculateButton.Text = 'Calculate';

            % Create XEditFieldLabel
            app.XEditFieldLabel = uilabel(app.UIFigure);
            app.XEditFieldLabel.HorizontalAlignment = 'right';
            app.XEditFieldLabel.Position = [145 721 25 22];
            app.XEditFieldLabel.Text = 'X';

            % Create XEditField
            app.XEditField = uieditfield(app.UIFigure, 'numeric');
            app.XEditField.ValueChangedFcn = createCallbackFcn(app, @XEditFieldValueChanged, true);
            app.XEditField.Position = [185 721 100 22];

            % Create YEditFieldLabel
            app.YEditFieldLabel = uilabel(app.UIFigure);
            app.YEditFieldLabel.HorizontalAlignment = 'right';
            app.YEditFieldLabel.Position = [145 608 25 22];
            app.YEditFieldLabel.Text = 'Y';

            % Create YEditField
            app.YEditField = uieditfield(app.UIFigure, 'numeric');
            app.YEditField.ValueChangedFcn = createCallbackFcn(app, @YEditFieldValueChanged, true);
            app.YEditField.Position = [185 608 100 22];

            % Create ZEditFieldLabel
            app.ZEditFieldLabel = uilabel(app.UIFigure);
            app.ZEditFieldLabel.HorizontalAlignment = 'right';
            app.ZEditFieldLabel.Position = [146 501 25 22];
            app.ZEditFieldLabel.Text = 'Z';

            % Create ZEditField
            app.ZEditField = uieditfield(app.UIFigure, 'numeric');
            app.ZEditField.ValueChangedFcn = createCallbackFcn(app, @ZEditFieldValueChanged, true);
            app.ZEditField.Position = [186 501 100 22];

            % Create SliderAngleLabel
            app.SliderAngleLabel = uilabel(app.UIFigure);
            app.SliderAngleLabel.HorizontalAlignment = 'right';
            app.SliderAngleLabel.Position = [71 457 66 22];
            app.SliderAngleLabel.Text = 'SliderAngle';

            % Create SliderAngle
            app.SliderAngle = uislider(app.UIFigure);
            app.SliderAngle.Limits = [-180 180];
            app.SliderAngle.ValueChangedFcn = createCallbackFcn(app, @SliderAngleValueChanged, true);
            app.SliderAngle.Position = [157 467 150 3];

            % Create AngleEditFieldLabel
            app.AngleEditFieldLabel = uilabel(app.UIFigure);
            app.AngleEditFieldLabel.HorizontalAlignment = 'right';
            app.AngleEditFieldLabel.Position = [135 390 36 22];
            app.AngleEditFieldLabel.Text = 'Angle';

            % Create AngleEditField
            app.AngleEditField = uieditfield(app.UIFigure, 'numeric');
            app.AngleEditField.ValueChangedFcn = createCallbackFcn(app, @AngleEditFieldValueChanged, true);
            app.AngleEditField.Position = [186 390 100 22];

            % Create ViewLabel
            app.ViewLabel = uilabel(app.UIFigure);
            app.ViewLabel.HorizontalAlignment = 'right';
            app.ViewLabel.Position = [140 318 31 22];
            app.ViewLabel.Text = 'View';

            % Create ViewDropDown
            app.ViewDropDown = uidropdown(app.UIFigure);
            app.ViewDropDown.Items = {'Phalanges'};
            app.ViewDropDown.ValueChangedFcn = createCallbackFcn(app, @ViewDropDownValueChanged, true);
            app.ViewDropDown.Position = [186 318 112 22];
            app.ViewDropDown.Value = 'Phalanges';

            % Create ThetaValuesTextAreaLabel
            app.ThetaValuesTextAreaLabel = uilabel(app.UIFigure);
            app.ThetaValuesTextAreaLabel.HorizontalAlignment = 'right';
            app.ThetaValuesTextAreaLabel.Position = [196 224 75 22];
            app.ThetaValuesTextAreaLabel.Text = 'Theta Values';

            % Create ThetaValuesTextArea
            app.ThetaValuesTextArea = uitextarea(app.UIFigure);
            app.ThetaValuesTextArea.Editable = 'off';
            app.ThetaValuesTextArea.HorizontalAlignment = 'center';
            app.ThetaValuesTextArea.Position = [129 48 209 170];

            % Create L1EditFieldLabel
            app.L1EditFieldLabel = uilabel(app.UIFigure);
            app.L1EditFieldLabel.HorizontalAlignment = 'right';
            app.L1EditFieldLabel.Position = [141 881 25 22];
            app.L1EditFieldLabel.Text = 'L1';

            % Create L1EditField
            app.L1EditField = uieditfield(app.UIFigure, 'numeric');
            app.L1EditField.HorizontalAlignment = 'center';
            app.L1EditField.Position = [136 860 45 22];

            % Create L2EditFieldLabel
            app.L2EditFieldLabel = uilabel(app.UIFigure);
            app.L2EditFieldLabel.HorizontalAlignment = 'right';
            app.L2EditFieldLabel.Position = [211 881 25 22];
            app.L2EditFieldLabel.Text = 'L2';

            % Create L2EditField
            app.L2EditField = uieditfield(app.UIFigure, 'numeric');
            app.L2EditField.HorizontalAlignment = 'center';
            app.L2EditField.Position = [208 860 42 22];

            % Create L3EditFieldLabel
            app.L3EditFieldLabel = uilabel(app.UIFigure);
            app.L3EditFieldLabel.HorizontalAlignment = 'right';
            app.L3EditFieldLabel.Position = [277 881 25 22];
            app.L3EditFieldLabel.Text = 'L3';

            % Create L3EditField
            app.L3EditField = uieditfield(app.UIFigure, 'numeric');
            app.L3EditField.HorizontalAlignment = 'center';
            app.L3EditField.Position = [276 860 38 22];

            % Create SetLengthsButton
            app.SetLengthsButton = uibutton(app.UIFigure, 'push');
            app.SetLengthsButton.ButtonPushedFcn = createCallbackFcn(app, @SetLengthsButtonPushed, true);
            app.SetLengthsButton.Position = [179 826 100 23];
            app.SetLengthsButton.Text = 'Set Lengths';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Robotic_Finger_GUI_App

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end