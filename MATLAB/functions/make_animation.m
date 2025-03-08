function make_animation(th, r1, r2, xlims, ylims, zlims, filename, scale_factor, pre_rot_dcm, offset, fth, cullrate, fixed_view)
%MAKE_ANIMATION Animates an STL model using position and quaternion state history.
%
% th is an Nx13 matrix where:
%   Columns 1-3  = [x, y, z] position
%   Columns 7-10 = [qx, qy, qz, qw] quaternion
%
% filename = STL file path
% fixed_view = true to lock the camera to the vehicle's origin
% cullrate = how frequently frames are processed (for performance)

    % 1) Load STL model
    [F, V] = stlread(filename);
    
    % 1.5) Immediately rotate the STL and shift the origin.
    for i = 1:length(V)
        V(i,:) = V(i,:)*pre_rot_dcm.';
        V(i,:) = V(i,:) + offset * 1/scale_factor;
    end

    % 2) Figure Setup
    fig = figure('Color', 'white', 'Position', [100, 100, 800, 600]);
    hold on; grid on; axis equal;
    
    % Default view settings
    default_view = [-135 35];  % Standard perspective view
    view(default_view);

    % 3) Patch for STL
    patch_handle = patch('Faces', F, 'Vertices', V, ...
        'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);

    camlight('headlight');
    material('dull');

    % Store original vertices
    V0 = V * scale_factor;

    % Initialize quiver arrows for forces
    f1_quiver = quiver3(0, 0, 0, 0, 0, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    f2_quiver = quiver3(0, 0, 0, 0, 0, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    % Initialize quivers for body-fixed coordinate axes
    body_x = quiver3(0, 0, 0, 0, 0, 0, 'r--', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X-axis (red)
    body_y = quiver3(0, 0, 0, 0, 0, 0, 'g--', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y-axis (green)
    body_z = quiver3(0, 0, 0, 0, 0, 0, 'b--', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z-axis (blue)

    % 4) Video Setup
    vid = VideoWriter('animation.mp4', 'MPEG-4');
    vid.FrameRate = 30;
    open(vid);

    % Define a bounding box size for tracking the vehicle in view
    bound_size = max([diff(xlims), diff(ylims), diff(zlims)]) / 2;  % Maintain consistent frame size

    % 5) Animation Loop
    for k = 1:cullrate:size(th,1)
        % Extract position
        pos = th(k, 1:3); % [x, y, z]

        % Extract quaternion (qx, qy, qz, qw)
        qx = th(k, 7);
        qy = th(k, 8);
        qz = th(k, 9);
        qw = th(k, 10);

        % Convert quaternion to rotation matrix
        R = quat_to_rotm([qw, qx, qy, qz]);

        % Apply rotation and translation to STL vertices
        V_transformed = (R * V0')' + pos;

        % Compute thruster positions
        f1_loc = (R * (-r1)')' + pos;
        f2_loc = (R * (-r2)')' + pos;

        % Compute rotated force vectors
        f2_rotated = (R * -fth(k, 1:3).').';
        f1_rotated = (R * -fth(k, 4:6).').';

        % Compute body-fixed coordinate frame vectors
        x_axis = R(:,1)/3'; % Body X-axis (red)
        y_axis = -R(:,2)/3'; % Body Y-axis (green)
        z_axis = -R(:,3)/3'; % Body Z-axis (blue)

        % Update quiver arrows (replace instead of stacking)
        set(f1_quiver, 'XData', f1_loc(1), 'YData', f1_loc(2), 'ZData', f1_loc(3), ...
                       'UData', f1_rotated(1), 'VData', f1_rotated(2), 'WData', f1_rotated(3));

        set(f2_quiver, 'XData', f2_loc(1), 'YData', f2_loc(2), 'ZData', f2_loc(3), ...
                       'UData', f2_rotated(1), 'VData', f2_rotated(2), 'WData', f2_rotated(3));

        % Update body-fixed coordinate system
        set(body_x, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                    'UData', x_axis(1), 'VData', x_axis(2), 'WData', x_axis(3));

        set(body_y, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                    'UData', y_axis(1), 'VData', y_axis(2), 'WData', y_axis(3));

        set(body_z, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                    'UData', z_axis(1), 'VData', z_axis(2), 'WData', z_axis(3));

        % Update STL
        set(patch_handle, 'Vertices', V_transformed);

        % **New: Adjust Camera View and Axis Limits if `fixed_view` is Enabled**
        if fixed_view
            camtarget(pos);  % Keep camera looking at the vehicle's origin
            
            % Dynamically update the axis limits around the vehicle
            xlim([pos(1) - bound_size, pos(1) + bound_size]);
            ylim([pos(2) - bound_size, pos(2) + bound_size]);
            zlim([pos(3) - bound_size, pos(3) + bound_size]);
        end

        % Capture frame
        frame = getframe(gcf);
        writeVideo(vid, frame);
        drawnow;
    end

    % 6) Close video
    close(vid);
    disp('Animation saved to animation.mp4');
end
