function [x_p, y_p, dx_p, dy_p, pathLength, seg_idx, kappa] = pathParam(path_var, path)
    
    % Determine length of segments
    segLen = zeros(1,numel(path));
    for segment = 1:numel(path)
        if path{segment}.type == "line"
            segLen(segment) = norm(path{segment}.end - path{segment}.start);
        elseif path{segment}.type == "arc"
            segLen(segment) = abs(ssa(diff(path{segment}.angle))) * path{segment}.radius;   % Arc length
        end
    end
    cumLen = [0, cumsum(segLen)];
    pathLength = sum(segLen);   % Total length of the path

    % Determine which segment path_var is in
    seg_idx = find(path_var <= cumLen(2:end), 1);
    if isempty(seg_idx)
        seg_idx = length(cumLen) - 1;
        error('path_var exceeds total path length.')
    end

    % Local path_var for the current segment
    local_path_var = path_var - cumLen(seg_idx);

    % Compute x_p, y_p, dx_p, dy_p based on segment type
    current_seg = path{seg_idx};
    switch current_seg.type
        case 'line'
            psi = current_seg.heading;     % This is the same as pi_p
            x_p = current_seg.start(1) + local_path_var * cos(psi);
            y_p = current_seg.start(2) + local_path_var * sin(psi);
            dx_p = cos(psi);
            dy_p = sin(psi);
            kappa = 0;

        case 'arc'
            if current_seg.direction == "CW"
                directionSign = 1;
                angle_center_start = ssa(current_seg.angle(1) - pi/2);
            elseif current_seg.direction == "CCW"
                directionSign = -1;
                angle_center_start = ssa(current_seg.angle(1) + pi/2);
            else
                error('Arc direction not specified correctly.')
            end
            
            % angle_center_start is the angle of the vector pointing from
            % center of circle to start point of turn relative to the
            % x-axis in NED-frame (North direction). This means that
            % startAngle and angle is the same.
            
            angle = angle_center_start + directionSign * (local_path_var / current_seg.radius); % Arc length / r = angle
            x_p = current_seg.center(1) + current_seg.radius * cos(angle);
            y_p = current_seg.center(2) + current_seg.radius * sin(angle);
            dx_p = -sin(angle) * directionSign;
            dy_p = cos(angle) * directionSign;
            kappa = directionSign / current_seg.radius;
    end
end