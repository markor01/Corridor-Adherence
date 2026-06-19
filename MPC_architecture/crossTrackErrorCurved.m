function [e_y,pi_p] = crossTrackErrorCurved(x_p, y_p, dx_p, dy_p, xn, yn)
    
    pi_p = atan2(dy_p, dx_p);   % eq 12.160 Fossen

    rot_pi = [cos(pi_p) -sin(pi_p);
              sin(pi_p) cos(pi_p)];

    track_error = rot_pi'*[xn - x_p;     % 12.161 Fossen, but the matrix  is transposed
                   yn - y_p];
    
    e_y = [0 1]*track_error;            % Extracting cross-track error
    
    % Can alternatively use 12.162 and .163

end