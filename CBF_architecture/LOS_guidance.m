function chi_d = LOS_guidance(e_y, pi_p, Delta)
% LOS_GUIDANCE computes desired course using Line-of-Sight (LOS) guidance
%
% Inputs:
%   e_y  - cross-track error (perpendicular distance to path) [m]
%   pi_p - path-tangent angle (from crossTrackError) [rad]
%   Delta - lookahead distance [m] (optional, default 80 m)
%
% Output:
%   chi_d - desired course angle [rad], wrapped to [-pi, pi]
%
% Implements Fossen (2021) Eq. 12.78:
%   chi_d = pi_p - atan(K_p * e_y),  with K_p = 1/Delta

    if nargin < 3
        Delta = 80;  % default lookahead distance [m]
    end
    if Delta <= 0
        error('LOS_guidance: Delta must be positive');
    end

    K_p = 1 / Delta;

    chi_d = pi_p - atan2(K_p * e_y, 1);

    % Wrap to [-pi, pi] for consistency
    chi_d = ssa(chi_d);

end
