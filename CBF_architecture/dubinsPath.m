function [segments, totalLength, dubinsWord] = dubinsPath(x1,y1,psi1, x2,y2,psi2, R)
% Finds the shortest feasible path between two poses

% Inputs:
%   x1,y1,psi1  -   start position [m] and heading [rad]
%   x2,y2,psi2  -   end/goal position [m] and heading [rad]
%   R           -   minimum turning radius [m]

% Outputs:
%   segments    -   cell array of structs compatible with pathParam.m
%                   returns empty if no path is found
%   totalLength -   total path length [m] of set of segments
%                   returns Inf if no path is found
%   dubinsWord  -   string identifying the chosen Dubins word

segments = {};
totalLength = Inf;
dubinsWord = '';

%---------------------------------------------------------------
% Case 1: already at goal position

if norm([x2-x1, y2-y1]) < 1e-3 && abs(ssa(psi2 - psi1)) < 1e-6
    totalLength = 0;
    return
end


%-----------------------------------------------------------------
% Case 2: same heading and goal straight ahead, pure straight line

bearing = atan2(y2-y1, x2-x1);

if abs(ssa(psi1 - bearing)) < 1e-6 && abs(ssa(psi2 - bearing)) < 1e-6

    p1 = [x1 , y1];
    p2 = [x2 , y2];

    segments = {struct('type','line', 'start',p1, 'end',p2, 'heading',psi1)};
    totalLength = norm([x2-x1, y2-y1]);
    dubinsWord = 'straight';
    return
end


%-------------------------------------------------------
% General case: try all four CSC Dubins words and pick shortest feasible

words = {'RSR', 'LSL', 'RSL', 'LSR'};

shortest_length = Inf;
shortest_segs = {};
shortest_word = '';

for w = 1:4
    [segs, L] = dubinsSegment(x1,y1,psi1, x2,y2,psi2, R, words{w});
    if ~isempty(segs) && L < shortest_length
        shortest_length = L;
        shortest_segs = segs;
        shortest_word = words{w};
    end
end

if isempty(shortest_segs)
    warning('dubinsPath.m: no feasible path found between the poses')
    return
end

segments = shortest_segs;
totalLength = shortest_length;
dubinsWord = shortest_word;

end