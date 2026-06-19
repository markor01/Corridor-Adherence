function [segments, totalLength] = dubinsSegment(x1,y1,psi1, x2,y2,psi2, R, dubinsWord)
% Computes one Dubins word (CSC) based on two poses. 

% Inputs:
%   x1,y1,psi1  -   start position [m] and heading [rad]
%   x2,y2,psi2  -   end/goal position [m] and heading [rad]
%   R           -   turning radius [m]
%   dubinsWord  -   Either 'RSR', 'LSL', 'RSL' or 'LSR'

% Outputs:
%   segments    -   cell array (1x3) of structs for pathParam.m
%                   returns empty if not feasible
%   totalLength -   total path length [m] of set of segments
%                   returns Inf if not feasible

segments = {};
totalLength = Inf;

% Offsets for turn centers, right is +90 deg offset, left is -90 deg offset
left = @(psi) [sin(psi); -cos(psi)];    % Unit vector pointing left of heading
right = @(psi) [-sin(psi); cos(psi)];   % Unit vector pointing right of heading

% Start and end positions
p1 = [x1; y1];
p2 = [x2; y2];

switch dubinsWord
    case 'RSR'
        c1 = p1 + R * right(psi1);  % center of start arc in a right turn
        c2 = p2 + R * right(psi2);  % center of end/goal arc in right turn
        [seg1,seg2,seg3,ok_flag] = csc_ext(p1,c1,'CW', p2,c2,'CW', R);
    case 'LSL'
        c1 = p1 + R * left(psi1);
        c2 = p2 + R * left(psi2);
        [seg1,seg2,seg3,ok_flag] = csc_ext(p1,c1,'CCW', p2,c2,'CCW', R);
    case 'RSL'
        c1 = p1 + R * right(psi1);
        c2 = p2 + R * left(psi2);
        [seg1,seg2,seg3,ok_flag] = csc_int(p1,c1,'CW', p2,c2,'CCW', R);
    case 'LSR'
        c1 = p1 + R * left(psi1);
        c2 = p2 + R * right(psi2);
        [seg1,seg2,seg3,ok_flag] = csc_int(p1,c1,'CCW', p2,c2,'CW', R);
    otherwise
        error('dubinsSegment.m: unknown dubinsWord ''%s''', dubinsWord);
end

% Check if dubinsWord is geometrically feasible
if ~ok_flag
    return
end

% Outputs
segments = {seg1, seg2, seg3};
totalLength = segLen(seg1) + segLen(seg2) + segLen(seg3);

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function[seg1, seg2, seg3, ok_flag] = csc_ext(p1,c1,dir1, p2,c2,dir2, R)
% Computes the three segments of a CSC Dubins word with external tangent
% (RSR or LSL, same-side turns).
% Tangent line is prallel to c1->c2 with an offset R
% dir1/dir2 are the directions of the turns, either 'CW' or 'CCW'.

ok_flag = false;
seg1 = [];
seg2 = [];
seg3 = [];

% Distance between centers of the arcs
d = norm(c2 - c1);

if d < 1e-6     % Circles have same center, no solution
    return;
end

% Calculating the tangent points:

alpha = atan2(c2(2)-c1(2), c2(1)-c1(1));    % Angle of vector pointing from c1 to c2 (bearing)

% Perpendicular offset direction, CW -> right, CCW -> left
% For RSR (CW) path is to the left of c1->c2, then perp is left of alpha
% For LSL (CCW) path is to the right of c1->c2, then perp is right of alpha
if strcmp(dir1, 'CW')
    perp = [sin(alpha); -cos(alpha)];    % left of c1->c2
else
    perp = [-sin(alpha); cos(alpha)];   % right of c1->c2
end

t1 = c1 + R * perp;     % tangent point leaving arc 1
t2 = c2 + R * perp;     % tangent point entering arc 2

seg1 = makeArc(c1, R, p1, t1, dir1);
seg2 = struct('type','line', 'start',t1', 'end',t2', 'heading',alpha);
seg3 = makeArc(c2, R, t2, p2, dir2);
ok_flag = true;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [seg1, seg2, seg3, ok_flag] = csc_int(p1,c1,dir1, p2,c2,dir2, R)
% Computes the three segments of a CSC Dubins word with internal tangent
% (RSL or LSR, opposite-side turns).
% Tangent line crosses c1->c2
% dir1/dir2 are the directions of the turns, either 'CW' or 'CCW'.

ok_flag = false;
seg1 = [];
seg2 = [];
seg3 = [];

% Distance between centers of the arcs
d = norm(c2 - c1);

if d < 2*R  % Circles overlap, no solution
    return;
end

alpha = atan2(c2(2)-c1(2), c2(1)-c1(1));    % Bearing c1->c2 in NED
beta = acos(2*R / d);   % Angle between c1->c2 and c1->t1

% RSL: CW first, LSR: CCW first
if strcmp(dir1, 'CW')   % RSL
    theta1 = alpha - beta;  % Direction of c1->t1
    straight_heading = theta1 + pi/2;   % Direction of straight line segment
else
    theta1 = alpha + beta;
    straight_heading = theta1 - pi/2;
end

t1 = c1 + R * [cos(theta1); sin(theta1)];     % tangent point leaving arc 1
t2 = c2 - R * [cos(theta1); sin(theta1)];     % tangent point entering arc 2

seg1 = makeArc(c1, R, p1, t1, dir1);
seg2 = struct('type','line', 'start',t1', 'end',t2', 'heading', straight_heading);
seg3 = makeArc(c2, R, t2, p2, dir2);
ok_flag = true;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function seg = makeArc(center, R, p_start, p_end, direction)
% Creates the struct that defines an arc

% Calculating bearing of vector from center to p_start, and from center to p_end
theta_start = atan2(p_start(2)-center(2), p_start(1)-center(1));
theta_end = atan2(p_end(2)-center(2), p_end(1)-center(1));

% Calculating ship heading at at start and end of the arc
if strcmp(direction, 'CW')
    psi_start = theta_start + pi/2;
    psi_end = theta_end + pi/2;
else
    psi_start = theta_start - pi/2;
    psi_end = theta_end - pi/2;
end

seg = struct('type','arc', 'center',center', 'radius',R, 'angle',[psi_start, psi_end], 'direction',direction);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5


function L = segLen(seg)
% Calculates the length of a segment

if strcmp(seg.type, 'line')
    L = norm(seg.end - seg.start);
else
    L = abs(ssa(diff(seg.angle))) * seg.radius;
end


end