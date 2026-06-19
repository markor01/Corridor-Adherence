classdef Navigator < handle % handle makes it behave like python objects
    %NAVIGATOR Mock digital navigator for Dubins path generation
    
    % Generates a continous Dubins path from a list of waypoint poses,
    % attaches corridor widths and speed limits for each segment, and
    % enables replanning in the middle of a maneuver

    % How to use:
    %   nav = Navigator(waypoints, corridor_widths, speed_limits, R);
    %   [x_p,y_p,dx_p,dy_p,y_e_max,U_max] = nav.getPathReference(path_var);
    %   nav.replan(x,y,psi,new_waypoints,new_corridor_widths,new_speed_limits);
    %   flag = nav.replanned();
    %   L = nav.pathLength();
    
    properties (Access = private)
        path            % cell array containing all segment structs
        corridor_widths % [1 x N_segs] corridor half-width per segment[m]
        speed_limits    % [1 x N_segs] speed limit per segment [m/s]
        R               % minimum turning radius [m]
        replan_flag     % true if replan() has been called since last getPathReference
    end
    
    methods
        function obj = Navigator(waypoints, corridor_widths, speed_limits, R)
            %NAVIGATOR Construct an instance of this class
            % (Constructor a continous Dubins path from waypoint poses)

            % Inputs:
            %   waypoint        - [N x 3] matrix of poses [x, y, psi]
            %                     minimum 2 rows (start and goal pose)
            %                     [m, m, rad]
            %   corridor_widths - [1 x N-1] corridor half-width per
            %                     leg between waypoints [m]
            %   speed_limits    - [1 x N-1] speed limit per leg between
            %                     waypoints [m/s]
            %   R               - minimum turning radius

            % Checking if input is in correct format
            assert(size(waypoints,1) >= 2, ...
                'Navigator.m: need at least 2 waypoints')
            assert(numel(corridor_widths) == size(waypoints,1) - 1, ...
                ['Navigator.m: corridor_widths must have one entry per' ...
                ' leg (N_waypoints - 1)'])
            assert(numel(speed_limits) == size(waypoints,1) - 1, ...
                ['Navigator.m: speed_limits must have one entry per' ...
                ' leg (N_waypoints - 1)'])

            obj.R = R;
            obj.replan_flag = false;

            % Build path and expand metadata to per-segment
            [obj.path, obj.corridor_widths, obj.speed_limits] = ...
                obj.buildPath(waypoints, corridor_widths, speed_limits);

        end
        
        function [x_p, y_p, dx_p, dy_p, y_e_max, U_max] = getPathReference(obj, path_var)
            % Returns path geometry and corridor metadata at path_var

            % Inputs:
            %   path_var    - current path parameter (distance travelled
            %                 along path) [m]

            % Outputs:
            %   x_p, y_p    - reference point on path [m]
            %   dx_p, dy_p  - Unit tangent vector, shows what direction the
            %                 path is heading at that point
            %   y_e_max     - corridor half-width for current segment [m]
            %   U_max       - speed limit for current segment [m/s]

            obj.replan_flag = false;

            [x_p, y_p, dx_p, dy_p, ~, seg_idx] = pathParam(path_var, obj.path);

            y_e_max = obj.corridor_widths(seg_idx);
            U_max = obj.speed_limits(seg_idx);
        end

        function replan(obj, x, y, psi, new_waypoints, new_corridor_widths, new_speed_limits)
            % Replaces current path with a new path from current pose

            % Inputs:
            %   x, y, psi           - current pose [m, m, rad]
            %   new_waypoints       - [N x 3] matrix of new goal poses
            %   new_corridor_widths - [1 x N-1] corridor half-widths for
            %                         new legs
            %   new_speed_limits    - [1 x N-1] speed limits for new legs

            % Prepend current pose to new waypoints so the new path starts
            % from current ship location
            all_waypoints = [x, y, psi; new_waypoints];

            assert(numel(new_corridor_widths) == size(new_waypoints,1), ...
                'Navigator.replan: new_corridor_widths must have one entry for each new leg')
            assert(numel(new_speed_limits) == size(new_waypoints,1), ...
                'Navigator.replan: new_speed_limits must have one entry for each new leg')

            [new_path,  new_cw, new_sl] = obj.buildPath( ...
                all_waypoints, new_corridor_widths, new_speed_limits);

            if isempty(new_path)
               warning('Navigator.replan: no valid path found, replan ignored')
               return
            end

            obj.path = new_path;
            obj.corridor_widths = new_cw;
            obj.speed_limits = new_sl;
            obj.replan_flag = true;

            fprintf('Navigator: replanned %d segments, path length L = %.0f m\n', ...
                numel(obj.path), obj.pathLength())
        end

        function flag = replanned(obj)
            % True if replan() was called since last getPathReference
            flag = obj.replan_flag;
        end

        function L = pathLength(obj)
            % Total length of current path [m]
            [~,~,~,~,L,~] = pathParam(0, obj.path);
        end

        function R_seg = getCurrentRadius(obj, path_var)
            % Returns the truning radius of the current segment
            % (Inf for straight lines)

            [~,~,~,~,~,seg_idx] = pathParam(path_var, obj.path);
            seg = obj.path{seg_idx};

            if strcmp(seg.type, 'arc')
                R_seg = seg.radius;
            else
                R_seg = Inf;
            end
        end
    end

    % functions that can only be called from within Navigator.m
    methods (Access = private)
        function [all_segs, all_cw, all_sl] = buildPath(obj, waypoints, corridor_widths, speed_limits)
            % Calls dubinsPath.m between each pair of waypoints and
            % concatenates segments with expanded metadata

            all_segs = {};
            all_cw = [];
            all_sl = [];
            
            for k = 1:size(waypoints,1)-1
                x1 = waypoints(k, 1);
                y1 = waypoints(k, 2);
                psi1 = waypoints(k, 3);
                x2 = waypoints(k+1, 1);
                y2 = waypoints(k+1, 2);
                psi2 = waypoints(k+1, 3);

                [segs, totalLength, dubinsWord] = dubinsPath(x1,y1,psi1, x2,y2,psi2, obj.R);
%                 fprintf('DEBUG leg %d->%d: L=%.0f, straight=%.0f, ratio=%.2f\n', ...
%                     k, k+1, totalLength, norm([x2-x1,y2-y1]), totalLength/norm([x2-x1,y2-y1]))

                % Feasibility check
                straight_dist = norm([x2-x1, y2-y1]);
                if totalLength > 2* straight_dist
                    warning(['Navigator.m: leg %d->%d may be infeasible - path length (%.0f m) ' ...
                        'much longer than straight line distance (%.0f m). Reduce R_min or adjust waypoints'], ...
                        k, k+1, totalLength, straight_dist)
                end

                if isempty(segs)
                    fprintf('Navigator: leg %d->%d is already at goal, skipping leg\n', k, k+1)
                    continue
                end

                % number of segments will always be either 1 or 3 depending
                % on whether it is a CSC dubins word or just a straight
                fprintf('Navigator: leg %d->%d, %s, %d segments\n', ...
                    k, k+1, dubinsWord, numel(segs))

                % Expanding corridor widths and speed limits to cover all
                % segments in this leg
                n = numel(segs);
                all_segs = [all_segs, segs];
                all_cw = [all_cw, repmat(corridor_widths(k), 1, n)];
                all_sl = [all_sl, repmat(speed_limits(k), 1, n)];
            end
        end
    end
end

