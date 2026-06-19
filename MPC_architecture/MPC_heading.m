function [delta_c, infeasible_count_out] = MPC_heading(z0, pi_p_horizon, U, y_e_max, delta_prev, h, N)
% Linear MPC heading controller with hard corridor constraint

% Alternative to ILOS + reference model + PID/FBL controller. 

% Key advantage over PID/FBL with CBF:
%   pi_p_horizon gives MPC full knowledge of upcoming path curvature, so it
%   can begin correcting early rather than reacting as y_e is already
%   growing

% Inputs:
%   z0              - current state [y_e; psi; r]   [m; rad; rad/s]
%   pi_p_horizon    - path tangential angles over   [rad]
%                     horizon, computed in main
%                     before calling this function
%   U               - total ship speed              [m/s]
%   y_e_max         - corridor half width           [m]
%   delta_prev      - previous rudder angle         [rad]
%   h               - sample time                   [s]
%   N               - prediction horizon            [steps]

% Output:
%   delta_c         - commanded rudder angle        [rad]

persistent infeasible_count
if isempty(infeasible_count)
    infeasible_count = 0;
end

% Make shure pi_p_horizon has exactly N+1 elements
assert(numel(pi_p_horizon) == N+1, ['MPC_heading.m: pi_p_horizon ' ...
    'must have N+1 elements (steps 0 to N) where N is horizon length']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prediction model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State: z = [y_e; psi; r]  (cross-track error, heading, yaw rate)
% Input: u = delta_c        (commanded rudder angle)

% Continuous time model where dy_e/dt=U*sin(psi-pi_p) is linearized using
% small angle approximation:

% dy_e/dt = U * (psi-pi_p)
% dpsi/dt = r
% dr/dt   = -1/T * r + K/T * delta  (first order Nomoto)

% Format used by MPC:
%   dz/dt = A_c*z + B_c*u + g_c(k)

%   g_c(k) = [-U*pi_p(k); 0; 0]     (time varying along horizon)

% Nomoto parameters (identified through step test simulation)
% T = 50;     % time constant
% K = 0.027;  % rudder gain
T = 50;     % time constant
K = 0.03;  % rudder gain

% Continuous time system matrices
A_c = [0 U 0;
       0 0 1;
       0 0 -1/T];

B_c = [0; 0; K/T];

% Euler discretization
% z(k+1) = A_d*z(k) + B_d*u(k) + g_d(k)
%   A_d and B_d are constant, g_d varies along the horizon via pi_p_horizon
%   and is computed inside the Gd_vec loop below
A_d = eye(3) + h * A_c;
B_d = h * B_c;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Prediction matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% unroll N steps ahead:
%   Z = Phi_mat*z0 + Gamma_mat*U_vec + Gd_vec
%       where:
%           Z     = [z(1); z(2); ...; z(N)]     predicted states (3*N x 1)
%           U_vec = [u(0); u(1); ...; u(N-1)]   control inputs (N x 1)
%   NOTE:   Z AND U_vec IS SHIFTED RELATIVE TO EACH OTHER WITH ONE
%           TIME STEP!!! u(k) is applied at step k and produces z(k+1)

%           Phi_mat   (3*N x 3) where Phi_mat(3k-2:3k, :) = A_d^k
%           Gamma_mat (3*N x N) where Gamma_mat(3k-2:3k, j) = A_d^(k-j)*B_d 
%           Gd_vec    (3*N x 1) time varying feedforward accumulated over
%                               the horizon

% Gd_vec recursive formula:
%   Gd_vec(k) = A_d*Gd_vec(k-1) + g_d(k-1)
%   where:
%       g_d(k-1) = h*[-U*pi_p(k-1); 0; 0]
%   and
%       pi_p(k-1) = pi_p_horizon(k)     (matlab 1-indexing can be
%                                       confusing here)

Phi_mat = zeros(3*N, 3);
Gamma_mat = zeros(3*N, N);
Gd_vec = zeros(3*N, 1);

% Build Phi_mat and Gd_vec
A_power = eye(3);       % holds A_d^(k-1) at the start of iteration k
Gsum = zeros(3, 1);     % accumulates Gd_vec recursively

for k = 1:N
    % g_d at step k-1 using pi_p(k-1) = pi_p_horizon(k)
    g_d_prev = h * [-U*pi_p_horizon(k); 0; 0];

    % Recursive update
    Gsum = A_d * Gsum + g_d_prev;
    A_power = A_d * A_power;        % now A_power = A_d^k

    Phi_mat(3*k-2:3*k, :) = A_power;
    Gd_vec(3*k-2:3*k) = Gsum;
end

% Build Gamma_mat column by column
%   Gamma_mat is a lower block-triangular Toeplitz matrix where column j
%   corresponds to input u(j-1). The diagonal block (k=j) = B_d = A_d^0*B_d
%   and each block below multiplies by one more A_d (raises power by one)
for j = 1:N
    A_block = eye(3);   % Sets A_d^0 for the diagonal block
    for k = j:N
        Gamma_mat(3*k-2:3*k, j) = A_block * B_d;
        A_block = A_d * A_block;    % A_d^(k-j) for next row
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% J = sum_{k=0}^{N-1} (z(k+1)-z_ref(k+1))'*Q*(z(k+1)-z_ref(k+1)) + u(k)'*R*u(k)

% The reference trajectory is time varying over the prediction horizon:
%   y_e_ref(k) = 0          stay on path centreline
%   psi_ref(k) = pi_p(k)    heading aligned with path tangent at step k
%   r_ref(k)   = 0          no reference yaw rate

% Want to penalize |y_e| > 0 (stay on path centreline), and |psi| > psi_ref
% r not penalized (follows naturally from psi dynamics)

Q_ey = 1.0;     % cross tack error weight
Q_psi = 0.5;    % heading weight, path aligned at each predicted step
% Q_r = 0.0;      % yaw rate weight, could be increased if oscillations
Q_r = 0.5;
% R_u = 1e-3;     % rudder effort weight (increase to smooth, decrease to sharpen)
% R_u = 0.5;
R_u = 50;


Q_block = diag([Q_ey, Q_psi, Q_r]);
Q = kron(eye(N), Q_block);      % block diagonal cost over horizon
R = R_u * eye(N);

% Time varying reference trajectory:
%   psi_ref(k) = pi_p(k) = pi_p_horizon(k+1) [1-indexed]
Z_ref = zeros(3*N,  1);
for k = 1:N
    Z_ref(3*k-2:3*k) = [0; pi_p_horizon(k+1); 0];
end

% Derivation of condensed cost function:
%   J = (Z - Z_ref)' * Q * (Z - Z_ref) + U_vec' * R * U_vec
%   where:
%       Z = Phi_mat*z0 + Gamma_mat*U_vec + Gd_vec
%   so
%       Z - Z_ref = (Phi_mat*z0 + Gamma_mat*U_vec + Gd_vec) - Z_ref
%                 = Z_free - Z_ref + Gamma_mat*U_vec
%                 = E_free + Gamma_mat*U_vec
%   E_Free is the predicted error if the MPC does nothing

%   J = (E_free + Gamma_mat*U_vec)'*Q*(E_free + Gamma_mat*U_vec) + U_vec'*R*U_vec
%     = E_free'*Q*E_free + 2*E_free'*Q*Gamma_mat*U_vec +
%           U_vec'*Gamma_mat'*Q*Gamma_mat*U_vec + U_vec'*R*U_vec
%   The first term is a constant (does not depend on U_vec) and can be
%   dropped as it does not affect the minimizer
%   J = U_vec'*(Gamma_mat'*Q*Gamma_mat + R)*U_vec + 
%           2*(Gamma_mat'*Q*E_free)'*U_vec

%   Now H_qp and f_qp can be found by matching to the quadprog format:
%       min 0.5*x'*H*x + f'*x

% Free response: predicted Z without future inputs
Z_free = Phi_mat*z0 + Gd_vec;

% Error of free response from reference trajectory
E_free = Z_free - Z_ref;

% QP cost: min 0.5 * U_vec' * H_qp * U_vec + f_qp' * U_vec
H_qp = Gamma_mat' * Q * Gamma_mat + R;
f_qp = Gamma_mat' * Q * E_free;

% Symmetrize to avoid warnings in quadprog from floating point asymmetry
H_qp = (H_qp + H_qp')/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraint matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% A_ineq * U_vec <= b_ineq

% 1: Rudder angle bounds (-delta_max <= u(k) <= delta_max)
delta_max = 40 * pi/180;    % 40 degrees in [rad]
A_rud = [eye(N); -eye(N)];
b_rud = delta_max * ones(2*N, 1);

% 2: Rudder rate bounds (|u(k)-u(k-1)| <= ddelta_max * h [rad/step])
%   Difference matrix D extracts increments between consecutive inputs
ddelta_max = 5 * pi/180;    % 5 deg/s in [rad/s]
D = eye(N) - diag(ones(N-1, 1), -1);    % lower bidiagonal difference matrix

%   First step is |u(0)-delta_prev| <= ddelta_max * h
%   delta_prev is not inside U_vec, but only affects first step
b_upper = ddelta_max*h * ones(N, 1);
b_lower = ddelta_max*h * ones(N, 1);

b_upper(1) = ddelta_max*h + delta_prev;
b_lower(1) = ddelta_max*h - delta_prev;

A_rate = [D; -D];
b_rate = [b_upper; b_lower];

% 3: Corridor constraint (|y_e(k)| <= y_e_max for all k in horizon)
%   y_e(k) = [1 0 0]*z(k), extracted using block selector matrix C_ey
C_ey = kron(eye(N), [1 0 0]);   % (N x 3*N), picks y_e from each z(k)

% C_ey * Z = [y_e(1); y_e(2); ...; y_e(N)]  (N x 1)
%          = C_ey * (Phi_mat*z0 + Gamma_mat*U_vec + Gd_vec)
%          = y_e_free + C_ey*Gamma_mat*U_vec

% |y_e(k)| <= y_e_max
% |y_e_free + C_ey*Gamma_mat*U_vec| <= y_e_max
% which means:
%   C_ey*Gamma_mat*U_vec <= y_e_max - y_e_free
% and
%   -C_ey*Gamma_mat*U_vec <= y_e_max + y_e_free
% compare to:
%   A_ineq * U_vec <= b_ineq

y_e_free = C_ey * Z_free;       % predicted y_e with no future input

A_corr = [C_ey * Gamma_mat;
         -C_ey * Gamma_mat];
b_corr = [y_e_max * ones(N, 1) - y_e_free;
          y_e_max * ones(N, 1) + y_e_free];

% Stack all inequality constraints
A_ineq = [A_rud; A_rate; A_corr];
b_ineq = [b_rud; b_rate; b_corr];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Solve QP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('quadprog', 'Display','off');
% if any(b_corr < 0)
%     fprintf('Step %d: b_corr min=%.3f at index %d, y_e_free range=[%.2f, %.2f]\n', ...
%         i, min(b_corr), find(b_corr==min(b_corr)), min(y_e_free), max(y_e_free));
% end
[U_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, ...
                                [], [], [], [], [], options);

% Apply only the first optimal input
if exitflag > 0     % feasible point is found
    delta_c = U_opt(1);
    infeasible_count = 0;
else
    % Original QP infeasible
    % Remove corridor constraint and resolve, safety is abandoned but
    % recovery is prioritized
    
    A_ineq_recovery = [A_rud; A_rate];
    b_ineq_recovery = [b_rud; b_rate];
    [U_recovery, ~, exitflag2] = quadprog(H_qp, f_qp, A_ineq_recovery, ...
        b_ineq_recovery, [], [], [], [], [], options);

    if exitflag2 > 0
        delta_c = U_recovery(1);
    else
        delta_c = delta_prev;   % last resort
    end
    
    infeasible_count = infeasible_count + 1;
    if mod(infeasible_count, 500) == 1
        warning('MPC_heading.m: QP with constraint is infeasible (exitflag=%d), corridor dropped for recovery', exitflag);
    end


end

infeasible_count_out = infeasible_count;

end

