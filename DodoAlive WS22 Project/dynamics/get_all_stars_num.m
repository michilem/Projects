function all_stars = get_all_stars_num(q, dq)
% Compute terms for com frame constrained dynamics numerically.
% Some inverse matrices are too complicated to calculate using symbolic math.

g = 9.81;

% Get inverse mass matrix
M_inv = get_M_inv(q);

% The selection matrix for actuated joints
S = zeros(2,4);
S(1,3) = 1;
S(2,4) = 1;

% Contact point jacobian and the time derivative of it.
J_c = get_J_c(q);
J_c_dot = get_J_c_dot([q;dq]);

% Center of mass jacobian and the time derivative of it.
J_com = get_jac_com(q); %2*4
J_com_dot = get_J_cog_dot([q;dq]);

% Coriolis matrix
C = get_Coriolis(q, dq);

% Null space stance
lambda_s = inv(J_c * M_inv * J_c');
N_s = eye(4) - M_inv * J_c' * lambda_s * J_c;

S_Ns_t = transpose(S * N_s); %4*2

J_star = J_com * M_inv * S_Ns_t * inv(S * N_s * M_inv * S_Ns_t); %2*2

lambda_star = pinv(J_com * M_inv * S_Ns_t * transpose(J_star)); %2*2;

% Coriolis term
b = C * dq;

% Gravitational term
grav = get_G(q);

mu_star = lambda_star * J_com * M_inv * transpose(N_s) * b ...
            - lambda_star * J_com_dot * dq...
            + lambda_star * J_com * M_inv * transpose(J_c) * lambda_s * J_c_dot * dq;

p_star = lambda_star * J_com * M_inv * transpose(N_s) * grav; 

all_stars = {J_star; lambda_star; mu_star; p_star};
