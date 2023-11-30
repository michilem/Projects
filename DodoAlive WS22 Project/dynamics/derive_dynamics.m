%% Derive monopod dynamics
% Init
clear; clc;

syms x y q1 q2 real
syms dx dy dq1 dq2 real
syms ux uy u1 u2 real
syms H_d
syms t

%% State & control vectors
q = [x; y; q1; q2];
dq = [dx; dy; dq1; dq2];
u = [ux; uy; u1; u2];

%% Load monopod parameters
load('monopod_parameters', 'm_B', 'm_UL', 'm_LL', 'l_B', 'l_UL', 'l_LL', 'I_B', 'I_UL', 'I_LL', 'K_l', 'q_rest')
g = 9.81;
%% Homogeneous transformations
% World to base frame
W_X_B = [1, 0, 0, x;
        0,  1, 0, y;
         0, 0, 1, 0;
         0, 0, 0, 1];

% Base to upper leg frame
B_X_UL = [cos(q1), -sin(q1), 0, l_UL/2*cos(q1);
          sin(q1),  cos(q1), 0, l_UL/2*sin(q1);
                0,        0, 1,              0;
                0,        0, 0,              1];

% Upper leg to lower leg frame
UL_X_LL = [cos(q2), -sin(q2), 0, l_UL/2+l_LL/2*cos(q2);
           sin(q2),  cos(q2), 0,        l_LL/2*sin(q2);
                 0,        0, 1,                     0;
                 0,        0, 0,                     1];

% Lower leg to foot frame
LL_X_F = [1, 0, 0, l_LL/2;
          0, 1, 0,      0;
          0, 0, 1,      0;
          0, 0, 0,      1];

% Compute frames
W_X_UL = W_X_B * B_X_UL;
W_X_LL = W_X_UL * UL_X_LL;
W_X_F = W_X_LL * LL_X_F;
B_X_LL = B_X_UL * UL_X_LL;
B_X_F = B_X_LL * LL_X_F;

X_CoM = {W_X_B, W_X_UL, W_X_LL};
X_base_CoM = {sym(eye(4)), B_X_UL, B_X_LL};
%% Polar coordinates
B_x_F = B_X_F(1, 4);
B_y_F = B_X_F(2, 4);

l_pol = sqrt(B_x_F^2 + B_y_F^2);  % Tutorial 5: Equation (3.1)
phi_pol = atan2(B_y_F, B_x_F);  % Tutorial 5: Equation (3.2)

r_F_pol = simplify([l_pol; phi_pol]);

J_pq = simplify(jacobian(r_F_pol, [q1, q2]));  % Polar jacobian

total_mass = 0;
%% Translational jacobian
for i = 1:length(X_CoM)
    r_CoM{i} = simplify(X_CoM{i}(1:3, 4));  % Position vector
    J_v{i} = jacobian(r_CoM{i}, q);
    J_w{i} = sym(zeros(3,4));
end

% Manually derived from rotational joints
J_w{2}(3,3) = 1;
J_w{3}(3,3) = 1;
J_w{3}(3,4) = 1;


%% Mass matrix
m = {m_B, m_UL, m_LL};  % Mass
I = {I_B, I_UL, I_LL};  % Inertia
M = sym(zeros(4, 4));

pos_base_CoM = sym([0;0;0]);
pos_world_CoM = sym([0;0;0]);
jac_CoM = sym(zeros(3, 4));

for i = 1:length(m)  
    total_mass = total_mass + m{i};
    r_base_CoM = simplify(X_base_CoM{i}(1:3, 4));
    pos_base_CoM = pos_base_CoM + r_base_CoM * m{i};
    pos_world_CoM = pos_world_CoM + X_CoM{i}(1:3, 4) * m{i};
    jac_CoM = jac_CoM + J_v{i} * m{i};

    M = M + m{i} * (transpose(J_v{i}) * J_v{i}) + (transpose(J_w{i}) * I{i} * J_w{i});  % Tutorial 1: Equation 4.15
end

M = simplify(M);
M_inv = inv(M);

world_CoM = pos_world_CoM / total_mass;
world_CoM = simplify(world_CoM);
pos_CoM = pos_base_CoM / total_mass;
jac_CoM = jac_CoM / total_mass;

%% Coriolis Matrix
C = sym(zeros(4, 4));

for i=1:size(C,1)
    for j=1:size(C,2)
        for k=1:length(q)
           C(i,j) = C(i,j) + 0.5 * (diff(M(i, j), q(k)) + diff(M(i, k), q(j)) - diff(M(j, k), q(i))) * dq(i);  % Tutorial 1: Equation 4.22
        end
    end
end

C = simplify(C);

%% Gravitational terms
grav = [0; g];
E_pot_g = sym(0.);

for i = 1:length(X_CoM)
    E_pot_g = E_pot_g + (m{i} * r_CoM{i}(1:2,:));
end
E_pot_g = transpose(grav) * E_pot_g;

G = simplify(transpose(jacobian(E_pot_g, q)));  % Tutorial 1 Equation 4.24

%% Rest length for elastic potential energy
q1_rest = q_rest(2);
q2_rest = q_rest(3);
l_rest = subs(l_pol, [q1 q2], [q1_rest q2_rest]);

%% Total system energy
E_pot_e = 0.5 * K_l * (l_rest - l_pol)^2;
E_kin = 0.5 * dq' * M * dq;

E_sys = E_pot_g + E_kin + E_pot_e;

%% Constraint forces
r_F = simplify(W_X_F(1:3, 4));  % Position vector of foot
J_r_F = jacobian(r_F, q);  % Jacobian of foot position
J_c = simplify(J_r_F(1:2, :));  % Constraint Jacobian

for i = 1:2
    H{i} = jacobian(J_c(i,:), q);  % Hessian tensor
    dJ_c{i} = H{i} * dq; 
end

J_c_T = transpose(J_c);
J_c_dot = simplify([transpose(dJ_c{1}); transpose(dJ_c{2});]);

J_cq = simplify(jacobian([B_x_F,B_y_F], q));
for i = 1:2
    H{i} = jacobian(J_cq(i,:), q);  % Hessian tensor
    dJ_cq{i} = H{i} * dq;
end
J_cq_dot = simplify([transpose(dJ_cq{1}); transpose(dJ_cq{2});]);
n = C * dq + G;

L = simplify(inv(J_c * M_inv * J_c_T));
lambda = L * (J_c * M_inv * (n - u) - J_c_dot * dq);  % Sobotka: Equation 2.15
lambda = simplify(lambda);

S = sym(zeros(4,4));
S(3,3) = 1;
S(4,4) = 1;
N_s = sym(eye(4,4)) - (M_inv * J_c_T * L * J_c);

%% mu_star
for i = 1:2
    H{i} = jacobian(jac_CoM(i,:), q);  % Hessian tensor for COG
    djac_CoM{i} = H{i} * dq;
end
J_CoM_dot = simplify([transpose(djac_CoM{1}); transpose(djac_CoM{2});]);

%% Impact law
dq_plus = dq - M_inv * J_c_T * L * J_c * dq;  % Sobotka: Equation 2.19
dq_plus = simplify(dq_plus);

%% Flight phase
ddq_flight = M_inv * (u - n);  % Tutorial 5: Equation 1.1 solved for ddq

%% Stance phase
ddq_stance = M_inv * (u + J_c_T * lambda - n);  % Tutorial 5: Equation 1.2 solved for ddq
%ddq_stance = M_inv * (transpose(S * N_s) * u - (N_s' * n + J_c' * L * J_c_dot * dq));
%% ODEs
ode_flight = [dq; ddq_flight];
ode_stance = [dq; ddq_stance];

%% Export functions
if isfolder('dynamics')
    path_dest = fullfile(pwd, 'dynamics');
else
    path_dest = fullfile(fileparts(pwd), 'dynamics');
end

% Foot y position wrt world frame
W_y_F = W_X_F(2, 4);
matlabFunction(W_y_F, 'File', fullfile(path_dest, 'get_c'), 'Vars', {q});

% CoM
matlabFunction(world_CoM(1:2), 'File', fullfile(path_dest, 'get_world_com'), 'Vars', {q});
matlabFunction(pos_CoM(1:2), 'File', fullfile(path_dest, 'get_pos_com'), 'Vars', {q});
matlabFunction(jac_CoM(1:2,:), 'File', fullfile(path_dest, 'get_jac_com'), 'Vars', {q});
% Foot position wrt body frame
foot_pos = B_X_F(1:2, 4);
matlabFunction(foot_pos, 'File', fullfile(path_dest, 'get_foot_pos'), 'Vars', {q});
% Frame orientation and position wrt world frame
matlabFunction(W_X_B, 'File', fullfile(path_dest, 'get_W_X_B'), 'Vars', {q});
matlabFunction(W_X_UL, 'File', fullfile(path_dest, 'get_W_X_UL'), 'Vars', {q});
matlabFunction(W_X_LL, 'File', fullfile(path_dest, 'get_W_X_LL'), 'Vars', {q});
matlabFunction(W_X_F, 'File', fullfile(path_dest, 'get_W_X_F'), 'Vars', {q});
% Foot position in polar copordinates wrt world frame
matlabFunction(r_F_pol, 'File', fullfile(path_dest, 'get_r_F_pol'), 'Vars', {q});

% Jacobian polar to general coordinates
matlabFunction(J_pq, 'File', fullfile(path_dest, 'get_J_pq'), 'Vars', {q});

% Jacobian cartesian to general coordinates

matlabFunction(J_cq, 'File', fullfile(path_dest, 'get_J_cq'), 'Vars', {q});

% Jacobians
matlabFunction(J_v{1}(1:2,:), 'File', fullfile(path_dest, 'get_J_v_B'));
matlabFunction(J_v{2}(1:2,:), 'File', fullfile(path_dest, 'get_J_v_UL'), 'Vars', {q});
matlabFunction(J_v{3}(1:2,:), 'File', fullfile(path_dest, 'get_J_v_LL'), 'Vars', {q});

% Rest length for elastic potential energy
matlabFunction(l_rest, 'File', fullfile(path_dest, 'get_l_rest'));

% Mass matrix
matlabFunction(M, 'File', fullfile(path_dest, 'get_M'), 'Vars', {q});
matlabFunction(M_inv, 'File', fullfile(path_dest, 'get_M_inv'), 'Vars', {q});

% Grav matrix
matlabFunction(G, 'File', fullfile(path_dest, 'get_G'), 'Vars', {q});
matlabFunction(C, 'File', fullfile(path_dest, 'get_Coriolis'), 'Vars', {q, dq});
% Bias term
matlabFunction(n, 'File', fullfile(path_dest, 'get_n'), 'Vars', {q, dq});

% Contact Jacobian
matlabFunction(J_c, 'File', fullfile(path_dest, 'get_J_c'), 'Vars', {q});

% System energies
matlabFunction(E_sys, 'File', fullfile(path_dest, 'get_E_sys'), 'Vars', {[q; dq]});

% Contact forces function
matlabFunction(lambda, 'File', fullfile(path_dest, 'get_lambda'), 'Vars', {[q; dq]; u});

matlabFunction(L, 'File', fullfile(path_dest, 'get_L'), 'Vars', {q});

% Flight dynamics function
matlabFunction(ode_flight, 'File', fullfile(path_dest, 'get_dyn_flight'), 'Vars', {t; [q; dq]; u});

% N_s
matlabFunction(N_s, 'File', fullfile(path_dest, 'get_N_s'), 'Vars', {[q; dq]});

% J_cog_dot
matlabFunction(J_CoM_dot, 'File', fullfile(path_dest, 'get_J_cog_dot'), 'Vars', {[q; dq]});
matlabFunction(J_c_dot, 'File', fullfile(path_dest, 'get_J_c_dot'), 'Vars', {[q; dq]});
matlabFunction(J_cq_dot, 'File', fullfile(path_dest, 'get_J_cq_dot'), 'Vars', {[q; dq]});

% Stance dynamics function
matlabFunction(ode_stance, 'File', fullfile(path_dest, 'get_dyn_stance'), 'Vars', {t; [q; dq]; u});

% Impact jump map function
matlabFunction(dq_plus, 'File', fullfile(path_dest, 'get_impact'), 'Vars', {[q; dq]});
