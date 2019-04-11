function Ct = translation_joint_dq(i, j, q, q_int)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
r_i = q(idx_i(1:2));
phi_i_int = q_int(idx_i(3));
idx_j = body_idx(j);
phi_j = q(idx_j(3));
r_j = q(idx_j(1:2));
phi_j_int = q_int(idx_j(3));

ni = [-r_i(2);r_i(1)];
dj = r_j(1:2);
Ct = [r_j(2), -r_j(1), 0, -r_i(2), r_i(1) 0; 0, 0, 1, 0, 0, -1];
end
