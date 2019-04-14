function C_r_dq = revolute_joint_dq(i, j, s_i, s_j, q)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
idx_j = body_idx(j);
phi_j = q(idx_j(3));

C_r_dq = [eye(2), omega * rot(phi_i) * s_i, ...
    -eye(2), -omega * rot(phi_j) * s_j];
