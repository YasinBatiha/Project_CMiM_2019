function Ct = translation_joint_dtt(i, j, dq)

idx_i = body_idx(i);
dr_i = dq(idx_i(1:2));
idx_j = body_idx(j);
dr_j = dq(idx_j(1:2));

Ct = [dr_i(2)*dr_j(1) + dr_j(1)*dr_i(2) - dr_j(2)*dr_i(1) - dr_i(1)*dr_j(2); 0];
end
