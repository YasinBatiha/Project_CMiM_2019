function C = constraint_dtt_dyn(revolute, simple,translation, q, dq)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translation);

n_constr = 2 * r_len + s_len + 2 * t_len;

C = zeros(n_constr, 1);

c_idx = 0;

for r = revolute
        C(c_idx + (1:2)) = revolute_joint_dtt(r.i, r.j, r.s_i, r.s_j, dq, q);
        c_idx = c_idx + 2;
end

c_idx = c_idx + s_len;

for t = translation
    C(c_idx + (1:2)) = translation_joint_dtt(t.i, t.j, dq);
end

end