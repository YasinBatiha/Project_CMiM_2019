function Cq = constraint_dq(revolute, simple, driving,translation, t, q, q_int)

r_len = length(revolute);
s_len = length(simple);
d_len = length(driving);
t_len = length(translation);

n_constr = 2 * r_len + s_len + d_len+2*t_len;

Cq = zeros(n_constr, length(q));

c_idx = 0;
for r = revolute
    Cq(c_idx + (1:2), [body_idx(r.i), body_idx(r.j)]) = ...
        revolute_joint_dq(r.i, r.j, r.s_i, r.s_j, q);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    Cq(c_idx, body_idx(s.i)) = simple_joint_dq(s.k);
end

for d = driving
    c_idx = c_idx + 1;
    Cq(c_idx, body_idx(d.i)) = driving_joint_dq(d.k);
end
for t = translation
    Cq(c_idx + (1:2),[body_idx(t.i), body_idx(t.j)]) = translation_joint_dq(t.i, t.j, q, q_int);
end
end
