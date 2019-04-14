function C = constraint_dyn(revolute, simple, translation, t, q,q_int)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translation);

n_constr = 2 * r_len + s_len + 2*t_len;

C = zeros(n_constr, 1);

c_idx = 0;
for r = revolute
    C(c_idx + (1:2)) = revolute_joint(r.i, r.j, r.s_i, r.s_j, q);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    C(c_idx) = simple_joint(s.i, s.k, s.c_k, q);
end

for t = translation
    C(c_idx + (1:2)) = translation_joint(t.i, t.j, q, q_int);
end
end