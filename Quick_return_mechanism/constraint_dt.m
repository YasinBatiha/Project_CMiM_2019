function C = constraint_dt(revolute, simple,driving, translation, t, q)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translation);
d_len = length(driving);

n_constr = 2 * r_len + s_len + 2*t_len + d_len;

C = zeros(n_constr, 1);

c_idx = 2 * r_len + s_len;

for d = driving
    C(c_idx + 1) = driving_joint_dt(d.d_k_t, t);
end

end