function C = constraint_dt(revolute, simple, driving,translation, t, q)

r_len = length(revolute);
s_len = length(simple);
d_len = length(driving);
t_len = length(translation);

n_constr = 2 * r_len + s_len + d_len+2*t_len;

C = zeros(n_constr, 1);

c_idx = 2 * r_len + s_len;

for d = driving
    c_idx = c_idx + 1;
    C(c_idx) = driving_joint_dt(d.d_k_t, t);
end