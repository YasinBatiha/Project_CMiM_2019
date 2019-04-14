% Quick-return mechanism - kinematic analysis
clc
close all
%% Gometry equations
% given parameters (dimensions in meters)
a = 0.5;
b = 0.12;
c = 0.2;
d = 0.3;
e = 0.1;
angular_v = 1;  % driving angular velocity [rad s^(-1)]
% Calculated initital parameters of the mechanism
phi_1 = acos((a^2+(c+d)^2-b^2)/(2*a*(c+d)));
phi_2 = asin(d/e*sin(phi_1));
phi_3 = pi - (phi_1+phi_2);
phi_4 = phi_3 - pi/2;
phi_5 = asin(a/b*sin(phi_1));
phi_6 = pi/2 - phi_5;
phi_7 = pi - phi_6;
% xg - x-dimension - local to global CS
% xg - y-dimension - local to global CS
% phig - rotation of the local with respect to global CS
% ground
xg_1 = 0;
yg_1 = 0;
phig_1 = 0;
% first link
xg_2 = 0.5*a*sin(phi_1);
yg_2 = 0.5*a*cos(phi_1);
phig_2 = pi/2 - phi_1;
% slider 1
xg_3 = e* cos(phi_4);
yg_3 = d + e*sin(phi_4);
phig_3 = pi/2 - phi_1;
% link 2
xg_4 = b/2* sin(phi_5);
yg_4 = d+c-b/2*cos(phi_5);
phig_4 = phi_7;
% second slider
xg_5 = 0;
yg_5 = c + d;
phig_5 = pi;
% link 3
xg_6 = e/2*cos(phi_4);
yg_6 = d + e/2*sin(phi_4);
phig_6 = phi_4;
% distance between slider and link 2 CG
y_d = yg_3-yg_2;
x_d = xg_3-xg_2;
r_d = sqrt(y_d^2+x_d^2);
%% Coordinates
% ground
q1 = [xg_1;yg_1;phig_1];
% link 1
q2 = [xg_2;yg_2;phig_2];
% slider
q3 = [xg_3;yg_3;phig_3];
% link 2
q4 = [xg_4;yg_4;phig_4];
% slider 2
q5 = [xg_5;yg_5;phig_5];
% link 3
q6 = [xg_6;yg_6;phig_6];
% initial coordinates
q_0 = [q1; q2; q3; q4; q5; q6]; 
% Constraints
%% Revolute joints
% i, j - bodies
% s - coordinates of the joint in local CS
% 1 connects ground and link 1
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-a/2; 0];

% 2 slider 1 and link 3
revolute(2).i = 3;
revolute(2).j = 6;
revolute(2).s_i = [0; 0];
revolute(2).s_j = [e/2; 0];

% 3 ground and link 3
revolute(3).i = 1;
revolute(3).j = 6;
revolute(3).s_i = [0; d];
revolute(3).s_j = [-e/2; 0];

% 4 link 1 and link 2
revolute(4).i = 2;
revolute(4).j = 4;
revolute(4).s_i = [a/2; 0];
revolute(4).s_j = [-b/2; 0];

% 4 link 2 and slider 2
revolute(5).i = 4;
revolute(5).j = 5;
revolute(5).s_i = [b/2; 0];
revolute(5).s_j = [0; 0];
c_idx = 0;
% % Verification
% for r = revolute
%     C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0);
% % end
%% Simple constraints
% i - body
% k - body coordinate (1 = x, 2 = y, 3 = phi)
% c_k - value to be constatnt during analysis
% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% slider - use simple joints instead of translational
simple(4).i = 5;
simple(4).k = 2;
simple(4).c_k = c+d;

simple(5).i = 5;
simple(5).k = 3;
simple(5).c_k = pi;
% % Verification
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end
%% Driving constraints
% i - body
% k - body coordinate (1 = x, 2 = y, 3 = phi)
% d_k - driving function
% d_k_t/d_k_tt - first/second derivative of driving function
%  link 3 driving velocity
driving.i = 6;
driving.k = 3;
driving.d_k = @(t) phi_4 - angular_v .* t;
driving.d_k_t = @(t) angular_v;
driving.d_k_tt = @(t) 0;

% % Verification
% for d = driving
%   C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)
% end
%% Translation joint
% i,j - bodies
% Slider 1 sliding on link 1
translation.i = 2;
translation.j = 3;
% Verification
% for t = translation
%   C_t_i = translation_joint(t.i, t.j, q_0,q_0)
% end
%% Jacobian of our constraints
% Cq = constraint_dq(revolute, simple, driving,translation, 0, q_0, q_0);
%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q)constraint(revolute, simple, driving,translation, t, q,q_0); % Constraint equation
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving,translation, t, q,q_0);    % Jacobian of the constraint equation
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving,translation, t, q);  % first derivative of constraint equation without the velocity terms  
Ctt_fun = @(t, q, dq) constraint_dtt(revolute, simple, driving,translation, q, dq);  % second derivative of constraint equation without the acceleration terms
% Newthon-Raphsons solver for system of non-linear equations for given
% time interval
t_end = 10; dt = 0.001;
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun,Ctt_fun, t_end, q_0, dt);
%% Plotting results
% Positions
figure
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    Q(:, 13), Q(:, 14), ...
    Q(:, 16), Q(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Positions')
xlabel('x [m]')
ylabel('y [m]')
%% Some verification plots
% Velocities
figure
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    QP(:, 13), QP(:, 14), ...
    QP(:, 16), QP(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Velocities')
xlabel('v_x [m\cdot s^{-1}]')
ylabel('v_y [m\cdot s^{-1}]')
% Accelerations
figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    QPP(:, 13), QPP(:, 14), ...
    QPP(:, 16), QPP(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Accelerations')
xlabel('a_x [m\cdot s^{-2}]')
ylabel('a_y [m\cdot s^{-2}]')
% % Checking the velocities and accelerations by numerical derivation of
% % positions
% QP_ref = num_der(T, Q);
% figure
% plot(QP_ref(:, 4), QP_ref(:, 5), ...
%     QP_ref(:, 7), QP_ref(:, 8), ...
%     QP_ref(:, 10), QP_ref(:, 11), ...
%     QP_ref(:, 13), QP_ref(:, 14), ...
%     QP_ref(:, 16), QP_ref(:, 17), ...
%     0, 0, '*', 'LineWidth', 1);
% axis equal
% legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
% title('Velocities _ ref')
% 
% QPP_ref = num_der(T, QP_ref);
% figure
% plot(QPP_ref(:, 4), QPP_ref(:, 5), ...
%     QPP_ref(:, 7), QPP_ref(:, 8), ...
%     QPP_ref(:, 10), QPP_ref(:, 11), ...
%     QPP_ref(:, 13), QPP_ref(:, 14), ...
%     QPP_ref(:, 16), QPP_ref(:, 17), ...
%     0, 0, '*', 'LineWidth', 1);
% axis equal
% legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
% title('Accelerations _ ref')
