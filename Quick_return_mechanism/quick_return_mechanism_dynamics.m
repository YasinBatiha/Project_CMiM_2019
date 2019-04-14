% Quick-return mechanism - dynamic analysis
quick_return_mechanism_kinematics
% close all
 %% Defining all bodies
 % m - mass in kg
 % Ic - moment of inertia in kg m^2 
 % q - vector of initial coordinates
% ground 
body(1).m = 2;
body(1).Ic = body(1).m *  12;
body(1).q = q1;

% link 1
L_2 = 0.5;
body(2).m = L_2 * 10;
body(2).Ic = body(2).m * L_2^2 / 12;
body(2).q = q2;

% slider
body(3).m = 2;
body(3).Ic = body(3).m * 1.2;
body(3).q = q3;

% link 2 
L_4 = 0.12;
body(4).m = L_4 * 10;
body(4).Ic = body(4).m * L_4^2 / 12;
body(4).q = q4;

% slider 2
body(5).m = 2;
body(5).Ic = body(5).m * 1.2;
body(5).q = q5;

% link 3 
L_6 = 0.1;
body(6).m = L_6 * 10;
body(6).Ic = body(6).m * L_4^2 / 12;
body(6).q = q6;

% Vector of initial coordinates
q_0_dyn = system_coordinates(body);
%% Definition of loading
% f - force vector in global CS [N]
% i - body
% u_i - force position
% t - torque [N m]
sforce.f = [0; 0];
sforce.i = 6;
sforce.u_i = [0; 0];
sforce.t = 5;

grav = [0; -9.81]; % gravitational acceleration in global CS
%% Definition of vectors and matrixes needed for equation of motion
M = mass_matrix(body);  % mass matrix
F_dyn = @(q)force_vector(grav, sforce, body, q);    % force vector
Cq_fun_dyn = @(t, q)constraint_dq_dyn(revolute, simple,translation, t, q, q_0); % Jacobian matrix
Ctt_fun_dyn = @(t,q,dq)constraint_dtt_dyn(revolute, simple,translation, q, dq); % g - vector
C_fun_dyn = @(t,q)constraint_dyn(revolute, simple, translation, t, q, q_0); % vector of constraint equation
%% Baumgarte method modification
% M_full = @(t, q) [M, Cq_fun_dyn(t,q)'; Cq_fun_dyn(t,q), zeros(17,17)];
% % Full M matrix
beta = 10;  % Baumgarte method parameter
% baumgarte g_hat vector
g_hat = @(t, q, dq) Ctt_fun_dyn(t,q,dq) - beta^2 * C_fun_dyn(t, q);
% F_full = @(t, q, dq) [F_dyn(q); g_hat(t, q, dq)]; % full force vector
%% Euler Crommer solution of EoM
% Function for EC solution of EoM
acc_f_2nd = @(t, q, dq) inv(M)*F_dyn(q)+inv(M)*Cq_fun_dyn(t, q)'*inv((Cq_fun_dyn(t, q)*inv(M)*Cq_fun_dyn(t, q)'))*(g_hat(t, q, dq)-Cq_fun_dyn(t, q)*inv(M)*F_dyn(q));
t_end = 2; dt = 0.001;
[t, u, v] = EulerCromer(acc_f_2nd, t_end, q_0_dyn, zeros(size(q_0_dyn)), dt);
%% Plots for EC
% Positions
figure
plot(u(:, 4), u(:, 5), ...
    u(:, 7), u(:, 8), ...
    u(:, 10), u(:, 11), ...
    u(:, 13), u(:, 14), ...
    u(:, 16), u(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Positions - EC')
xlabel('x [m]')
ylabel('y [m]')
% Velocities
figure
plot(v(:, 4), v(:, 5), ...
    v(:, 7), v(:, 8), ...
    v(:, 10), v(:, 11), ...
    v(:, 13), v(:, 14), ...
    v(:, 16), v(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Velocities - EC')
xlabel('v_x [m\cdot s^{-1}]')
ylabel('v_y [m\cdot s^{-1}]')

%% Ode45 solution of EoM
t_end = 2;
[T, Y] = ode45(@(t, q)accODE(t, q, M, F_dyn, Cq_fun_dyn, g_hat),[0, t_end],[q_0_dyn; zeros(size(q_0_dyn))]);
%% Plots for ode45
% Positions
figure
plot(Y(:, 4), Y(:, 5),...
    Y(:, 7), Y(:, 8),...
    Y(:, 10), Y(:, 11),...
    Y(:, 13), Y(:, 14),...
    Y(:, 16), Y(:, 17), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Positions - ode45')
xlabel('x [m]')
ylabel('y [m]')
% Velocities
figure
plot(Y(:, 22), Y(:, 23),...
    Y(:, 25), Y(:, 26),...
    Y(:, 28), Y(:, 29),...
    Y(:, 31), Y(:, 32),...
    Y(:, 34), Y(:, 35), ...
    0, 0, '*', 'LineWidth', 1);
axis equal
legend('link 1','slider 1', 'link 2', 'slider 2', 'link 3')
title('Velocities - ode45')
xlabel('v_x [m\cdot s^{-1}]')
ylabel('v_y [m\cdot s^{-1}]')
% function definition for the ode45 solver
function ddq = accODE(t, q, M, F_dyn, Cq_fun_dyn, g_hat)
ddq=[q(19:36); inv(M)*F_dyn(q(1:18))+inv(M)*Cq_fun_dyn(t, q(1:18))'*inv((Cq_fun_dyn(t, q(1:18))*inv(M)*Cq_fun_dyn(t, q(1:18))'))*(g_hat(t, q(1:18), q(19:36))-Cq_fun_dyn(t, q(1:18))*inv(M)*F_dyn(q(1:18)))];
end