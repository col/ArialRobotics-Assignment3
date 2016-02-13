function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% =================== Your code goes here ===================

m = params.mass;
g = params.gravity;
I = params.I;
invI = params.invI;
arm_length = params.arm_length;
%disp(['m:', num2str(m), ', g:', num2str(g)]);

x = state.pos(1);
y = state.pos(2);
z = state.pos(3);
%disp(['x:', num2str(x), ', y:', num2str(y), ', z:', num2str(z)]);

x_dot = state.vel(1);
y_dot = state.vel(2);
z_dot = state.vel(3);
%disp(['x_dot:', num2str(x_dot), ', y_dot:', num2str(y_dot), ', z_dot:', num2str(z_dot)]);

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
%disp(['phi:', num2str(phi), ', theta:', num2str(theta), ', psi:', num2str(psi)]);

p = state.omega(1);
q = state.omega(2);
r = state.omega(3);
%disp(['p:', num2str(p), ', q:', num2str(q), ', r:', num2str(r)]);

des_x = des_state.pos(1);
des_y = des_state.pos(2);
des_z = des_state.pos(3);
%disp(['Desired - x:', num2str(des_x), ', y:', num2str(des_y), ', z:', num2str(des_z)]);

des_x_dot = des_state.vel(1);
des_y_dot = des_state.vel(2);
des_z_dot = des_state.vel(3);
%disp(['Desired - x_dot:', num2str(des_x_dot), ', y_dot:', num2str(des_y_dot), ', z_dot:', num2str(des_z_dot)]);

des_x_ddot = des_state.acc(1);
des_y_ddot = des_state.acc(2);
des_z_ddot = des_state.acc(3);
%disp(['Desired - x_ddot:', num2str(des_x_ddot), ', y_ddot:', num2str(des_y_ddot), ', z_ddot:', num2str(des_z_ddot)]);

des_yaw = des_state.yaw;
des_yaw_dot = des_state.yawdot;
%disp(['Desired - yaw:', num2str(des_yaw), ', yaw_dot:', num2str(des_yaw_dot)]);

% PD Values
kpz = 0.1;
kdz = 0;

kpx = 0.1;
kdx = 0;

kpy = 0.1;
kdy = 0;

kpphi = 0.1;
kdphi = 0;

kptheta = 0.1;
kdtheta = 0;

kppsi = 0.1;
kdpsi = 0;

% Thurst
F = m * (g + des_z_ddot + kdz * (des_z_dot-z_dot) + kpz * (des_z-z));

% Moment
r1c = des_x_ddot + kdx * (des_x_dot - x_dot) + kpx * (des_x - x);
r2c = des_y_ddot + kdy * (des_y_dot - y_dot) + kpy * (des_y - y);

des_phi = 1/g * (r1c * des_yaw - r2c);
des_theta = 1/g * (r1c + r2c*des_yaw);

u_phi = kpphi * (des_phi-phi) + kdphi * (0-p);
u_theta = kptheta*(des_theta-theta) + kdtheta*(0-q);
u_psi = kppsi*(des_yaw-psi) + kdpsi*(des_yaw_dot-r);

M = [u_phi;u_theta;u_psi];

% =================== Your code ends here ===================

end
