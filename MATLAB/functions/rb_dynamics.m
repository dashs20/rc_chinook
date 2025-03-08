function dstate = rb_dynamics(state, mass, I, F, tau, opt)
%GENERIC_RB_DYNAMICS_MODEL Computes the time derivative of state for a generic 6-DOF rigid body.

arguments
    state (13,1) double % current state.
    mass (1,1) double % mass (kg)
    I (3,3) double % inertia matrix. Assumed to have no cross terms.
    F (3,1) double % Force vector (N)
    tau (3,1) double % Moment vector (Nm)
    opt.gravity (1,1) = 0; % Turn on gravity?
end

% --- Unpack states ---
vx  = state(4);   % velocity (m/s)
vy  = state(5);
vz  = state(6);
e1  = state(7);   % qx
e2  = state(8);   % qy
e3  = state(9);   % qz
e4  = state(10);  % qw
wx  = state(11); % angular velocity (rad/s)
wy  = state(12);
wz  = state(13);

% --- Translational Kinematics ---
dx = vx;
dy = vy;
dz = vz;

% --- Translational Dynamics ---
% Assume F is already expressed in inertial coordinates:
dvx = F(1) / mass;
dvy = F(2) / mass;

if(opt.gravity)
    F(3) = F(3) - mass * 9.81;
end

dvz = F(3) / mass;

% Standard quaternion derivative formula (e4 = scalar part)
de1 =  0.5 * ( + wx*e4 + wy*e3 - wz*e2 );
de2 =  0.5 * ( - wx*e3 + wy*e4 + wz*e1 );
de3 =  0.5 * ( + wx*e2 - wy*e1 + wz*e4 );
de4 =  0.5 * ( - wx*e1 - wy*e2 - wz*e3 );

% --- Rotational Dynamics: I * dw = tau - w x (I*w) ---
dwx = (tau(1) + (I(2,2) - I(3,3)) * wy * wz) / I(1,1);
dwy = (tau(2) + (I(3,3) - I(1,1)) * wx * wz) / I(2,2);
dwz = (tau(3) + (I(1,1) - I(2,2)) * wx * wy) / I(3,3);


% --- Assemble dstate ---
dstate = [
    dx
    dy
    dz
    dvx
    dvy
    dvz
    de1
    de2
    de3
    de4
    dwx
    dwy
    dwz
];
end
