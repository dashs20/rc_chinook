function dstate = rb_dynamics(state, mass, I, F, tau)
%GENERIC_RB_DYNAMICS_MODEL Computes the time derivative of state for a generic 6-DOF rigid body.
%
%   state = [ x; y; z; vx; vy; vz; e1; e2; e3; e4; wx; wy; wz ]
%       (e1,e2,e3) = quaternion vector part (x,y,z)
%       e4         = quaternion scalar part (w)
%   mass = scalar mass of the rigid body
%   I    = 3x3 inertia matrix
%   F    = 3x1 net force (in inertial frame, unless you rotate it!)
%   tau  = 3x1 net torque (in body frame)
%
%   dstate = [ dx; dy; dz; dvx; dvy; dvz; de1; de2; de3; de4; dwx; dwy; dwz ]

% --- Unpack states ---
x   = state(1);
y   = state(2);
z   = state(3);
vx  = state(4);
vy  = state(5);
vz  = state(6);
e1  = state(7);   % qx
e2  = state(8);   % qy
e3  = state(9);   % qz
e4  = state(10);  % qw
wx  = state(11);
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
