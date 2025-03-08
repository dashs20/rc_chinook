%% define vehicle properties
fuse_len_m = 0.3; % m
fuse_rad_m = 0.05; % m
mass_kg = 0.3; % kg
r1 = [0.15,0,0.1]; % positions of thrusters
r2 = [-0.15,0,0.1];

% assume vehicle is a cylinder 
Ixx = 1/2*mass_kg*fuse_rad_m^2;
Iyyzz = 1/4*mass_kg*fuse_rad_m^2 + 1/12*mass_kg*fuse_len_m^2;
I = diag([Ixx,Iyyzz,Iyyzz]);

%% define initial condition
r0 = [0;0;0];
v0 = [0;0;0];
q0 = [1;0;0;0];
w0 = [0;0;0];
I0 = vertcat(r0,v0,q0,w0);

% generate preliminary forces/moments for demo
[F,M,f] = cmd2fm(30,-30,10,10,r1,r2,30,500/1000*9.81);

% run a 10 second simulation
tmax = 3;
tvec = linspace(0,tmax,tmax*100); % 100 hz

dynamics_fun = @(t, state) rb_dynamics(state, mass_kg, I, F, M);
[t_out, state_out] = ode45(dynamics_fun, tvec, I0);

% make animation
pre_rot = angle2dcm(pi,pi/2,-pi/2);
fth = zeros(length(t_out),6);
for i = 1:length(t_out)
    fth(i,:) = f;
end
make_animation(state_out,r1,r2,[-0.5,0.5],[-0.5,0.5],[0,1],"chinook.stl",1/1000,pre_rot,[0.15,0,0],fth*10);