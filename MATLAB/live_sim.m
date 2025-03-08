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
r0 = [0;0;-1];
v0 = [0;0;0];
q0 = [1;0;0;0];
w0 = [0;0;0];
I0 = vertcat(r0,v0,q0,w0);

%% Define sim parameters
sen_sample_rate = 500; % hz
steps_between_reading = sen_sample_rate/10; % steps
tmax = 10; % seconds
dt = 1/sen_sample_rate;

th = I0.';
cur_time = 0;
fth = zeros(sen_sample_rate*tmax,6);
tvec = linspace(0,tmax,sen_sample_rate*tmax);

% define command th.
wx_cmd = square(2*pi*1*tvec);
wy_cmd = zeros(1,length(tvec));
wz_cmd = zeros(1,length(tvec));

for i = 1:sen_sample_rate*tmax - 1
    % Obtain forces and moments from cmd2fm
    [F,M,cur_forces] = cmd2fm(sin(i/10)*30,sin(i/10)*30,0.1,0.1,r1,r2);
    fth(i,:) = cur_forces;

    % Propagate forward until next sensor reading
    tmp_tvec = linspace(cur_time,cur_time+dt,steps_between_reading);
    I0 = th(i,:);
    dynamics_fun = @(t, state) rb_dynamics(state, mass_kg, I, F, M);
    [t_out, state_out] = ode45(dynamics_fun, tmp_tvec, I0);

    % Update time history of state
    th(i+1,:) = state_out(end,:);

    % Increment current time
    cur_time = cur_time + dt;
end

%% Make Animation
pre_rot = angle2dcm(pi,pi/2,-pi/2);
make_animation(th,r1,r2,[-1,1],[-1,1],[-1,1],"chinook.stl",1/1000,pre_rot,[0.15,0,0],fth*10);
close all