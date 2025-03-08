close all
clear
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

%% define PID variables
ints = zeros(3,1); % integrals of error
prev_error = zeros(3,1); % previous error
cur_error = zeros(3,1); % current error

Ps = -[2;2;-2]; % kP terms
Is = -[0;0;0]; % kI terms
Ds = -[0;0;0]; % kD terms

%% Define sim parameters
sen_sample_rate = 500; % hz
steps_between_reading = sen_sample_rate/10; % steps
tmax = 10; % seconds
dt = 1/sen_sample_rate;

th = I0.';
cur_time = 0;
fth = zeros(sen_sample_rate*tmax,6);
ath = zeros(sen_sample_rate*tmax,2);
tvec = linspace(0,tmax,sen_sample_rate*tmax);

% define command th.
wx_cmd_degps = square(2*pi/3*tvec) * 30;
wy_cmd_degps = square(2*pi/3*tvec) * 0;
wz_cmd_degps = square(2*pi/3*tvec) * 0;

for i = 1:sen_sample_rate*tmax - 1
    % define pilot commands
    throttle = 30;
    yaw_cmd_pilot = deg2rad(wz_cmd_degps(i));
    pitch_cmd_pilot = deg2rad(wy_cmd_degps(i));
    roll_cmd_pilot = deg2rad(wx_cmd_degps(i));

    % compute error
    cur_error(1) = th(i,11) - roll_cmd_pilot;
    cur_error(2) = th(i,12) - pitch_cmd_pilot;
    cur_error(3) = th(i,13) - yaw_cmd_pilot;

    % compute PID command
    cmd_controller = Ps .* cur_error + Is .* ints + Ds .* (cur_error - prev_error);

    % increment integral and save previous error
    prev_error = cur_error;
    ints = ints + cur_error;
    
    [a1_cmd, a2_cmd, f1_cmd, f2_cmd] = ...
        mixer(cmd_controller(1),cmd_controller(2),cmd_controller(3),throttle,3);

    % Obtain body-fixed forces and moments from cmd2fm
    [F,M,cur_forces,cur_angles] = cmd2fm(a1_cmd, a2_cmd, f1_cmd, f2_cmd,...
        r1, r2, 60, 0.5*9.81);
    fth(i,:) = cur_forces;
    ath(i,:) = cur_angles;

    % Convert force to inertial using current attitude
    cur_dcm = quat_to_rotm(th(i,7:10));
    F = cur_dcm*F;

    % Propagate forward until next sensor reading
    tmp_tvec = linspace(cur_time,cur_time+dt,steps_between_reading);
    I0 = th(i,:);
    dynamics_fun = @(t, state) rb_dynamics(state, mass_kg, I, F, M, "gravity",true);
    [t_out, state_out] = ode45(dynamics_fun, tmp_tvec, I0);

    % Update time history of state
    th(i+1,:) = state_out(end,:);

    % Increment current time
    cur_time = cur_time + dt;
end

%% Plots
subplot(3,1,1)
plot(tvec,th(:,11))
hold on
plot(tvec,deg2rad(wx_cmd_degps))

subplot(3,1,2)
plot(tvec,th(:,12))
hold on
plot(tvec,deg2rad(wy_cmd_degps))

subplot(3,1,3)
plot(tvec,th(:,13))
hold on
plot(tvec,deg2rad(wz_cmd_degps))
sgtitle("\omega Commands vs. Plant")

figure()

plot(tvec,rad2deg(ath(:,1)))
hold on
plot(tvec,rad2deg(ath(:,2)))

title("Servo angles")

figure()

f1_norm = sqrt(fth(:,1).^2 + fth(:,2).^2 + fth(:,3).^2);
f2_norm = sqrt(fth(:,4).^2 + fth(:,5).^2 + fth(:,6).^2);

plot(tvec,f1_norm)
hold on
plot(tvec,f2_norm)

title("Motor Forces")

%% Make Animation
pre_rot = angle2dcm(pi,pi/2,-pi/2);
make_animation(th,r1,r2,[-1,1],[-1,1],[-1,1],"chinook.stl",1/1000,pre_rot,[0.15,0,0],fth/max(max(fth)),20,1);