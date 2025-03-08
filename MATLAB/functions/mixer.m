function [a1_percent, a2_percent, f1_percent, f2_percent] = mixer(wx_cmd,wy_cmd,wz_cmd,throttle_cmd,min_throttle_percent)
% this funciton mixes the rate commands into actuator signals.
arguments
    wx_cmd (1,1) double
    wy_cmd (1,1) double
    wz_cmd (1,1) double
    throttle_cmd (1,1) double 
    min_throttle_percent (1,1) double
end

% generate actuator commands
f1_percent = bound(-wy_cmd + throttle_cmd,min_throttle_percent,100);
f2_percent = bound( wy_cmd + throttle_cmd,min_throttle_percent,100);
a1_percent = bound(wx_cmd + wz_cmd,-100,100);
a2_percent = bound(wx_cmd - wz_cmd,-100,100);