function [a1, a2, f1, f2] = mixer(wx_cmd_percent,wy_cmd_percent,wz_cmd_percent,throttle_percent)
% this funciton mixes the rate commands into actuator signals.
arguments
    wx_cmd_percent (1,1) double
    wy_cmd_percent (1,1) double
    wz_cmd_percent (1,1) double
    throttle_percent (1,1) double 
end

f1 = -wy_cmd_percent + throttle_percent
f2 =  wy_cmd_percent + throttle_percent