function [F, M, f] = cmd2fm(a1, a2, f1, f2, r1, r2, max_tilt, max_thrust)
% this function maps commands into forces and moments.
arguments
    a1 (1,1) double % angle (percent) for servo 1.
    a2 (1,1) double % angle (percent) for servo 2.
    f1 (1,1) double % force (percent) for motor 1.
    f2 (1,1) double % force (percent) for motor 2.
    r1 (1,1) double % location of the 1st motor/servo (m)
    r2 (1,1) double % location of the 2nd motor/servo (m)
    max_tilt (1,1) double % maximum tilt (degrees) the servos can achieve.
    max_thrust (1,1) double % maximum thrust (newtons) the motors can create.
end

    % first, convert the angles from percent to radians.
    a1 = a1/100 * deg2rad(max_tilt);
    a2 = a2/100 * deg2rad(max_tilt);

    % compute unit vectors for the force vectors (body coords)
    d1 = [ 0; -sin(a1); cos(a1) ];
    d2 = [ 0; -sin(a2); cos(a2) ];
    
    % second, convert the forces from percent to actual force.
    f1 = f1/100 * max_thrust;
    f2 = f2/100 * max_thrust;

    % multiply the forces by their unit vectors (in body coords)
    F1 = f1 * d1;
    F2 = f2 * d2;
    
    %----------------------------------------------------
    % 3. Sum forces to get net force
    %----------------------------------------------------
    F = F1 + F2;
    
    %----------------------------------------------------
    % 4. Compute moments about the vehicle origin (CG)
    %    Using cross(r, F) for each thruster
    %----------------------------------------------------
    M1 = cross(r1, F1);
    M2 = cross(r2, F2);
    
    M = M1 + M2;

    f = [F1.',F2.'];
end
