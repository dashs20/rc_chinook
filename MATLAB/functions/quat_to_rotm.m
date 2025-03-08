function R = quat_to_rotm(q)
%QUAT_TO_ROTM Convert quaternion [qw, qx, qy, qz] to a 3x3 rotation matrix.

arguments
    q (1,4) double % quaternion (e4 = scalar part)
end

    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    R = [ 1 - 2*qy^2 - 2*qz^2,  2*qx*qy - 2*qw*qz,  2*qx*qz + 2*qw*qy;
          2*qx*qy + 2*qw*qz,  1 - 2*qx^2 - 2*qz^2,  2*qy*qz - 2*qw*qx;
          2*qx*qz - 2*qw*qy,  2*qy*qz + 2*qw*qx,  1 - 2*qx^2 - 2*qy^2 ];
end
