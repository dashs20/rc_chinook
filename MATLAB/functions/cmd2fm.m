function [F, M, f] = cmd2fm(a1, a2, f1, f2, r1, r2)
%CMD2FM Converts thruster commands (angles & forces) into net force & moment in the body frame.
%
%   Inputs:
%       a1, a2 - Angles of thrusters [degrees], rotation about body-fixed X-axis
%       f1, f2 - Scalar thrust magnitudes
%       r1, r2 - [3x1] position vectors of thruster mount points in body frame
%
%   Outputs:
%       F      - [3x1] net force in body frame
%       M      - [3x1] net moment about the body origin (usually the CG)

    %----------------------------------------------------
    % 1. Compute direction unit vectors for each thruster
    %    Assuming a rotation around the X-axis by angle a,
    %    and that zero angle means pointing along +Z.
    %
    %    R_x(a) * [0; 0; 1] = [ 0; -sin(a); cos(a) ]
    %----------------------------------------------------
    a1 = deg2rad(a1);
    a2 = deg2rad(a2);


    d1 = [ 0; -sin(a1); cos(a1) ];
    d2 = [ 0; -sin(a2); cos(a2) ];
    
    %----------------------------------------------------
    % 2. Compute individual thruster forces in body coords
    %----------------------------------------------------
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
