function [sys, x0, str, ts] = uav_dynamics(t,x,u,flag,UAV)   
    switch flag
        case 0
            % Initializing the model
            [sys,x0,str,ts] = mdlInitializeSizes(UAV);
        case 1
            % Defining the kinematic motions 
            sys = mdlDerivatives(t,x,u,UAV);
        case 2
            % Updating UAV from kinematic motions
            sys = mdlUpdate(t,x,u);
        case 3
            % Defining outputs after kinematic motions
            sys = mdlOutputs(t,x);
        otherwise
            sys = [];
    end
end

%========================================================================%

function [sys,x0,str,ts] = mdlInitializeSizes(UAV)
    % Sizes structure for the s-function
    sizes = simsizes;

    sizes.NumContStates = 12;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 12;
    sizes.NumInputs = 6;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);

    % Initial state of the system
    x0 = [
        UAV.pn0;
        UAV.pe0;
        UAV.pd0;
        UAV.u0;
        UAV.v0;
        UAV.w0;
        UAV.phi0;
        UAV.theta0;
        UAV.psi0;
        UAV.p0;
        UAV.q0;
        UAV.r0;
        ];

    str = [];

    ts = [0 0];
end

%========================================================================%
% Find the derivatives for the continuous states
function sys = mdlDerivatives(t,x,uu,UAV)
    % Define UAV Parameters
    pn =    x(1);
    pe =    x(2);
    pd =    x(3);
    u =     x(4);
    v =     x(5);
    w =     x(6);
    phi =   x(7);
    theta = x(8);
    psi =   x(9);
    p =     x(10);
    q =     x(11);
    r =     x(12);
    fx = uu(1);
    fy = uu(2);
    fz = uu(3);
    ell = uu(4);
    m = uu(5);
    n = uu(6);
    
    e = Euler2Quaternion(UAV.phi0, UAV.theta0, UAV.psi0);
    e0 = e(1);
    e1 = e(2);
    e2 = e(3);
    e3 = e(4);

    % Find components of Euler angles
    cphi    = cos(phi);
    ctheta  = cos(theta);
    cpsi    = cos(psi);
    sphi    = sin(phi);
    stheta  = sin(theta);
    spsi    = sin(psi);
    
    % Translational velocities and positions
    pndot = ctheta*cpsi*u + (sphi*stheta*cpsi-cphi*spsi)*v +...
        (cphi*stheta*cpsi+sphi*spsi)*w;
    pedot = ctheta*spsi*u + (sphi*stheta*spsi+cphi*cpsi)*v +...
        (cphi*stheta*spsi-sphi*cpsi)*w;
    pddot = -stheta*u + sphi*ctheta*v + cphi*ctheta*w;
    
    % Angular rates and position
    C = [
        1 sin(phi)*tan(theta) cos(phi)*tan(theta)
        0 cos(phi) -sin(phi);
        0 sin(phi)*sec(theta) cos(phi)*sec(theta);
        ];
    angulardot = C*[p; q; r];

    udot = r*v - q*w + (1/UAV.mass)*fx;
    vdot = p*w - r*u + (1/UAV.mass)*fy;
    wdot = q*u - p*v + (1/UAV.mass)*fz;
    
    sqr_mag_e = (norm([e0 e1 e2 e3]))^2;
    edot = e1*[e0; e1; e2; e3];
      
    e0dot = 0.5*(1000*(1-sqr_mag_e)*e0 -p*e1 + -q*e2 + -r*e3);
    e1dot = 0.5*(p*e0 + 1000*(1-sqr_mag_e)*e1 + r*e2 + -q*e3);
    e2dot = 0.5*(q*e0 + -r*e1 + 1000*(1-sqr_mag_e)*e2 + p*e3);
    e3dot = 0.5*(r*e0 + q*e1 + -p*e2 + 1000*(1-sqr_mag_e)*e3);

    pdot = UAV.Gamma1*p*q - UAV.Gamma2*q*r + UAV.Gamma3*ell + UAV.Gamma4*n;
    qdot = UAV.Gamma5*p*r - UAV.Gamma6*(p^2-r^2) + (1/UAV.Jy)*m;
    rdot = UAV.Gamma7*p*q - UAV.Gamma1*q*r + UAV.Gamma4*ell + UAV.Gamma8*n;

    sys = [pndot; pedot; pddot; udot; vdot; wdot; angulardot(1); angulardot(2); angulardot(3);...
        pdot; qdot; rdot];
end

%=========================================================================%
function sys = mdlUpdate(t,x,u)
    sys = [];
end

%=========================================================================%
function sys = mdlOutputs(t,x)
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi    = x(7);
    theta    = x(8);
    psi    = x(9);
    p    = x(10);
    q     = x(11);
    r     = x(12);

    y = [
        pn;...
        pe;...
        pd;...
        u;...
        v;...
        w;...
        phi;...
        theta;...
        psi;...
        p;...
        q;...
        r;...
        ];
    sys = y;
end

function [phi, theta, psi] = Quaternion2Euler(quaternion)
    % converts a quaternion attitude to an euler angle attitude
    e0 = quaternion(1);
    e1 = quaternion(2);
    e2 = quaternion(3);
    e3 = quaternion(4);
    phi = atan2(2*(e0*e1 + e2*e3),(e0^2 + e3^2 - e1^2 - e2^2));
    theta = asin(2*(e0*e2 - e1*e3));
    psi = atan2(2*(e0*e3),(e0^2 + e1^2 - e2^2 - e3^3));
end
























