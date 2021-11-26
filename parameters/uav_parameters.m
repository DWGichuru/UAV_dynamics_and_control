% initial conditions
UAV.pn0    = 0;     % initial North position
UAV.pe0    = 0;     % initial East position
UAV.pd0    = -10;   % initial Down position (negative altitude)
UAV.u0     = 0;     % initial velocity along body x-axis
UAV.v0     = 0;     % initial velocity along body y-axis
UAV.w0     = 0;     % initial velocity along body z-axis
UAV.phi0   = 0;     % initial roll angle
UAV.theta0 = 0;     % initial pitch angle
UAV.psi0   = 0;     % initial yaw angle
e = Euler2Quaternion(UAV.phi0, UAV.theta0, UAV.psi0);
UAV.e0     = e(1);  % initial quaternion
UAV.e1     = e(2);
UAV.e2     = e(3);
UAV.e3     = e(4);
UAV.p0     = 0;     % initial body frame roll rate
UAV.q0     = 0;     % initial body frame pitch rate
UAV.r0     = 0;     % initial body frame yaw rate

%physical parameters of airframe
UAV.gravity = 9.81;
UAV.mass = 11.0;
UAV.Jx   = 0.824;
UAV.Jy   = 1.135;
UAV.Jz   = 1.759;
UAV.Jxz  = 0.120;
UAV.S_wing        = 0.55;
UAV.b             = 2.90;
UAV.c             = 0.19;
UAV.S_prop        = 0.2027;
UAV.C_prop        = 1.0;
UAV.rho           = 1.2682;
UAV.e             = 0.9;
UAV.AR            = UAV.b^2/UAV.S_wing;

% Gamma parameters from uavbook page 36
UAV.Gamma  = UAV.Jx*UAV.Jz-UAV.Jxz^2;
UAV.Gamma1 = (UAV.Jxz*(UAV.Jx-UAV.Jy+UAV.Jz))/UAV.Gamma;
UAV.Gamma2 = (UAV.Jz*(UAV.Jz-UAV.Jy)+UAV.Jxz*UAV.Jxz)/UAV.Gamma;
UAV.Gamma3 = UAV.Jz/UAV.Gamma;
UAV.Gamma4 = UAV.Jxz/UAV.Gamma;
UAV.Gamma5 = (UAV.Jz-UAV.Jx)/UAV.Jy;
UAV.Gamma6 = UAV.Jxz/UAV.Jy;
UAV.Gamma7 = (UAV.Jx*(UAV.Jx-UAV.Jy)+UAV.Jxz*UAV.Jxz)/UAV.Gamma;
UAV.Gamma8 = UAV.Jx/UAV.Gamma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
UAV.C_L_0         = 0.23;
UAV.C_D_0         = 0.043;
UAV.C_m_0         = 0.0135;
UAV.C_L_alpha     = 5.61;
UAV.C_D_alpha     = 0.030;
UAV.C_m_alpha     = -2.74;
UAV.C_L_q         = 7.95;
UAV.C_D_q         = 0.0;
UAV.C_m_q         = -38.21;
UAV.C_L_delta_e   = 0.13;
UAV.C_D_delta_e   = 0.0135;
UAV.C_m_delta_e   = -0.99;
UAV.M             = 50;
UAV.alpha0        = 0.47;
UAV.epsilon       = 0.16;
UAV.C_D_p         = 0.0;

UAV.C_Y_0         = 0.0;
UAV.C_ell_0       = 0.0;
UAV.C_n_0         = 0.0;
UAV.C_Y_beta      = -0.98;
UAV.C_ell_beta    = -0.13;
UAV.C_n_beta      = 0.073;
UAV.C_Y_p         = 0.0;
UAV.C_ell_p       = -0.51;
UAV.C_n_p         = -0.069;
UAV.C_Y_r         = 0.0;
UAV.C_ell_r       = 0.25;
UAV.C_n_r         = -0.095;
UAV.C_Y_delta_a   = 0.075;
UAV.C_ell_delta_a = 0.17;
UAV.C_n_delta_a   = -0.011;
UAV.C_Y_delta_r   = 0.19;
UAV.C_ell_delta_r = 0.0024;
UAV.C_n_delta_r   = -0.069;

% Parameters for propulsion thrust and torque models
UAV.D_prop = 0.508;     % prop diameter in m

% Motor parameters
UAV.K_V = 145;                    % from datasheet RPM/V
UAV.KQ = (1/UAV.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
UAV.R_motor = 0.042;              % ohms
UAV.i0 = 1.5;                     % no-load (zero-torque) current (A)

UAV.k_motor = 80;
UAV.k_T_p = 0;
UAV.k_Omega = 0;

% Inputs
UAV.ncells = 12;
UAV.V_max = 3.7*UAV.ncells;       % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
UAV.C_Q_2 = -0.01664;
UAV.C_Q_1 = 0.004970;
UAV.C_Q_0 = 0.005230;

UAV.C_T_2 = -0.1079;
UAV.C_T_1 = -0.06044;
UAV.C_T_0 = 0.09357;

function quaternions = Euler2Quaternion(phi,theta,psi)
    e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(psi/2);
    e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(psi/2);
    e2 = cos(psi/2)*sin(theta/2)*cos(psi/2) + sin(psi/2)*cos(theta/2)*sin(psi/2);
    e3 = sin(psi/2)*cos(theta/2)*cos(psi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);
    quaternions = [e0; e1; e2; e3];
end

