% x_trim is the trimmed state,
% u_trim is the trimmed input

% defining trimmed variables
theta_trim  = P.x_trim(8);

delta_e_trim = P.u_trim(1);
delta_a_trim = P.u_trim(2);
delta_r_trim = P.u_trim(3);
delta_t_trim = P.u_trim(4);

Va_trim     = P.y_trim(1);
alpha_trim  = P.y_trim(2);
beta_trim   = P.y_trim(3);

% computing aerodynamic coeffecients
UAV.C_p_p = UAV.Gamma3*UAV.C_ell_p + UAV.Gamma4*UAV.C_n_p;
UAV.C_p_delta_a = UAV.Gamma3*UAV.C_ell_delta_a + UAV.Gamma4*UAV.C_n_delta_a;

% defining roll coeffecients
a_phi1 = -0.5*UAV.rho*Va^2*UAV.S_wing*UAV.b*UAV.C_p_p*(UAV.b/2*Va);
a_phi2 = 0.5*UAV.rho*Va^2*UAV.S_wing*UAV.b*UAV.C_p_delta_a;

% defining pitch coeffecients
a_theta1 = -(UAV.rho*Va^2*UAV.c*UAV.S_wing*UAV.C_m_q*UAV.c)/(4*Va*UAV.Jy);
a_theta2 = -(UAV.rho*Va^2*UAV.c*UAV.S_wing*UAV.C_m_alpha)/(2*UAV.Jy);
a_theta3 =  (UAV.rho*Va^2*UAV.c*UAV.S_wing*UAV.C_m_delta_e)/(2*UAV.Jy);

% computing propeller speed
V_in = UAV.V_max*delta_t_trim;

b1 = (UAV.rho*(UAV.D_prop^4)*UAV.C_Q_1)/(2*pi);
c1 = UAV.rho*(UAV.D_prop^3)*UAV.C_Q_2;
a = (UAV.rho*(UAV.D_prop^5)*UAV.C_Q_0)/(2*pi)^2;
b = b1*Va_trim + (UAV.KQ*UAV.K_V)/UAV.R_motor;
c = c1*Va_trim^2 - (UAV.KQ*V_in)/UAV.R_motor + UAV.KQ*UAV.i0;

Omega_p = (-b + sqrt(b*b-4*a*c))/2*a;

% computing partial derivatives of propeller speed
d_Va_Omega_p = (0.5/a)*(b1 + 0.5/sqrt((b^2-4*a*c)))*(2*b*b1 -4*a*c1);
d_delta_t_Omega_p = (0.5/a)*(0.5*(1/sqrt(b^2-4*a*c))*(-4*a*(-UAV.KQ*UAV.V_max/R)));

% computing partial derivatives of torque
T_p1 = (UAV.rho*(UAV.D_prop^5)*UAV.C_T_0)/(4*pi^2);
T_p2 = (UAV.rho*(UAV.D_prop^4)*UAV.C_T_1)/(2*pi);
T_p3 = UAV.rho*(UAV.D_prop^2)*UAV.C_T_2;

d_Va_T_p = T_p1*2*Omega_p*d_Va_Omega_p + T_p2*Omega_p + T_p2*Va*d_Va_Omega_p + T_p3;
d_delta_t_T_p = T_p1*2*Omega_p*d_delta_t_Omega_p + T_p2*Va*d_delta_t_Omega_p;

% TODO: Check whether these calculations are correct
% defining airspeed coeffecients
a_V1 = 1/UAV.mass*(UAV.rho*Va_trim*UAV.S_wing*(UAV.C_D_0 + UAV.C_D_alpha*alpha_trim*UAV.C_D_delta_e*delta_e_trim)) - 1/UAV.mass*d_Va_T_p;
a_V2 = 1/UAV.mass*d_delta_t_T_p;
a_V3 = UAV.gravity*cos(theta_trim - alpha_trim);

% defining sideslip coeffecients
a_beta1 = -(UAV.rho*Va*UAV.S_wing*UAV.C_Y_beta)/(2*UAV.mass*cos(beta_trim));
a_beta2 =  (UAV.rho*Va*UAV.S_wing*UAV.C_Y_delta_r)/(2*UAV.mass*cos(beta_trim));

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([UAV.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);
