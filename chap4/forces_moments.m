 % forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, UAV)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % rotation matrix from vehicle frame to body frame
    R = R_b_v(phi,theta,psi);
    
    % compute wind velocity in body frame
    V_b_w = R*[w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg]; %[u_w; v_w; w_w]
    
    % vehicle ground velocity in body frame
    V_b_g = [u; v; w];                               %[u; v; w]
    
    % vehicle air velocity in body frame
    V_b_a = V_b_g - V_b_w;                           %[u_r; v_r; w_r]
    
    % compute air data
    Va = norm(V_b_a);
    alpha = atan(V_b_a(3)/V_b_a(1));                %atan(w_r/u_r)
    beta = asin(V_b_a(2)/Va);                       %asin(v_r/Va)
    
    % compute wind vector in NED
    wind = V_b_w + R'*[u_wg; v_wg; w_wg];
    w_n = wind(1);
    w_e = wind(2);
    w_d = wind(3);
    
    % compute propeller thrust and torque
    V_in = UAV.V_max*delta_t;
    
    a = (UAV.rho*(UAV.D_prop^5)*UAV.C_Q_0)/(2*pi)^2;
    b = (UAV.rho*(UAV.D_prop^4)*UAV.C_Q_1*Va)/(2*pi) + (UAV.KQ*UAV.KQ)/UAV.R_motor;
    c = UAV.rho*(UAV.D_prop^3)*UAV.C_Q_2*Va^2 - (UAV.KQ*V_in)/UAV.R_motor + UAV.KQ*UAV.i0;
    
    Omega_p = (-b + sqrt(b^2-4*a*c))/2*a;
    
    T_p = ((UAV.rho*(UAV.D_prop^4)*UAV.C_T_0)/4*pi^2)*Omega_p^2 + ...
        ((UAV.rho*(UAV.D_prop^3)*UAV.C_T_1*Va)/(2*pi))*Omega_p + ...
        (UAV.rho*(UAV.D_prop^2)*UAV.C_T_2*Va^2);
    Q_p = ((UAV.rho*(UAV.D_prop^5)*UAV.C_Q_0)/4*pi^2)*Omega_p^2 + ...
        ((UAV.rho*(UAV.D_prop^4)*UAV.C_Q_1*Va)/(2*pi))*Omega_p + ...
        (UAV.rho*(UAV.D_prop^3)*UAV.C_Q_2*Va^2);
    Q_p = 0;
    
    %Nonlinear aerodynamics
    M1 = -UAV.M*(alpha-UAV.alpha0);
    M2 = UAV.M*(alpha+UAV.alpha0);
    sigma = (1+exp(M1)+exp(M2))/((1+exp(M1))*(1+exp(M2)));
    
    C_L_alpha = (pi*UAV.AR)/(1+sqrt(1+(UAV.AR^2)/4));
    C_L = (1-sigma)*(UAV.C_L_0 + C_L_alpha*alpha) + sigma*(2*sign(alpha)*(sin(alpha)^2)*cos(alpha));
    C_D = UAV.C_D_p + ((UAV.C_L_0 + C_L_alpha*alpha)^2)/(pi*UAV.e*UAV.AR);
    
%     % define longitudinal coeff using linear models
%     C_L = UAV.C_L_0 + UAV.C_L_alpha*alpha;
%     C_D = UAV.C_D_0 + UAV.C_D_alpha*alpha;
    
    % compute lift and drag coeff
    C_X         = -C_D*cos(alpha) + C_L*sin(alpha);
    C_X_q       = -UAV.C_D_q*cos(alpha) + UAV.C_L_q*sin(alpha);
    C_X_delta_e = -UAV.C_D_delta_e*cos(alpha) + UAV.C_L_delta_e*sin(alpha);
    C_Z         = -C_D*sin(alpha) - C_L*cos(alpha);
    C_Z_q       = -UAV.C_D_q*sin(alpha) - UAV.C_L_q*cos(alpha);
    C_Z_delta_e = -UAV.C_D_delta_e*sin(alpha) - UAV.C_L_delta_e*cos(alpha);
    
    % calculating coeffecients from UAV struct data
    C_X_q_alpha = (C_X_q*UAV.c)/(2*Va);
    C_Y_p       = (UAV.C_Y_p*UAV.b)/(2*Va);
    C_Y_r       = (UAV.C_Y_r*UAV.b)/(2*Va);
    C_Z_q_alpha = (C_Z_q*UAV.c)/(2*Va);
    
    % compute external forces on UAV
    f_grav = [
        -UAV.mass*UAV.gravity*sin(theta);
        UAV.mass*UAV.gravity*cos(theta)*sin(phi);
        UAV.mass*UAV.gravity*cos(theta)*cos(phi);
        ];
    
    f_lift = [
        C_X + C_X_q_alpha*q + C_X_delta_e*delta_e;
        UAV.C_Y_0 + UAV.C_Y_beta*beta + C_Y_p*p + C_Y_r*r + UAV.C_Y_delta_a*delta_a + UAV.C_Y_delta_r*delta_r;
        C_Z + C_Z_q_alpha*q + C_Z_delta_e*delta_e;
        ];
    Force = f_grav + 0.5*UAV.rho*Va*Va*UAV.S_prop*f_lift + [T_p; 0; 0];
    
%     f_thrust = [(UAV.k_motor*delta_t)^2 - Va^2; 0; 0];
%     Force = f_grav + 0.5*rho*Va*Va*UAV.S_wing*f_lift + 0.5*UAV.rho*UAV.S_prop*UAV.C_prop*f_thrust;
    
    % Pulling coeffecients from UAV struct
    C_ell_p = (UAV.C_ell_p*UAV.b)/(2*Va);
    C_ell_r = (UAV.C_ell_r*UAV.b)/(2*Va);
    C_m_q = (UAV.C_m_q*UAV.c)/(2*Va);
    C_n_p = (UAV.C_n_p*UAV.b)/(2*Va);
    C_n_r = (UAV.C_n_r*UAV.b)/(2*Va);
    
    % compute external torque on UAV
    t_lift = [
        UAV.b*(UAV.C_ell_beta*beta + C_ell_p*p + C_ell_r*r + UAV.C_ell_delta_a*delta_a + UAV.C_ell_delta_r*delta_r);
        UAV.c*(UAV.C_m_0 + UAV.C_m_alpha*alpha + C_m_q*q + UAV.C_m_delta_e*delta_e);
        UAV.b*(UAV.C_n_0 + UAV.C_n_beta*beta + C_n_p*p + C_n_r*r + UAV.C_n_delta_a*delta_a + UAV.C_n_delta_r*delta_r);
        ];
    Torque = 0.5*UAV.rho*Va*Va*UAV.S_prop*t_lift + [Q_p; 0; 0];
    
%     t_thrust = [-UAV.k_T_p*(UAV.k_Omega*delta_t)^2; 0; 0];
%     Torque = 0.5*rho*Va*Va*UAV.S_wing*t_lift + t_thrust;
   
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end



