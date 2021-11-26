function drawAircraft(uu)
% process inputs to functions
pn       = uu(1);       % inertial North position
pe       = uu(2);       % inertial East position
pd       = uu(3);       % inertial Down position
u        = uu(4);       % ground velocity x-axis
v        = uu(5);       % ground velocity y-axis
w        = uu(6);       % ground velocity z-axis
phi      = uu(7);       % roll angle
theta    = uu(8);       % pitch angle
psi      = uu(9);       % yaw angle
p        = uu(10);      % roll rate
q        = uu(11);      % pitch rate
r        = uu(12);      % yaw rate
t        = uu(13);      % time

% define persistent variables
persistent box_handle;

if t==0
    figure(1); clf;
    box_handle = drawAirframeVFC(pn, pe, pd, phi, theta, psi, []);
    title('Box')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    view(32,47)  % set the view angle for figure
    axis([-1000,1000,-1000,1000,-1000, 1000]);
    grid on
    
    % at every other time step, redraw box
else
    drawAirframeVFC(pn, pe, pd, phi, theta, psi, box_handle);
end