%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,patchcolors] = airframeVFC
% Define the parameters
fuse_l1    = 7;
fuse_l2    = 4;
fuse_l3    = 15;
fuse_w     = 2;

fuse_h     = 2;
wing_l     = 6;
wing_w     = 20;
tail_l     = 3;
tail_h     = 3;
tailwing_w = 10;
tailwing_l = 3;

% Define the vertices (physical location of vertices
V = [...
    fuse_l1                 0               0;...           % point 1
    fuse_l2                 fuse_w/2        fuse_h/2;...    % point 2
    fuse_l2                 -fuse_w/2       fuse_h/2;...    % point 3
    fuse_l2                 fuse_w/2        -fuse_h/2;...   % point 4
    fuse_l2                 -fuse_w/2       -fuse_h/2;...   % point 5
    -fuse_l3                0               0;...           % point 6
    0                       wing_w/2        0;...           % point 7
    -wing_l                 wing_w/2        0;...           % point 8
    -wing_l                 -wing_w/2       0;...           % point 9
    0                       -wing_w/2       0;...           % point 10
    -fuse_l3+tailwing_l     tailwing_w/2    0;...           % point 11
    -fuse_l3                tailwing_w/2    0;...           % point 12
    -fuse_l3                -tailwing_w/2   0;...           % point 13
    -fuse_l3+tailwing_l     -tailwing_w/2   0;...           % point 14
    -fuse_l3+tail_l         0               0;...           % point 15
    -fuse_l3                0               -tail_h;         % point 16
    ];
% define faces as a list of vertices numbered above
F = [...
    1, 3, 4, 1;...     % tip right
    1, 2, 3, 1;...     % tip top
    1, 2, 5, 1;...     % tip left
    1, 5, 4, 1;...     % tip bottom
    3, 4, 6, 3;...     % fuse right
    2, 3, 6, 2;...     % fuse top
    2, 5, 6, 2;...     % fuse left
    5, 4, 6, 5;...     % fuse bottom
    7, 8, 9, 10;...    % wing
    11, 12, 13, 14;... % tail wing
    6, 15, 16, 6;...   % tail
    ];
% define colors for each face
myred     = [1, 0, 0];
mygreen   = [0, 1, 0];
myblue    = [0, 0, 1];
myyellow  = [1,1,0];
mymagenta = [0, 1, 1];

patchcolors = [...
    myred;... % tip
    myred;... % tip
    myred;... % tip
    myred;... % tip
    mygreen;... % fuse
    mygreen;... % fuse
    mygreen;... % fuse
    mygreen;... % fuse
    myblue;... % wing
    myblue;... % tail wing
    myblue;... % tail
    ];
end