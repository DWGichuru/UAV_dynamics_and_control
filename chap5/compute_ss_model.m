% x_trim is the trimmed state,
% u_trim is the trimmed input
  
x_trim = P.x_trim;
u_trim = P.u_trim;
  
[A,B,C,D]=linmod('uavsim_trim',x_trim,u_trim);

E1 = [
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    ];
E2 = [
    0 1 0 0;
    0 0 1 0;
    ];

E3 = [
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 -1 0 0 0 0 0 0 0 0 0;
    ];
E4 = [
    1 0 0 0;
    0 0 0 1;
    ];
    
A_lat = E1 * A * E1';
B_lat = E1 * B * E2';

A_lon = E3 * A * E3';
B_lon = E3 * B * E4';

A_lon_eigen = eig(A_lon);
A_lat_eigen = eig(A_lat);
  