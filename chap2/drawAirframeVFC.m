function handle = drawAirframeVFC(pn, pe, pd, phi, theta, psi, handle)
% define points on airframe
[V, F, patchcolors] = airframeVFC;

V = V*10;

% rotate spacecraft
V = rotate(V', phi, theta, psi)';

% translate spacecraft
V = translate(V', pn, pe, pd)';

% transform from NED to XYZ coordinates
R = [
    0, 1, 0;
    1, 0, 0;
    0, 0, -1;
    ];
V = V * R;

if isempty(handle)
    handle = patch('Vertices', V, 'Faces', F, 'FaceVertexCData', patchcolors, 'FaceColor', 'flat');
else
    set(handle, 'Vertices', V, 'Faces', F);
end