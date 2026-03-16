clear; clc; close all;

bagger = BaggerRobot();

% ---------------- SETTINGS ----------------
pathType = 'figure8';   % 'figure8', 'oval', 'boundarypush', 'pickplacewave'
nPts = 260;
showTrace = true;
recordVideo = true;                     % set true to save MP4
videoName = 'bagger_trajectory_demo.mp4';

xyzPath = make_demo_path(pathType, bagger, nPts);
[xyzPath, qPath, muPath] = bagger.sampleCartesianPath(xyzPath);

muNorm = muPath / max(muPath);

% crude strain score: high radius + low manipulability
rPath = hypot(xyzPath(:,1), xyzPath(:,2));
strain = 0.6*(rPath/(bagger.L1+bagger.L2)) + 0.4*(1 - muNorm);
strain = max(0, min(1, strain));

% ---------------- FIGURE ----------------
fig = figure('Name','Bagger Trajectory Demo','Color','w', ...
    'Units','normalized','Position',[0.06 0.08 0.88 0.80]);

ax3 = subplot(1,2,1); hold(ax3,'on'); grid(ax3,'on'); axis(ax3,'equal');
title(ax3, '3D Motion');
xlabel(ax3,'x [m]'); ylabel(ax3,'y [m]'); zlabel(ax3,'z [m]');

ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
title(ax2, 'Top View: Path + Workspace');
xlabel(ax2,'x [m]'); ylabel(ax2,'y [m]');

draw_scene(ax3, bagger, true);
draw_scene(ax2, bagger, false);

% Reach boundary
th = linspace(0,2*pi,500);
rOuter = bagger.L1 + bagger.L2;
plot(ax2, rOuter*cos(th), rOuter*sin(th), 'k--', 'LineWidth', 1.5);

% Optional colorbar legend for strain
imagesc(ax2, [bagger.dx bagger.dx+bagger.wsW], [bagger.dy bagger.dy+bagger.wsD], zeros(2));
hImgs = findobj(ax2, 'Type', 'image');
set(hImgs(1), 'Visible', 'off');

cmap = [linspace(0.10,0.85,256)' linspace(0.75,0.10,256)' linspace(0.20,0.10,256)'];
colormap(ax2, cmap);
cb = colorbar(ax2);
cb.Label.String = 'Path strain / reduced manipulability';
cb.Ticks = [0 1];
cb.TickLabels = {'Low','High'};
caxis(ax2, [0 1]);

% Pre-create colored path segments in top view (initially hidden)
hSeg2D = gobjects(nPts-1,1);
for k = 1:nPts-1
    hSeg2D(k) = plot(ax2, xyzPath(k:k+1,1), xyzPath(k:k+1,2), '-', ...
        'Color', [0.85 0.85 0.85], 'LineWidth', 3, 'Visible', 'off');
end

% Pre-create colored path segments in 3D (initially hidden)
hSeg3D = gobjects(nPts-1,1);
for k = 1:nPts-1
    hSeg3D(k) = plot3(ax3, xyzPath(k:k+1,1), xyzPath(k:k+1,2), xyzPath(k:k+1,3), '-', ...
        'Color', [0.85 0.85 0.85], 'LineWidth', 3, 'Visible', 'off');
end

% Current point markers
hCurrent2D = plot(ax2, xyzPath(1,1), xyzPath(1,2), 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [0.10 0.75 0.20], ...
    'MarkerEdgeColor', 'k');

hCurrent3D = plot3(ax3, xyzPath(1,1), xyzPath(1,2), xyzPath(1,3), 'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [0.10 0.75 0.20], ...
    'MarkerEdgeColor', 'k');

% Start robot
q0 = qPath(1,:);
axes(ax3);
bagger.robot.plot(q0, 'scale', 0.3, 'noname');
view(ax3, 38, 24);
camlight('headlight');
axis(ax3, [-0.9 1.1 -0.2 1.2 0 1.1]);

% Telemetry
txt1 = annotation(fig,'textbox',[0.37 0.89 0.25 0.04], ...
    'String','', 'EdgeColor','none','FontSize',11,'FontWeight','bold');
txt2 = annotation(fig,'textbox',[0.37 0.85 0.25 0.04], ...
    'String','', 'EdgeColor','none','FontSize',11);
txt3 = annotation(fig,'textbox',[0.37 0.81 0.25 0.04], ...
    'String','', 'EdgeColor','none','FontSize',11);

% Optional video
v = [];
if recordVideo
    v = VideoWriter(videoName, 'MPEG-4');
    v.FrameRate = 30;
    open(v);
end

% ---------------- ANIMATION ----------------
for i = 1:nPts
    q = qPath(i,:);

    axes(ax3);
    bagger.robot.plot(q, 'scale', 0.3, 'noname');
    view(ax3, 38, 24);
    axis(ax3, [-0.9 1.1 -0.2 1.2 0 1.1]);

    T = bagger.fk(q);
    p = T.t;

    % green -> red based on strain
    cLow  = [0.10 0.75 0.20];
    cHigh = [0.85 0.10 0.10];
    cNow = (1 - strain(i)) * cLow + strain(i) * cHigh;

    % reveal and color traversed segments
    if i > 1
        set(hSeg2D(i-1), 'Visible', 'on', 'Color', cNow, 'LineWidth', 3.5);
        set(hSeg3D(i-1), 'Visible', 'on', 'Color', cNow, 'LineWidth', 3.5);
    end

    % move current markers
    set(hCurrent2D, ...
        'XData', p(1), ...
        'YData', p(2), ...
        'MarkerFaceColor', cNow);

    set(hCurrent3D, ...
        'XData', p(1), ...
        'YData', p(2), ...
        'ZData', p(3), ...
        'MarkerFaceColor', cNow);

    % telemetry
    set(txt1, 'String', sprintf('Path: %s', pathType));
    set(txt2, 'String', sprintf('r = %.3f m   |   manipulability = %.3f', rPath(i), muNorm(i)));
    set(txt3, 'String', sprintf('strain score = %.3f', strain(i)));

    drawnow;

    if ~isempty(v)
        writeVideo(v, getframe(fig));
    end
end

if ~isempty(v)
    close(v);
    fprintf('Saved video: %s\n', videoName);
end

% ---------------- SUPPORTING PLOTS ----------------
figure('Color','w','Position',[120 120 1100 420]);

subplot(1,3,1);
plot(muNorm, 'LineWidth', 2);
grid on;
title('Manipulability along path');
xlabel('sample');
ylabel('\mu / \mu_{max}');

subplot(1,3,2);
plot(rPath, 'LineWidth', 2);
grid on;
title('Radial distance along path');
xlabel('sample');
ylabel('r [m]');

subplot(1,3,3);
plot(strain, 'LineWidth', 2);
grid on;
title('Strain score along path');
xlabel('sample');
ylabel('score');

% ---------------- HELPERS ----------------
function draw_scene(ax, bagger, is3D)
zFloor = 0.02; zBelt = 0.05; zBag = 0.4;
if ~is3D
    zFloor = 0; zBelt = 0; zBag = 0;
end

draw_box(ax, bagger.dx, bagger.dy, bagger.wsW, bagger.wsD, zFloor, [0.90 0.90 0.90], 0.22);
draw_box(ax, bagger.dx+0.1, bagger.dy+0.1, 0.5, 0.5, zBelt, [0.12 0.12 0.12], 1.0);
draw_box(ax, bagger.dx+0.75, bagger.dy+0.2, 0.2, 0.3, zBag, [0.55 0.35 0.15], 1.0);

if ~is3D
    rectangle(ax, 'Position', [bagger.dx, bagger.dy, bagger.wsW, bagger.wsD], ...
        'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.5);
    rectangle(ax, 'Position', [bagger.dx+0.1, bagger.dy+0.1, 0.5, 0.5], ...
        'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1.5);
    rectangle(ax, 'Position', [bagger.dx+0.75, bagger.dy+0.2, 0.2, 0.3], ...
        'EdgeColor', [0.35 0.20 0.05], 'LineWidth', 1.5);

    xlim(ax, [-1.0 1.0]);
    ylim(ax, [-0.2 1.2]);
end
end

function h = draw_box(ax, x, y, w, l, hgt, color, alphaVal)
v = [0 0 0; w 0 0; w l 0; 0 l 0; 0 0 hgt; w 0 hgt; w l hgt; 0 l hgt];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
t = hgtransform('Parent', ax);
patch('Vertices',v,'Faces',f,'FaceColor',color,'FaceAlpha',alphaVal, ...
    'EdgeColor','k','LineWidth',1,'Parent',t);
set(t, 'Matrix', makehgtform('translate', x, y, 0));
h = t;
end