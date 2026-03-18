clear; clc; close all;
% ============================================================
% GOLDILOCKS BASE GEOMETRY DEMO
% 2 x 3 layout (Widescreen 16:9 Format for Presentations):
%   Columns: Too Close | Optimal | Too Far
%   Row 1: 3D Views
%   Row 2: Top Views
% ============================================================

% ---------------- USER SETTINGS ----------------
pathType = 'figure8';      % 'figure8', 'oval', 'boundarypush', 'pickplacewave'
nPts = 260;
recordVideo = true;
videoName = 'bagger_goldilocks_base_demo_1080p.mp4';

% Distance from robot base at (0,0) to workspace center along +y
y_too_close = 0.00;
y_optimal   = 0.30;
y_too_far   = 0.50;

% Base visualization settings (top view only)
baseRadius = 0.07;
baseColor  = [0.15 0.35 0.80];

% ------------------------------------------------------------
% MANUAL AXIS LIMITS FOR ALL 6 VIEWS
% ------------------------------------------------------------
% 3D format: [xmin xmax ymin ymax zmin zmax]
lims3D_close = [-0.85 0.85  -0.8 0.8   0.00 1.10];
lims3D_opt   = [-0.85 0.85  (-0.8+y_optimal) (0.8+y_optimal)   0.00 1.10];
lims3D_far   = [-0.85 0.85  (-0.8+y_too_far) (0.8+y_too_far)   0.00 1.10];

% 2D format: [xmin xmax ymin ymax]
lims2D_close = [-1.05 1.05  -0.6 0.6];
lims2D_opt   = [-1.05 1.05  (-0.6+y_optimal) (0.6+y_optimal)];
lims2D_far   = [-1.05 1.05  (-0.6+y_too_far) (0.6+y_too_far)];

cfg(1) = make_case('Too close', y_too_close, 'bagger_case_close',   lims3D_close, lims2D_close);
cfg(2) = make_case('Optimal',   y_optimal,   'bagger_case_optimal', lims3D_opt,   lims2D_opt);
cfg(3) = make_case('Too far',   y_too_far,   'bagger_case_far',     lims3D_far,   lims2D_far);

% ---------------- PRECOMPUTE ALL CASES ----------------
for r = 1:3
    cases(r).bagger = configure_bagger(cfg(r).workspaceCenterY, cfg(r).robotName);
    cases(r).xyzPath = make_demo_path(pathType, cases(r).bagger, nPts);
    [cases(r).xyzPath, cases(r).qPath, cases(r).muPath] = ...
        cases(r).bagger.sampleCartesianPath(cases(r).xyzPath);
    
    % Rigorous singularity metric from planar Jacobian conditioning
    cases(r).eta = zeros(nPts,1);       
    cases(r).badness = zeros(nPts,1);   
    for i = 1:nPts
        q = cases(r).qPath(i,:);
        Jfull = cases(r).bagger.robot.jacob0(q);   
        Jxy = Jfull(1:2,:);                        
        s = svd(Jxy);
        s = s(s > 1e-10);   
        if numel(s) < 2
            eta = 0;        
        else
            eta = min(s) / max(s);
        end
        cases(r).eta(i) = eta;
        cases(r).badness(i) = 1 - eta;
    end
end

% ---------------- FIGURE ----------------
% Force exact 1920x1080 resolution for crisp 16:9 video
fig = figure('Name','Bagger Goldilocks Base Demo','Color','w', ...
    'Units','pixels','Position',[100 100 1920 1080]); 

% ---- NEW 2x3 WIDESCREEN LAYOUT ----
marginL = 0.04; 
marginR = 0.04; 
gapX    = 0.04; 
colW = (1 - marginL - marginR - 2*gapX) / 3;

topMargin = 0.06;
botMargin = 0.06;
gapY      = 0.08;
rowH = (1 - topMargin - botMargin - gapY) / 2;

rowY_top = 1 - topMargin - rowH;
rowY_bot = botMargin;

for r = 1:3
    % Calculate X position for the current column
    colX = marginL + (r-1)*(colW + gapX);

    % Top Row: 3D
    cases(r).ax3 = axes('Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [colX rowY_top colW rowH], ...
        'PositionConstraint', 'innerposition');
    hold(cases(r).ax3,'on');
    grid(cases(r).ax3,'on');
    title(cases(r).ax3, sprintf('%s base | 3D', cfg(r).label), 'FontWeight', 'bold', 'FontSize', 14);
    xlabel(cases(r).ax3,'X');
    ylabel(cases(r).ax3,'Y');
    zlabel(cases(r).ax3,'Z');
    set(cases(r).ax3, 'LooseInset', [0.02 0.02 0.02 0.02]);
    
    % Bottom Row: Top view
    cases(r).ax2 = axes('Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [colX rowY_bot colW rowH], ...
        'PositionConstraint', 'innerposition');
    hold(cases(r).ax2,'on');
    grid(cases(r).ax2,'on');
    axis(cases(r).ax2,'equal');
    title(cases(r).ax2, sprintf('%s base | top view', cfg(r).label), 'FontWeight', 'bold', 'FontSize', 14);
    xlabel(cases(r).ax2,'x [m]');
    ylabel(cases(r).ax2,'y [m]');
    box(cases(r).ax2, 'on');
    set(cases(r).ax2, 'Layer', 'top');
    set(cases(r).ax2, 'LooseInset', [0.02 0.02 0.02 0.02]);
    
    draw_scene(cases(r).ax3, cases(r).bagger, true);
    draw_scene(cases(r).ax2, cases(r).bagger, false);
    draw_base_top(cases(r).ax2, cases(r).bagger.robotBaseXY, baseRadius, baseColor);
    
    % Reach boundary in top view
    th = linspace(0,2*pi,500);
    rOuter = cases(r).bagger.L1 + cases(r).bagger.L2;
    plot(cases(r).ax2, ...
        cases(r).bagger.robotBaseXY(1) + rOuter*cos(th), ...
        cases(r).bagger.robotBaseXY(2) + rOuter*sin(th), ...
        'k--', 'LineWidth', 1.5);
    apply_2d_axes_style(cases(r).ax2, cfg(r).lims2D);
    
    % Pre-create colored path segments
    cases(r).hSeg2D = gobjects(nPts-1,1);
    cases(r).hSeg3D = gobjects(nPts-1,1);
    for k = 1:nPts-1
        cases(r).hSeg2D(k) = plot(cases(r).ax2, ...
            cases(r).xyzPath(k:k+1,1), cases(r).xyzPath(k:k+1,2), '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 3, 'Visible', 'off');
        cases(r).hSeg3D(k) = plot3(cases(r).ax3, ...
            cases(r).xyzPath(k:k+1,1), cases(r).xyzPath(k:k+1,2), cases(r).xyzPath(k:k+1,3), '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 3, 'Visible', 'off');
    end
    
    % Current point markers
    cases(r).hCurrent2D = plot(cases(r).ax2, ...
        cases(r).xyzPath(1,1), cases(r).xyzPath(1,2), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.10 0.75 0.20], 'MarkerEdgeColor', 'k');
    cases(r).hCurrent3D = plot3(cases(r).ax3, ...
        cases(r).xyzPath(1,1), cases(r).xyzPath(1,2), cases(r).xyzPath(1,3), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.10 0.75 0.20], 'MarkerEdgeColor', 'k');
        
    % Start robot once, then force axes back to fixed size
    axes(cases(r).ax3);
    cases(r).bagger.robot.plot(cases(r).qPath(1,:), 'scale', 0.3, 'noname');
    
    % Re-apply precise position after plotting alters it
    set(cases(r).ax3, ...
        'Units', 'normalized', ...
        'Position', [colX rowY_top colW rowH], ...
        'PositionConstraint', 'innerposition');
        
    apply_3d_axes_style(cases(r).ax3, cfg(r).lims3D);
    lighting(cases(r).ax3, 'gouraud');
    material(cases(r).ax3, 'dull');
    light(cases(r).ax3, 'Position', [0.8 0.2 1.8], 'Style', 'local');
    add_case_label(cases(r).ax2, cfg(r));
end

% Optional video setup
v = [];
if recordVideo
    v = VideoWriter(videoName, 'MPEG-4');
    v.FrameRate = 30;
    v.Quality = 100; % Force highest quality to prevent Drive downgrading
    open(v);
    framesPass1(nPts) = struct('cdata', [], 'colormap', []);
    framesPass2(nPts) = struct('cdata', [], 'colormap', []);
end

% ---------------- PASS 1: DRAWING THE PATH ----------------
fprintf('Simulating Pass 1: Drawing the path...\n');
for i = 1:nPts
    for r = 1:3
        q = cases(r).qPath(i,:);
        cases(r).bagger.robot.animate(q);
        
        p = cases(r).bagger.fk(q).t;
        cLow  = [0.10 0.75 0.20];
        cHigh = [0.85 0.10 0.10];
        cNow = (1 - cases(r).badness(i)) * cLow + cases(r).badness(i) * cHigh;
        
        if i > 1
            set(cases(r).hSeg2D(i-1), 'Visible', 'on', 'Color', cNow, 'LineWidth', 3.5);
            set(cases(r).hSeg3D(i-1), 'Visible', 'on', 'Color', cNow, 'LineWidth', 3.5);
        end
        
        set(cases(r).hCurrent2D, 'XData', p(1), 'YData', p(2), 'MarkerFaceColor', cNow);
        set(cases(r).hCurrent3D, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'MarkerFaceColor', cNow);
        
        apply_3d_axes_style(cases(r).ax3, cfg(r).lims3D);
        apply_2d_axes_style(cases(r).ax2, cfg(r).lims2D);
    end
    drawnow;
    
    if recordVideo
        framesPass1(i) = getframe(fig);
    end
end

% ---------------- PASS 2: TRACING THE PRE-DRAWN PATH ----------------
fprintf('Simulating Pass 2: Tracing the fully drawn path...\n');
for i = 1:nPts
    for r = 1:3
        q = cases(r).qPath(i,:);
        cases(r).bagger.robot.animate(q);
        
        p = cases(r).bagger.fk(q).t;
        cLow  = [0.10 0.75 0.20];
        cHigh = [0.85 0.10 0.10];
        cNow = (1 - cases(r).badness(i)) * cLow + cases(r).badness(i) * cHigh;
        
        set(cases(r).hCurrent2D, 'XData', p(1), 'YData', p(2), 'MarkerFaceColor', cNow);
        set(cases(r).hCurrent3D, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'MarkerFaceColor', cNow);
        
        apply_3d_axes_style(cases(r).ax3, cfg(r).lims3D);
        apply_2d_axes_style(cases(r).ax2, cfg(r).lims2D);
    end
    drawnow;
    
    if recordVideo
        framesPass2(i) = getframe(fig);
    end
end

% ---------------- STITCHING THE VIDEO ----------------
if recordVideo
    numTracingLoops = 10; 
    fprintf('Stitching video: 1x Drawing Pass + %dx Tracing Passes...\n', numTracingLoops);
    
    for i = 1:nPts
        writeVideo(v, framesPass1(i));
    end
    
    for loopNum = 1:numTracingLoops
        for i = 1:nPts
            writeVideo(v, framesPass2(i));
        end
    end
    
    close(v);
    fprintf('Saved video: %s\n', videoName);
end

% ======================= HELPERS =======================
function bagger = configure_bagger(workspaceCenterY, robotName)
bagger = BaggerRobot();
bagger.workspaceCenter = [0.0 workspaceCenterY];
bagger.robotBaseXY = [0.0 0.0];
bagger.dx = bagger.workspaceCenter(1) - bagger.wsW/2;
bagger.dy = bagger.workspaceCenter(2) - bagger.wsD/2;
bagger.bagTarget = [bagger.dx + 0.85, bagger.dy + 0.35, 0.4];
bagger.robot.base = transl(bagger.robotBaseXY(1), bagger.robotBaseXY(2), 0);
bagger.robot.name = robotName;
end

function cfg = make_case(label, workspaceCenterY, robotName, lims3D, lims2D)
cfg.label = label;
cfg.workspaceCenterY = workspaceCenterY;
cfg.robotName = robotName;
cfg.lims3D = lims3D;
cfg.lims2D = lims2D;
end

function add_case_label(ax, cfg)
text(ax, 0.98, 0.95, sprintf('workspace center y = %.2f m', cfg.workspaceCenterY), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'FontSize', 10, 'BackgroundColor', 'w', 'Margin', 2);
end

function apply_3d_axes_style(ax, lims3D)
view(ax, 38, 24);
xlim(ax, lims3D(1:2));
ylim(ax, lims3D(3:4));
zlim(ax, lims3D(5:6));
axis(ax, 'manual');
pbaspect(ax, [1 1 0.75]);
end

function apply_2d_axes_style(ax, lims2D)
xlim(ax, lims2D(1:2));
ylim(ax, lims2D(3:4));
xticks(ax, -1.0:0.5:1.0);
yticks(ax, -0.25:0.5:1.75);
end

function draw_scene(ax, bagger, is3D)
zFloor = 0.02; zBelt = 0.05; zBag = 0.4;
if ~is3D
    zFloor = 0; zBelt = 0; zBag = 0;
end
draw_box(ax, bagger.dx,      bagger.dy,      bagger.wsW, bagger.wsD, zFloor, [0.90 0.90 0.90], 1.0);
draw_box(ax, bagger.dx+0.1,  bagger.dy+0.1,  0.5,        0.5,        zBelt,  [0.20 0.20 0.20], 1.0);
draw_box(ax, bagger.dx+0.75, bagger.dy+0.2,  0.2,        0.3,        zBag,   [0.55 0.35 0.15], 1.0);
if ~is3D
    rectangle(ax, 'Position', [bagger.dx, bagger.dy, bagger.wsW, bagger.wsD], ...
        'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.5);
    rectangle(ax, 'Position', [bagger.dx+0.1, bagger.dy+0.1, 0.5, 0.5], ...
        'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1.5);
    rectangle(ax, 'Position', [bagger.dx+0.75, bagger.dy+0.2, 0.2, 0.3], ...
        'EdgeColor', [0.35 0.20 0.05], 'LineWidth', 1.5);
end
end

function draw_base_top(ax, xy, radius, faceColor)
ang = linspace(0, 2*pi, 180);
patch(ax, xy(1) + radius*cos(ang), xy(2) + radius*sin(ang), faceColor, ...
    'FaceAlpha', 0.35, 'EdgeColor', faceColor, 'LineWidth', 2.0);
plot(ax, xy(1), xy(2), 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);
text(ax, xy(1)-2.5*radius, xy(2) + 0.75*radius, 'Base', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'Color', faceColor);
end

function h = draw_box(ax, x, y, w, l, hgt, color, alphaVal)
v = [0 0 0; w 0 0; w l 0; 0 l 0; 0 0 hgt; w 0 hgt; w l hgt; 0 l hgt];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
t = hgtransform('Parent', ax);
p = patch('Vertices',v,'Faces',f,'FaceColor',color,'FaceAlpha',alphaVal, ...
    'EdgeColor','k','LineWidth',1,'Parent',t, ...
    'FaceLighting','gouraud', ...
    'EdgeLighting','none', ...
    'BackFaceLighting','reverselit');
set(p, 'AmbientStrength', 0.35, ...
    'DiffuseStrength', 0.85, ...
    'SpecularStrength', 0.12, ...
    'SpecularExponent', 20);
set(t, 'Matrix', makehgtform('translate', x, y, 0));
h = t;
end