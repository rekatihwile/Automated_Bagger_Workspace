% Optimize_Global_Video.m
clear; clc; close all;

fprintf('======================================================\n');
fprintf('  STARTING GLOBAL COLLISION-FREE OPTIMIZATION\n');
fprintf('======================================================\n\n');

%% --- Phase 1: Define the Collision-Free Base Candidates (Global Sweep) ---
bagger = BaggerRobot();
ws_w = bagger.wsW;
ws_d = bagger.wsD;
base_radius = 0.05; % 5cm clearance radius (10cm diameter base)

staging_rect = [0.10, 0.10, 0.50, 0.50];
bag_rect     = [0.75, 0.20, 0.20, 0.30];

% Generate a dense grid that expands well OUTSIDE the workspace
step_size = 0.05; % 5cm grid resolution for global sweep
bx_vals = -0.6 : step_size : (ws_w + 0.6);
by_vals = -0.6 : step_size : (ws_d + 0.6);
[BX, BY] = meshgrid(bx_vals, by_vals);
all_bases = [BX(:), BY(:)];

valid_bases = [];
for i = 1:size(all_bases, 1)
    bx = all_bases(i, 1);
    by = all_bases(i, 2);
    
    col_staging = check_circle_rect_collision(bx, by, base_radius, staging_rect);
    col_bag     = check_circle_rect_collision(bx, by, base_radius, bag_rect);
    
    if ~col_staging && ~col_bag
        valid_bases = [valid_bases; bx, by];
    end
end

num_tests = size(valid_bases, 1);
fprintf('Found %d collision-free global base positions to test.\n', num_tests);

%% --- Phase 2: Scoring Points Definition ---
eval_res = 8; % 8x8 grid per area (128 total targets)
pts_staging_eval = generate_zigzag_waypoints(staging_rect, eval_res);
pts_bag_eval     = generate_zigzag_waypoints(bag_rect, eval_res);
target_points_eval = [pts_staging_eval; pts_bag_eval];
num_targets = size(target_points_eval, 1);

%% --- Phase 3: Brute Force Evaluation ---
results = zeros(num_tests, 3); 

fprintf('Evaluating kinematics (this may take a minute due to the expanded grid)...\n');
for i = 1:num_tests
    bx = valid_bases(i, 1);
    by = valid_bases(i, 2);
    
    bagger = BaggerRobot();
    bagger.workspaceCenter = [-bx + bagger.wsW/2, -by + bagger.wsD/2];
    bagger.dx = -bx;
    bagger.dy = -by;
    
    badness_sum = 0;
    valid_config = true;
    
    for k = 1:num_targets
        x_target = target_points_eval(k,1) + bagger.dx;
        y_target = target_points_eval(k,2) + bagger.dy;
        
        % Fast-Fail: isReachableXY is pure math and super fast.
        if ~bagger.isReachableXY(x_target, y_target)
            valid_config = false;
            break; 
        end
        
        q = bagger.ik([x_target, y_target, bagger.dBase]);
        J = bagger.robot.jacob0(q);
        s = svd(J(1:2, 1:2));
        s = s(s > 1e-12);
        eta = (numel(s) < 2) * 0 + (numel(s) >= 2) * (min(s) / max(s));
        badness_sum = badness_sum + (1 - eta);
    end
    
    if valid_config
        results(i, :) = [bx, by, badness_sum / num_targets];
    else
        results(i, :) = [bx, by, inf];
    end
end
fprintf('Evaluation Complete!\n');

%% --- Phase 4: Rank and Select Candidates ---
valid_results = results(results(:,3) < inf, :);
if isempty(valid_results)
    error('No global configurations provided 100% reachability to the targets.');
end

[~, sort_idx] = sort(valid_results(:, 3), 'ascend');
sorted_valid = valid_results(sort_idx, :);

best_data   = sorted_valid(1, :);
second_data = sorted_valid(2, :);
worst_data  = sorted_valid(end, :);

fprintf('\n======================================================\n');
fprintf('  OPTIMAL GEOMETRIES FOUND (Base relative to Workspace)\n');
fprintf('======================================================\n');
fprintf('1. BEST:        bx = %5.2fm, by = %5.2fm | Mean Badness: %.4f\n', best_data(1), best_data(2), best_data(3));
fprintf('2. SECOND BEST: bx = %5.2fm, by = %5.2fm | Mean Badness: %.4f\n', second_data(1), second_data(2), second_data(3));
fprintf('3. WORST VALID: bx = %5.2fm, by = %5.2fm | Mean Badness: %.4f\n', worst_data(1), worst_data(2), worst_data(3));

%% --- Phase 5: Generate Animation Paths ---
anim_res = 5; 
pts_staging_anim = generate_zigzag_waypoints(staging_rect, anim_res);
pts_bag_anim     = generate_zigzag_waypoints(bag_rect, anim_res);
anim_waypoints   = [pts_staging_anim; pts_bag_anim]; 

interp_steps = 4; 
xyz_smooth_path = generate_dense_path(anim_waypoints, interp_steps);
num_frames = size(xyz_smooth_path, 1);

area_titles = {'Worst Valid', 'Best Overall', 'Second Best'};
cases = struct();

for r = 1:3
    data = [];
    if r == 1, data = worst_data; elseif r == 2, data = best_data; else, data = second_data; end
    
    [cases(r).bagger, cases(r).q, cases(r).eta, cases(r).badness] = ...
        get_prep_robot(data(1), data(2), area_titles{r}, xyz_smooth_path);
        
    cases(r).title_str = sprintf('%s\\n(BaseX=%.2f, BaseY=%.2f)\\nMean Badness: %.3f', area_titles{r}, data(1), data(2), data(3));
end

%% --- Phase 6: Build 3x3 Dashboard & Render Video ---
% Opens at a manageable 720p on your screen to avoid taking over the monitor
fig = figure('Name', 'Global Workspace Optimization', 'Color', 'w', ...
    'Units', 'pixels', 'Position', [100 100 1280 720]);

% Create a custom grid layout: 4 rows, 3 columns
% This allows us to give the 3D plots twice as much vertical space
t = tiledlayout(fig, 4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for r = 1:3
    bagger = cases(r).bagger;
    wp_abs = anim_waypoints + [bagger.dx, bagger.dy, 0]; 
    
    % -- ROW 1 & 2: 3D Visualization (Spans 2 rows for extra height) --
    cases(r).ax3 = nexttile(t, r, [2 1]); 
    setup_3d_axes(cases(r).ax3, bagger, cases(r).title_str);
    draw_scene_3d_opaque(cases(r).ax3, bagger, staging_rect, bag_rect);
    
    bagger.robot.plot(cases(r).q(1,:), 'workspace', get_dynamic_bounds(bagger), 'scale', 0.25, 'noname', 'nobase');
    lighting(cases(r).ax3, 'gouraud'); material(cases(r).ax3, 'dull'); light(cases(r).ax3, 'Position', [0.8 0.2 1.8], 'Style', 'local');
    
    scatter3(cases(r).ax3, wp_abs(:,1), wp_abs(:,2), wp_abs(:,3), 15, [0.3 0.3 0.3], 'filled'); 
    
    cases(r).hSeg3D = gobjects(num_frames-1, 1);
    for k = 1:num_frames-1
        p1 = xyz_smooth_path(k,:) + [bagger.dx, bagger.dy, 0];
        p2 = xyz_smooth_path(k+1,:) + [bagger.dx, bagger.dy, 0];
        cases(r).hSeg3D(k) = plot3(cases(r).ax3, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 3.5, 'Visible', 'off');
    end
    p0 = xyz_smooth_path(1,:) + [bagger.dx, bagger.dy, 0];
    cases(r).hCurrent3D = plot3(cases(r).ax3, p0(1), p0(2), p0(3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.10 0.75 0.20], 'MarkerEdgeColor', 'k');
    
    % -- ROW 3: Opaque 2D top-down view (Spans 1 row) --
    cases(r).ax2D = nexttile(t, r + 6, [1 1]); 
    setup_2d_axes(cases(r).ax2D, bagger);
    draw_scene_2d_opaque(cases(r).ax2D, bagger, staging_rect, bag_rect);
    draw_base_2d(cases(r).ax2D, bagger);
    
    scatter(cases(r).ax2D, wp_abs(:,1), wp_abs(:,2), 15, [0.3 0.3 0.3], 'filled'); 
    
    cases(r).hSeg2D = gobjects(num_frames-1, 1);
    for k = 1:num_frames-1
        p1 = xyz_smooth_path(k,:) + [bagger.dx, bagger.dy, 0];
        p2 = xyz_smooth_path(k+1,:) + [bagger.dx, bagger.dy, 0];
        cases(r).hSeg2D(k) = plot(cases(r).ax2D, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 2.5, 'Visible', 'off');
    end
    cases(r).hCurrent2D = plot(cases(r).ax2D, p0(1), p0(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.10 0.75 0.20], 'MarkerEdgeColor', 'k');
    
    % -- ROW 4: Metric Line Plots (Spans 1 row) --
    cases(r).axLine = nexttile(t, r + 9, [1 1]); 
    cases(r).hLine = setup_line_plot(cases(r).axLine, num_frames);
end

%% --- Video Export Setup ---
fprintf('\nPreparing to render 1080p MP4 Video...\n');
video_filename = 'Workspace_Optimization_1080p.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.Quality = 100;
v.FrameRate = 15;
open(v);

drawnow;
pause(1);

fprintf('Recording frames...\n');
for i = 1:num_frames
    for r = 1:3
        cases(r).bagger.robot.animate(cases(r).q(i,:));
        
        cLow  = [0.10 0.75 0.20]; cHigh = [0.85 0.10 0.10];
        cNow = (1 - cases(r).badness(i)) * cLow + cases(r).badness(i) * cHigh;
        
        p = cases(r).bagger.fk(cases(r).q(i,:)).t;
        if i > 1
            set(cases(r).hSeg3D(i-1), 'Visible', 'on', 'Color', cNow);
            set(cases(r).hSeg2D(i-1), 'Visible', 'on', 'Color', cNow);
        end
        set(cases(r).hCurrent3D, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'MarkerFaceColor', cNow);
        set(cases(r).hCurrent2D, 'XData', p(1), 'YData', p(2), 'MarkerFaceColor', cNow);
        set(cases(r).hLine, 'XData', 1:i, 'YData', cases(r).eta(1:i));
    end
    drawnow;
    
    % Capture the 720p figure frame
    frame = getframe(fig);
    % Silently upscale it to 1080p for the actual video file
    img_1080p = imresize(frame.cdata, [1080, 1920]); 
    writeVideo(v, img_1080p);
end

close(v);
fprintf('\nSUCCESS: High-Res 1080p video saved to %s\n', fullfile(pwd, video_filename));

%% ======================= HELPER FUNCTIONS =======================

function is_collision = check_circle_rect_collision(cx, cy, r, rect)
    rx = rect(1); ry = rect(2); rw = rect(3); rh = rect(4);
    closestX = max(rx, min(cx, rx + rw));
    closestY = max(ry, min(cy, ry + rh));
    distX = cx - closestX;
    distY = cy - closestY;
    is_collision = (distX^2 + distY^2) < (r^2);
end

function points = generate_zigzag_waypoints(rect, res)
    x_vals = linspace(rect(1), rect(1)+rect(3), res);
    y_vals = linspace(rect(2), rect(2)+rect(4), res);
    points = zeros(res*res, 3);
    idx = 1;
    for r = 1:res
        y = y_vals(r);
        x_row = x_vals;
        if mod(r, 2) == 0, x_row = fliplr(x_row); end
        for c = 1:res
            points(idx, :) = [x_row(c), y, 0.01];
            idx = idx + 1;
        end
    end
end

function dense_path = generate_dense_path(waypoints, steps_per_seg)
    num_wp = size(waypoints, 1);
    num_dense = (num_wp - 1) * steps_per_seg + 1;
    dense_path = zeros(num_dense, 3);
    idx = 1;
    for i = 1:num_wp-1
        p1 = waypoints(i, :);
        p2 = waypoints(i+1, :);
        for j = 1:steps_per_seg
            t = (j-1) / steps_per_seg;
            t_smooth = t^2 * (3 - 2*t); 
            dense_path(idx, :) = p1 + t_smooth * (p2 - p1);
            idx = idx + 1;
        end
    end
    dense_path(end, :) = waypoints(end, :);
end

function [bagger, qPath, etaPath, badnessPath] = get_prep_robot(bx, by, rName, relative_path)
    bagger = BaggerRobot();
    bagger.robot.name = rName; 
    bagger.workspaceCenter = [-bx + bagger.wsW/2, -by + bagger.wsD/2];
    bagger.dx = -bx;
    bagger.dy = -by;
    
    num_pts = size(relative_path, 1);
    qPath = zeros(num_pts, 4);
    etaPath = zeros(num_pts, 1);
    badnessPath = zeros(num_pts, 1);
    
    for i = 1:num_pts
        x_target = relative_path(i,1) + bagger.dx;
        y_target = relative_path(i,2) + bagger.dy;
        q = bagger.ik([x_target, y_target, bagger.dBase]);
        qPath(i,:) = q;
        
        J = bagger.robot.jacob0(q);
        s = svd(J(1:2, 1:2));
        s = s(s > 1e-12);
        eta = (numel(s) < 2) * 0 + (numel(s) >= 2) * (min(s) / max(s));
        etaPath(i) = eta;
        badnessPath(i) = 1 - eta;
    end
end

function bounds = get_dynamic_bounds(bagger)
    xc = bagger.workspaceCenter(1); yc = bagger.workspaceCenter(2);
    bounds = [-1.5 + xc, 1.5 + xc, -1.2 + yc, 1.2 + yc, 0, 1.10];
end

function setup_3d_axes(ax, bagger, title_str)
    hold(ax, 'on'); grid(ax, 'on'); title(ax, sprintf(title_str), 'FontWeight', 'bold');
    b = get_dynamic_bounds(bagger); xlim(ax, b(1:2)); ylim(ax, b(3:4)); zlim(ax, b(5:6));
    view(ax, 38, 24); pbaspect(ax, [1 1 0.75]);
end

function setup_2d_axes(ax, bagger)
    hold(ax, 'on'); axis(ax, 'equal'); title(ax, 'Top-down Workspace');
    b = get_dynamic_bounds(bagger); xlim(ax, b(1:2)); ylim(ax, b(3:4));
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
end

function draw_scene_3d_opaque(ax, bagger, staging, bag)
    zFloor = 0.02; zBelt = 0.05; zBag = 0.4;
    draw_box(ax, bagger.dx, bagger.dy, bagger.wsW, bagger.wsD, zFloor, [0.90 0.90 0.90], 1.0);
    draw_box(ax, bagger.dx+staging(1), bagger.dy+staging(2), staging(3), staging(4), zBelt, [0.20 0.20 0.20], 1.0);
    draw_box(ax, bagger.dx+bag(1), bagger.dy+bag(2), bag(3), bag(4), zBag, [0.55 0.35 0.15], 1.0);
end

function draw_scene_2d_opaque(ax, bagger, staging, bag)
    x0 = bagger.dx; y0 = bagger.dy;
    patch(ax, [x0 x0+bagger.wsW x0+bagger.wsW x0], [y0 y0 y0+bagger.wsD y0+bagger.wsD], [0.9 0.9 0.9], 'FaceAlpha', 1.0, 'EdgeColor', 'k');
    patch(ax, [x0+staging(1), x0+staging(1)+staging(3), x0+staging(1)+staging(3), x0+staging(1)], ...
              [y0+staging(2), y0+staging(2), y0+staging(2)+staging(4), y0+staging(2)+staging(4)], [0.2 0.2 0.2], 'FaceAlpha', 1.0, 'EdgeColor', 'k');
    patch(ax, [x0+bag(1), x0+bag(1)+bag(3), x0+bag(1)+bag(3), x0+bag(1)], ...
              [y0+bag(2), y0+bag(2), y0+bag(2)+bag(4), y0+bag(2)+bag(4)], [0.55 0.35 0.15], 'FaceAlpha', 1.0, 'EdgeColor', 'k');
end

function draw_base_2d(ax, bagger)
    ang = linspace(0, 2*pi, 50);
    patch(ax, 0.05*cos(ang), 0.05*sin(ang), [0.15 0.35 0.80], 'FaceAlpha', 1.0, 'EdgeColor', [0.15 0.35 0.80], 'LineWidth', 2);
    plot(ax, 0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);
end

function h = draw_box(ax, x, y, w, l, hgt, color, alphaVal)
    v = [0 0 0; w 0 0; w l 0; 0 l 0; 0 0 hgt; w 0 hgt; w l hgt; 0 l hgt];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    t = hgtransform('Parent', ax);
    patch('Vertices',v,'Faces',f,'FaceColor',color,'FaceAlpha',alphaVal, 'EdgeColor','k','Parent',t, ...
        'FaceLighting','gouraud', 'EdgeLighting','none', 'AmbientStrength', 0.35, 'DiffuseStrength', 0.85);
    set(t, 'Matrix', makehgtform('translate', x, y, 0));
end

function h = setup_line_plot(ax, nPts)
    hold(ax, 'on'); grid(ax, 'on'); title(ax, 'S-Value ( \eta ) vs Time');
    h = plot(ax, 1, 0, '-', 'Color', [0.2 0.5 0.8], 'LineWidth', 2.5);
    xlim(ax, [1, nPts]); ylim(ax, [0, 1.05]); xlabel(ax, 'Path Step'); ylabel(ax, '\eta');
end