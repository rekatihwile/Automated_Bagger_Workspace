function optimize_workspace_base_position()
% OPTIMIZE_WORKSPACE_BASE_POSITION
% Brute-force sweep of workspace-center y offset for the RRPR bagger robot.
%
% Fixes:
%   1) struct preallocation issue
%   2) removes dependency on Image Processing Toolbox (no viscircles)

clear; clc; close all;

% ---------------- USER SETTINGS ----------------
ySweep      = 0.00:0.01:0.65;   % workspace-center y offsets to test [m]
etaWarn     = 0.15;             % warning threshold for conditioning metric
doDenseGrid = true;             % verify whole rectangle, not just raster path

% Full OUTER project workspace rectangle (the big one)
% Set these to your actual dimensions if needed.
W = 0.60;                       % width in x [m]
H = 0.40;                       % height in y [m]
rasterDx = 0.03;                % raster spacing in x [m]
rasterDy = 0.03;                % raster spacing in y [m]

% ---------------- RUN SWEEP ----------------
results = repmat(make_empty_result_template(), numel(ySweep), 1);

for k = 1:numel(ySweep)
    bagger = BaggerRobot();
    bagger.workspaceCenter = [0, ySweep(k), 0];

    xyzPath = make_raster_path(W, H, ySweep(k), rasterDx, rasterDy);
    results(k) = evaluate_path_case(bagger, xyzPath, etaWarn, doDenseGrid, W, H);
    results(k).workspaceCenterY = ySweep(k);
end

% Keep only valid-path candidates first
pathValid = [results.pathValid];
validResults = results(pathValid);

if isempty(validResults)
    error('No valid path cases found in the tested sweep.');
end

% Ranking: lowest RMS badness, then highest minimum eta, then highest mean eta
T = struct2table(validResults);
[~, idx] = sortrows([T.rmsBadness, -T.minEta, -T.meanEta], [1 2 3]);
validResults = validResults(idx);
best = validResults(1);

fprintf('\n============================================================\n');
fprintf('BEST WORKSPACE CENTER Y OFFSET FOUND\n');
fprintf('============================================================\n');
fprintf('workspace center y = %.4f m\n', best.workspaceCenterY);
fprintf('RMS badness        = %.6f\n', best.rmsBadness);
fprintf('Mean eta           = %.6f\n', best.meanEta);
fprintf('Median eta         = %.6f\n', best.medianEta);
fprintf('Min eta            = %.6f\n', best.minEta);
fprintf('Max eta            = %.6f\n', best.maxEta);
fprintf('Warn fraction      = %.2f %%   (eta < %.2f)\n', 100*best.warnFraction, etaWarn);
fprintf('Mean radius        = %.4f m\n', best.meanRadius);
fprintf('Max radius         = %.4f m\n', best.maxRadius);
if isfield(best,'denseGrid') && ~isempty(best.denseGrid)
    fprintf('Dense-grid valid   = %.2f %%\n', 100*best.denseGrid.validFraction);
    fprintf('Dense-grid RMS bad = %.6f\n', best.denseGrid.rmsBadness);
    fprintf('Dense-grid min eta = %.6f\n', best.denseGrid.minEta);
end
fprintf('============================================================\n\n');

disp(struct2table(results));

% Rebuild best bagger for visualization
bestBagger = BaggerRobot();
bestBagger.workspaceCenter = [0, best.workspaceCenterY, 0];
show_best_case(bestBagger, best.xyzPath, best.etaPath, etaWarn, W, H);

end

% =======================================================================

function xyzPath = make_raster_path(W, H, yCenter, dx, dy)
xv = (-W/2):dx:(W/2);
yv = (yCenter - H/2):dy:(yCenter + H/2);

pts = [];
for i = 1:numel(yv)
    if mod(i,2)==1
        xrow = xv;
    else
        xrow = fliplr(xv);
    end
    yrow = yv(i) * ones(size(xrow));
    zrow = zeros(size(xrow));
    pts = [pts; [xrow(:), yrow(:), zrow(:)]]; %#ok<AGROW>
end

xyzPath = pts;
end

function out = evaluate_path_case(bagger, xyzPath, etaWarn, doDenseGrid, W, H)
out = make_empty_result_template();

n = size(xyzPath,1);
qPath = nan(n,4);
etaPath = nan(n,1);
reachMask = false(n,1);

for i = 1:n
    x = xyzPath(i,1);
    y = xyzPath(i,2);

    [q, ok] = get_ik_safely(bagger, x, y);
    if ok
        qPath(i,1:numel(q)) = q(:).';
        etaPath(i) = planar_eta_from_q(bagger, q);
        reachMask(i) = true;
    end
end

out.xyzPath = xyzPath;
out.qPath = qPath;
out.etaPath = etaPath;
out.reachMask = reachMask;
out.pathValid = all(reachMask);

validEta = etaPath(reachMask);
if isempty(validEta)
    return;
end

badness = 1 - validEta;
rPath = hypot(xyzPath(reachMask,1), xyzPath(reachMask,2));

out.meanEta = mean(validEta);
out.medianEta = median(validEta);
out.minEta = min(validEta);
out.maxEta = max(validEta);
out.rmsBadness = sqrt(mean(badness.^2));
out.warnFraction = mean(validEta < etaWarn);
out.meanRadius = mean(rPath);
out.maxRadius = max(rPath);

if doDenseGrid
    dx = 0.02;
    dy = 0.02;
    xv = (-W/2):dx:(W/2);
    yv = (bagger.workspaceCenter(2)-H/2):dy:(bagger.workspaceCenter(2)+H/2);
    [X,Y] = meshgrid(xv,yv);

    eta = nan(size(X));
    reach = false(size(X));

    for r = 1:size(X,1)
        for c = 1:size(X,2)
            [q, ok] = get_ik_safely(bagger, X(r,c), Y(r,c));
            if ok
                eta(r,c) = planar_eta_from_q(bagger, q);
                reach(r,c) = true;
            end
        end
    end

    valid = eta(reach);
    if isempty(valid)
        dg.validFraction = 0;
        dg.meanEta = NaN;
        dg.minEta = NaN;
        dg.rmsBadness = inf;
    else
        dg.validFraction = mean(reach(:));
        dg.meanEta = mean(valid);
        dg.minEta = min(valid);
        dg.rmsBadness = sqrt(mean((1-valid).^2));
    end
    dg.X = X; dg.Y = Y; dg.eta = eta; dg.reach = reach;
    out.denseGrid = dg;
end
end

function [q, ok] = get_ik_safely(bagger, x, y)
q = [];
ok = false;

try
    if ismethod(bagger,'ikPrev')
        q = bagger.ikPrev([x y 0]);
    elseif ismethod(bagger,'inverseKinematics')
        q = bagger.inverseKinematics([x y 0]);
    elseif ismethod(bagger,'ik')
        q = bagger.ik([x y 0]);
    else
        error('No IK method found on BaggerRobot.');
    end
    q = q(:).';
    ok = all(isfinite(q));
catch
    ok = false;
end
end

function eta = planar_eta_from_q(bagger, q)
J = bagger.robot.jacob0(q);

if size(J,1) >= 2
    Jxy = J(1:2,:);
else
    eta = 0;
    return;
end

s = svd(Jxy);
if isempty(s) || max(s) <= eps
    eta = 0;
else
    eta = min(s)/max(s);
end
end

function show_best_case(bagger, xyzPath, etaPath, etaWarn, W, H)
figure('Color','w','Name','Best Base Offset');
hold on; axis equal; grid on;

% Outer project workspace rectangle
xRect = [-W/2, W/2, W/2, -W/2, -W/2];
y0 = bagger.workspaceCenter(2) - H/2;
y1 = bagger.workspaceCenter(2) + H/2;
yRect = [y0, y0, y1, y1, y0];
plot(xRect, yRect, 'k-', 'LineWidth', 1.5);

% Reach circle without viscircles dependency
plot_circle(bagger.robotBaseXY(1), bagger.robotBaseXY(2), bagger.L1 + bagger.L2, '--');

% Path colored by eta
valid = isfinite(etaPath);
scatter(xyzPath(valid,1), xyzPath(valid,2), 20, etaPath(valid), 'filled');
colormap(parula);
cb = colorbar;
ylabel(cb, '\eta = \sigma_{min}/\sigma_{max}', 'Interpreter','tex');

% Mark warning points
warn = valid & (etaPath < etaWarn);
plot(xyzPath(warn,1), xyzPath(warn,2), 'rx', 'MarkerSize', 8, 'LineWidth', 1.2);

plot(bagger.robotBaseXY(1), bagger.robotBaseXY(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

xlabel('x [m]');
ylabel('y [m]');
title(sprintf('Best workspace-center y = %.3f m', bagger.workspaceCenter(2)));
legend({'Outer workspace','Reach boundary','Path samples','Warning points','Robot base'}, ...
    'Location','bestoutside');
end

function plot_circle(xc, yc, r, lineStyle)
th = linspace(0, 2*pi, 400);
plot(xc + r*cos(th), yc + r*sin(th), lineStyle, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.0);
end

function out = make_empty_result_template()
out = struct( ...
    'workspaceCenterY', NaN, ...
    'xyzPath', [], ...
    'qPath', [], ...
    'etaPath', [], ...
    'reachMask', [], ...
    'pathValid', false, ...
    'meanEta', NaN, ...
    'medianEta', NaN, ...
    'minEta', NaN, ...
    'maxEta', NaN, ...
    'rmsBadness', inf, ...
    'warnFraction', 1, ...
    'meanRadius', NaN, ...
    'maxRadius', NaN, ...
    'denseGrid', struct( ...
        'X', [], 'Y', [], 'eta', [], 'reach', [], ...
        'validFraction', NaN, 'meanEta', NaN, ...
        'minEta', NaN, 'rmsBadness', inf));
end
