clear; clc; close all;
bagger = BaggerRobot();

%% Grid for top-view manipulability
xv = linspace(-1.05, 1.05, 300);
yv = linspace(-0.20, 1.20, 260);
[X, Y] = meshgrid(xv, yv);

MU = nan(size(X));

for k = 1:numel(X)
    x = X(k);
    y = Y(k);

    if bagger.isReachableXY(x,y)
        try
            MU(k) = bagger.manipulabilityXY(x,y);
        catch
            MU(k) = nan;
        end
    end
end

% Normalize and quantize
muMax = max(MU(:), [], 'omitnan');
MU_norm = MU / muMax;

% Quantized bands
edges = [0 0.15 0.35 0.55 0.75 1.01];
MU_q = nan(size(MU_norm));
for i = 1:length(edges)-1
    mask = MU_norm >= edges(i) & MU_norm < edges(i+1);
    MU_q(mask) = i-1;
end

%% Radial profiles
rVals = linspace(0, bagger.L1 + bagger.L2, 400);
muRad = nan(size(rVals));
tauRad = nan(size(rVals));

payloadMass = 2.0; % kg

for i = 1:length(rVals)
    r = rVals(i);
    x = r;
    y = 0;
    if bagger.isReachableXY(x,y)
        try
            muRad(i) = bagger.manipulabilityXY(x,y) / muMax;
            tauRad(i) = bagger.worstCaseStaticTorque(payloadMass, x, y);
        catch
        end
    end
end

%% Figure
figure('Color','w','Position',[100 100 1450 560]);

% ---------- TOP VIEW ----------
subplot(1,3,1); hold on; axis equal; grid on;
title('Top View: Quantized XY Manipulability');
xlabel('x [m]'); ylabel('y [m]');

imagesc(xv, yv, MU_q);
set(gca,'YDir','normal');

% Custom discrete colormap: black -> dark blue -> teal -> yellow -> orange
cmap = [
    0.00 0.00 0.00
    0.10 0.10 0.35
    0.00 0.45 0.55
    0.95 0.90 0.20
    0.95 0.55 0.10
];
colormap(gca, cmap);
caxis([0 4]);
cb = colorbar;
cb.Ticks = 0:4;
cb.TickLabels = {'Very Low','Low','Medium','High','Very High'};
ylabel(cb, 'Manipulability');

% Reach boundary
th = linspace(0, 2*pi, 500);
rOuter = bagger.L1 + bagger.L2;
rInner = abs(bagger.L1 - bagger.L2);
plot(rOuter*cos(th), rOuter*sin(th), 'w-', 'LineWidth', 2);
if rInner > 1e-6
    plot(rInner*cos(th), rInner*sin(th), 'w--', 'LineWidth', 2);
end

% Workspace box
rectangle('Position',[bagger.dx, bagger.dy, bagger.wsW, bagger.wsD], ...
    'EdgeColor','w','LineWidth',2);

% Staging area
rectangle('Position',[bagger.dx+0.1, bagger.dy+0.1, 0.5, 0.5], ...
    'EdgeColor','w','LineWidth',2,'LineStyle','--');

% Bag zone
rectangle('Position',[bagger.dx+0.75, bagger.dy+0.2, 0.2, 0.3], ...
    'EdgeColor',[1 1 1],'LineWidth',2);

plot(0,0,'wo','MarkerFaceColor','w','MarkerSize',8);
plot(bagger.bagTarget(1), bagger.bagTarget(2), 'wx', 'LineWidth', 2, 'MarkerSize', 12);

text(0.03,0.02,'Base','Color','w','FontWeight','bold');
text(bagger.dx+0.12, bagger.dy+0.17,'Staging','Color','w','FontWeight','bold');
text(bagger.dx+0.77, bagger.dy+0.22,'Bag','Color','w','FontWeight','bold');

xlim([-1.05 1.05]);
ylim([-0.20 1.20]);

% ---------- RADIAL MANIPULABILITY ----------
subplot(1,3,2); hold on; grid on;
title('Manipulability vs Radial Distance');
xlabel('r = \surd(x^2+y^2) [m]');
ylabel('Normalized manipulability');

plot(rVals, muRad, 'LineWidth', 3);
xline(rOuter, '--k', 'Max reach', 'LabelVerticalAlignment','bottom');
yline(0, ':k');

% Show approximate active task region from workspace
taskRmin = min(hypot([bagger.dx+0.1 bagger.dx+0.85], [bagger.dy+0.1 bagger.dy+0.35]));
taskRmax = max(hypot([bagger.dx+0.6 bagger.dx+0.85], [bagger.dy+0.6 bagger.dy+0.35]));
patch([taskRmin taskRmax taskRmax taskRmin], [0 0 1.05 1.05], ...
      [0.85 0.85 0.85], 'FaceAlpha',0.25, 'EdgeColor','none');
text(mean([taskRmin taskRmax]), 0.95, 'Task region', ...
    'HorizontalAlignment','center', 'FontWeight','bold');

ylim([0 1.05]);
xlim([0 1.05]);

% ---------- STATIC TORQUE ----------
subplot(1,3,3); hold on; grid on;
title('Shoulder Torque Estimate vs Radial Distance');
xlabel('r = \surd(x^2+y^2) [m]');
ylabel('Static torque [N·m]');

plot(rVals, tauRad, 'LineWidth', 3);
xline(rOuter, '--k', 'Max reach', 'LabelVerticalAlignment','bottom');

tauBag = bagger.worstCaseStaticTorque(payloadMass, bagger.bagTarget(1), bagger.bagTarget(2));
plot(hypot(bagger.bagTarget(1), bagger.bagTarget(2)), tauBag, 'o', 'MarkerSize', 8, 'LineWidth', 2);
text(hypot(bagger.bagTarget(1), bagger.bagTarget(2))+0.02, tauBag, 'Bag target');

xlim([0 1.05]);

sgtitle('Automated Grocery Bagger Engineering Analysis');