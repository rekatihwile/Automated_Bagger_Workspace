clear; clc; close all;

bagger = BaggerRobot();

mainFig = figure('Name','Grocery Bagger Demo','Color','w', ...
    'Units','normalized','Position',[0.08 0.08 0.84 0.78]);

ax3D = subplot(1,2,1); hold(ax3D,'on'); grid(ax3D,'on'); axis(ax3D,'equal');
title(ax3D,'3D View');
xlabel(ax3D,'x [m]'); ylabel(ax3D,'y [m]'); zlabel(ax3D,'z [m]');

ax2D = subplot(1,2,2); hold(ax2D,'on'); grid(ax2D,'on'); axis(ax2D,'equal');
title(ax2D,'Top View - Click items in staging area, press Enter to run');
xlabel(ax2D,'x [m]'); ylabel(ax2D,'y [m]');

txt_joints = uicontrol('Style','text','Units','normalized', ...
    'Position',[0.37 0.89 0.28 0.04], ...
    'String','Joints: [0,0,0,0]','BackgroundColor','w','FontSize',10);

txt_ee = uicontrol('Style','text','Units','normalized', ...
    'Position',[0.37 0.85 0.28 0.04], ...
    'String','EE Pos: [0,0,0]','BackgroundColor','w','FontSize',10);

draw_scene(ax3D, bagger, true);
draw_scene(ax2D, bagger, false);

qCurrent = bagger.parkConfig();

axes(ax3D);
bagger.robot.plot(qCurrent, 'scale', 0.3, 'noname');
view(ax3D, 35, 25);
camlight('headlight');

axes(ax2D);
view(ax2D, 2);
xlim(ax2D,[bagger.dx-0.2, bagger.dx + bagger.wsW + 0.2]);
ylim(ax2D,[bagger.dy-0.2, bagger.dy + bagger.wsD + 0.2]);

hItem = draw_box(ax3D, 0, 0, bagger.cubeSize, bagger.cubeSize, bagger.cubeH, [1 0 0], 1.0);

itemQueue = [];
markers = gobjects(0);

disp('Click inside staging area to queue items. Press Enter when done.');

while true
    if ~ishandle(mainFig), return; end
    axes(ax2D);
    [tx, ty, button] = ginput(1);
    if isempty(button) || button == 3
        break;
    end

    inStaging = tx >= bagger.dx+0.1 && tx <= bagger.dx+0.6 && ...
                ty >= bagger.dy+0.1 && ty <= bagger.dy+0.6;

    if inStaging
        itemQueue(end+1,:) = [tx ty];
        markers(end+1) = plot(ax2D, tx, ty, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    end
end

for i = 1:size(itemQueue,1)
    if ~ishandle(mainFig), break; end

    tx = itemQueue(i,1);
    ty = itemQueue(i,2);

    set(markers(i), 'Color', 'b', 'LineWidth', 3);

    set(hItem, 'Matrix', makehgtform('translate', ...
        tx - bagger.cubeSize/2, ty - bagger.cubeSize/2, bagger.zBelt));

    pickZ = bagger.zBelt + bagger.cubeH;

    qSteps = {
        bagger.ik([tx ty bagger.dBase]), ...
        bagger.ik([tx ty pickZ]), ...
        bagger.ik([tx ty bagger.dBase]), ...
        bagger.ik([bagger.bagTarget(1) bagger.bagTarget(2) bagger.dBase]), ...
        bagger.ik(bagger.bagTarget), ...
        bagger.ik([bagger.bagTarget(1) bagger.bagTarget(2) bagger.dBase]), ...
        bagger.parkConfig()
    };

    for s = 1:length(qSteps)
        carrying = (s >= 3 && s <= 5);
        qCurrent = move_robot(bagger, ax3D, qCurrent, qSteps{s}, ...
            hItem, carrying, txt_joints, txt_ee);
    end
end

disp('Done.');

%% ---------- helpers ----------
function qFinal = move_robot(bagger, ax3D, qStart, qTarget, hItem, carrying, txt_joints, txt_ee)
    qEnd = qTarget;
    for j = [1 2 4]
        dq = atan2(sin(qTarget(j)-qStart(j)), cos(qTarget(j)-qStart(j)));
        qEnd(j) = qStart(j) + dq;
    end

    traj = jtraj(qStart, qEnd, 25);

    for k = 1:size(traj,1)
        qNow = traj(k,:);
        axes(ax3D);
        bagger.robot.plot(qNow, 'scale', 0.3, 'noname');
        axis(ax3D, [-0.8 1.2 -0.2 1.2 0 1.2]);

        T = bagger.fk(qNow);
        p = T.t;

        set(txt_joints, 'String', sprintf('Joints: [%.1f°, %.1f°, %.1f cm, %.1f°]', ...
            rad2deg(qNow(1)), rad2deg(qNow(2)), 100*qNow(3), rad2deg(qNow(4))));
        set(txt_ee, 'String', sprintf('EE Pos: [%.2f, %.2f, %.2f]', p(1), p(2), p(3)));

        if carrying
            set(hItem, 'Matrix', makehgtform('translate', ...
                p(1)-bagger.cubeSize/2, p(2)-bagger.cubeSize/2, p(3)-bagger.cubeH));
        end

        drawnow;
    end

    qFinal = qEnd;
end

function draw_scene(ax, bagger, is3D)
    zFloor = 0.02; zBelt = 0.05; zBag = 0.4;
    if ~is3D
        zFloor = 0; zBelt = 0; zBag = 0;
    end

    draw_box(ax, bagger.dx, bagger.dy, bagger.wsW, bagger.wsD, zFloor, [0.82 0.82 0.82], 0.6);
    draw_box(ax, bagger.dx+0.1, bagger.dy+0.1, 0.5, 0.5, zBelt, [0.12 0.12 0.12], 1.0);
    draw_box(ax, bagger.dx+0.75, bagger.dy+0.2, 0.2, 0.3, zBag, [0.5 0.3 0.1], 1.0);
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