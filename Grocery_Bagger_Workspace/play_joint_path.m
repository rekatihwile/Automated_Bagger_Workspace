function play_joint_path(bagger, qPath)
    figure('Color','w');
    ax = axes; hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    view(35,25);

    for i = 1:size(qPath,1)
        bagger.robot.plot(qPath(i,:), 'scale', 0.3, 'noname');
        axis([-0.9 1.1 -0.2 1.2 0 1.1]);
        drawnow;
    end
end