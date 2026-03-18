function xyzPath = make_demo_path(pathType, bagger, nPts)
    if nargin < 3
        nPts = 240;
    end

    t = linspace(0, 2*pi, nPts).';

    % Center of the outer workspace rectangle
    xc = bagger.dx + bagger.wsW/2;
    yc = bagger.dy + bagger.wsD/2;

    % Margin so the path stays just inside the workspace box
    marginX = 0.03;
    marginY = 0.03;

    aRect = bagger.wsW/2 - marginX;
    bRect = bagger.wsD/2 - marginY;

    switch lower(pathType)
        case 'figure8'
            % Inscribed figure-8 centered in workspace
            x = xc + aRect * sin(t);
            y = yc + bRect * sin(2*t);
            z = bagger.dBase * ones(size(t));

        case 'oval'
            x = xc + aRect * cos(t);
            y = yc + bRect * sin(t);
            z = bagger.dBase * ones(size(t));

        case 'boundarypush'
            th = linspace(0.18*pi, 0.48*pi, nPts).';
            r = 0.70 + 0.26*(0.5 + 0.5*sin(2*t));
            x = r .* cos(th);
            y = r .* sin(th);
            z = bagger.dBase * ones(size(t));

        case 'pickplacewave'
            x = xc + 0.85*aRect * cos(t);
            y = yc + 0.75*bRect * sin(2*t);
            z = bagger.dBase - 0.16*(0.5 + 0.5*sin(3*t));

        otherwise
            error('Unknown pathType.');
    end

    xyzPath = [x y z];
end