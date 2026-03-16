classdef BaggerRobot
    properties
        L1 double = 0.5
        L2 double = 0.5
        dBase double = 0.8
        qlimP double = [0 0.7]

        wsW double = 1.0
        wsD double = 0.7
        dx double = -0.5
        dy double = 0.2

        zBelt double = 0.05
        cubeH double = 0.08
        cubeSize double = 0.08

        workspaceCenter double = [0.0 0.3]
        robotBaseXY double = [0.0 0.0]

        bagTarget double
        robot


    end
    
    methods
        function obj = BaggerRobot()
        obj.dx = obj.workspaceCenter(1) - obj.wsW/2;
        obj.dy = obj.workspaceCenter(2) - obj.wsD/2;

        obj.bagTarget = [obj.dx + 0.85, obj.dy + 0.35, 0.4];

        L(1) = Link([0 obj.dBase 0 0], 'modified');
        L(2) = Link([0 0 obj.L1 0], 'modified');
        L(3) = Link([0 0 obj.L2 pi 1], 'modified');
        L(4) = Link([0 0 0 0], 'modified');
        L(3).qlim = obj.qlimP;

        obj.robot = SerialLink(L, 'name', 'BaggerBot');
        obj.robot.base = transl(obj.robotBaseXY(1), obj.robotBaseXY(2), 0);
        end

        function q = ik(obj, xyz)
        x = xyz(1);
        y = xyz(2);
        z = xyz(3);

        c2 = (x^2 + y^2 - obj.L1^2 - obj.L2^2) / (2*obj.L1*obj.L2);
        if abs(c2) > 1
            error('Target is out of reach.');
        end

        q2 = -acos(c2); % elbow-down
        q1 = atan2(y, x) - atan2(obj.L2*sin(q2), obj.L1 + obj.L2*cos(q2));
        q3 = max(obj.qlimP(1), min(obj.qlimP(2), obj.dBase - z));
        q4 = 0;

        q = [q1 q2 q3 q4];
        end

        function q = ikPrev(obj, xyz, qPrev)
        x = xyz(1);
        y = xyz(2);
        z = xyz(3);

        c2 = (x^2 + y^2 - obj.L1^2 - obj.L2^2) / (2*obj.L1*obj.L2);
        if abs(c2) > 1
            error('Target is out of reach.');
        end

        q2a = acos(c2);
        q2b = -acos(c2);

        q1a = atan2(y, x) - atan2(obj.L2*sin(q2a), obj.L1 + obj.L2*cos(q2a));
        q1b = atan2(y, x) - atan2(obj.L2*sin(q2b), obj.L1 + obj.L2*cos(q2b));

        q3 = max(obj.qlimP(1), min(obj.qlimP(2), obj.dBase - z));

        qa = [q1a q2a q3 0];
        qb = [q1b q2b q3 0];

        if nargin < 3 || isempty(qPrev)
            q = qb;
        else
            da = norm(wrapToPiLocal(qa(1:2) - qPrev(1:2)));
            db = norm(wrapToPiLocal(qb(1:2) - qPrev(1:2)));
            if da < db
                q = qa;
            else
                q = qb;
            end
        end
        end

        function T = fk(obj, q)
        T = obj.robot.fkine(q);
        end

        function tf = isReachableXY(obj, x, y)
        r = hypot(x,y);
        tf = (r <= obj.L1 + obj.L2) && (r >= abs(obj.L1 - obj.L2));
        end

        function mu = manipulabilityXY(obj, x, y)
        q = obj.ik([x y obj.dBase]);
        J = obj.robot.jacob0(q);
        Jxy = J(1:2,1:2);
        mu = sqrt(abs(det(Jxy * Jxy.')));
        end

        function tau = worstCaseStaticTorque(obj, payloadMass, x, y)
        r = hypot(x,y);
        tau = payloadMass * 9.81 * r;
        end

        function q = parkConfig(obj)
        q = obj.ik([0 0.3 obj.dBase]);
        end

        function [xyz, qPath, muPath] = sampleCartesianPath(obj, xyzPath)
        n = size(xyzPath,1);
        qPath = zeros(n,4);
        muPath = zeros(n,1);

        qPrev = [];
        for i = 1:n
            qPath(i,:) = obj.ikPrev(xyzPath(i,:), qPrev);
            qPrev = qPath(i,:);

            J = obj.robot.jacob0(qPath(i,:));
            Jxy = J(1:2,1:2);
            muPath(i) = sqrt(abs(det(Jxy*Jxy.')));
        end

        xyz = xyzPath;
        end
    end
end

function x = wrapToPiLocal(x)
x = atan2(sin(x), cos(x));
end