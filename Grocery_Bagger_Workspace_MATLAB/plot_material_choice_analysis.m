clear; clc; close all;

bagger = BaggerRobot();

% ---------------- USER SETTINGS ----------------
pathType = 'figure8';   % same path as your demo
nPts = 260;
cycleTime = 4.0;        % assumed full motion time [s] for one path traversal
g = 9.81;

% --- Rough mass assumptions for presentation / napkin math ---
m1 = 0.35;   % link 1 mass [kg]
m2 = 0.25;   % link 2 mass [kg]
mp = 1.00;   % payload mass [kg]

L1 = bagger.L1;
L2 = bagger.L2;
lc1 = L1/2;
lc2 = L2/2;

% Slender rod planar inertias about each link COM
I1 = (1/12)*m1*L1^2;
I2 = (1/12)*m2*L2^2;

% ---------------- PATH + IK ----------------
xyzPath = make_demo_path(pathType, bagger, nPts);
[xyzPath, qPath, muPath] = bagger.sampleCartesianPath(xyzPath);

t = linspace(0, cycleTime, nPts).';
dt = mean(diff(t));

q1 = unwrap(qPath(:,1));
q2 = unwrap(qPath(:,2));

qd1 = gradient(q1, dt);
qd2 = gradient(q2, dt);
qdd1 = gradient(qd1, dt);
qdd2 = gradient(qd2, dt);

% ---------------- PLANAR JOINT TORQUE ESTIMATE ----------------
% Gravity omitted here because the motion is primarily in the horizontal plane.
tau1 = zeros(nPts,1);
tau2 = zeros(nPts,1);

for k = 1:nPts
    c2 = cos(q2(k));
    s2 = sin(q2(k));
    
    M11 = I1 + I2 ...
        + m1*lc1^2 ...
        + m2*(L1^2 + lc2^2 + 2*L1*lc2*c2) ...
        + mp*(L1^2 + L2^2 + 2*L1*L2*c2);
        
    M12 = I2 ...
        + m2*(lc2^2 + L1*lc2*c2) ...
        + mp*(L2^2 + L1*L2*c2);
        
    M22 = I2 + m2*lc2^2 + mp*L2^2;
    
    h = -m2*L1*lc2*s2 - mp*L1*L2*s2;
    
    tau1(k) = M11*qdd1(k) + M12*qdd2(k) + h*(2*qd1(k)*qd2(k) + qd2(k)^2);
    tau2(k) = M12*qdd1(k) + M22*qdd2(k) - h*(qd1(k)^2);
end

% ---------------- GRAVITY-INDUCED BENDING MOMENT ----------------
% This is structural bending load from gravity, not z-axis actuator torque.
Mbase = zeros(nPts,1);
Mpayload = zeros(nPts,1);
Mlinks = zeros(nPts,1);

for k = 1:nPts
    th1 = q1(k);
    th12 = q1(k) + q2(k);
    
    r1 = [lc1*cos(th1), lc1*sin(th1)];
    r2 = [L1*cos(th1) + lc2*cos(th12), ...
          L1*sin(th1) + lc2*sin(th12)];
    rE = [L1*cos(th1) + L2*cos(th12), ...
          L1*sin(th1) + L2*sin(th12)];
          
    d1 = norm(r1);
    d2 = norm(r2);
    dE = norm(rE);
    
    Mlinks(k)   = g*(m1*d1 + m2*d2);
    Mpayload(k) = g*(mp*dE);
    Mbase(k)    = Mlinks(k) + Mpayload(k);
end

% ---------------- SUMMARY VALUES ----------------
[maxTau1, iTau1] = max(abs(tau1));
[maxTau2, iTau2] = max(abs(tau2));
[maxM, iM] = max(Mbase);

% ---------------- TRY TO GET WORKSPACE DIMENSIONS ----------------
% Falls back to reasonable defaults if these properties are not in your class.
if isprop(bagger, 'dx');   dx = bagger.dx;   else; dx = -0.5; end
if isprop(bagger, 'dy');   dy = bagger.dy;   else; dy = 0.2;  end
if isprop(bagger, 'wsW');  wsW = bagger.wsW; else; wsW = 1.0; end
if isprop(bagger, 'wsD');  wsD = bagger.wsD; else; wsD = 0.7; end

% ---------------- COLORS ----------------
cPath    = [0.10 0.65 0.30]; % Distinct emerald green to prevent color clashing
cReach   = [0.65 0.65 0.65];
cWS      = [0.20 0.20 0.20];
cStageF  = [0.87 0.93 1.00];
cStageE  = [0.00 0.45 0.74];
cBagF    = [0.97 0.89 0.80];
cBagE    = [0.85 0.33 0.10];
cJ1      = [0.00 0.45 0.74];
cJ2      = [0.85 0.33 0.10];
cTotal   = [0.00 0.45 0.74];
cLinks   = [0.85 0.33 0.10];
cPayload = [0.93 0.69 0.13];

% ---------------- FIGURE ----------------
% Adjusted to a tall vertical layout for 3x1 stacking
figure('Color','w','Position',[100 80 650 950]);

%% ===== Top: Workspace plot =====
subplot(3,1,1); hold on; grid on; axis equal;
set(gca, 'FontSize', 12); % Increases tick mark text size
title('Task Path in Workspace', 'FontWeight', 'bold', 'FontSize', 15);
xlabel('x [m]', 'FontSize', 13);
ylabel('y [m]', 'FontSize', 13);

% Reach envelope
th = linspace(0, 2*pi, 400);
rOuter = L1 + L2;
plot(rOuter*cos(th), rOuter*sin(th), '--', ...
    'Color', cReach, 'LineWidth', 1.2);

% Workspace outline
rectangle('Position',[dx, dy, wsW, wsD], ...
    'EdgeColor', cWS, 'LineWidth', 1.5);

% Staging area
rectangle('Position',[dx+0.1, dy+0.1, 0.5, 0.5], ...
    'FaceColor', cStageF, ...
    'EdgeColor', cStageE, ...
    'LineWidth', 1.5);

% Bag area
rectangle('Position',[dx+0.75, dy+0.2, 0.2, 0.3], ...
    'FaceColor', cBagF, ...
    'EdgeColor', cBagE, ...
    'LineWidth', 1.5);

% Task path
plot(xyzPath(:,1), xyzPath(:,2), ...
    'Color', cPath, 'LineWidth', 2.8);

% Base
plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
text(0.03, -0.02, 'Base', 'FontWeight', 'bold', 'FontSize', 12);

% Labels
text(dx+0.12, dy+0.54, 'Staging area', ...
    'Color', cStageE, 'FontWeight', 'bold', 'FontSize', 12);
text(dx+0.75, dy+0.55, 'Bag region', ...
    'Color', cBagE, 'FontWeight', 'bold', 'FontSize', 12);

xlim([-1.0 1.0]);
ylim([-0.2 1.2]);

%% ===== Middle: Torque plot =====
subplot(3,1,2); hold on; grid on;
set(gca, 'FontSize', 12); 
title('Planar Joint Drive Torque Estimate', 'FontWeight', 'bold', 'FontSize', 15);
xlabel('Path Sample', 'FontSize', 13);
ylabel('Torque [N·m]', 'FontSize', 13);

plot(tau1, 'Color', cJ1, 'LineWidth', 2.6, 'DisplayName', 'J1 torque');
plot(tau2, 'Color', cJ2, 'LineWidth', 2.6, 'DisplayName', 'J2 torque');

% Added 'HandleVisibility', 'off' to keep these out of the legend
plot(iTau1, tau1(iTau1), 'o', ...
    'MarkerSize', 8, 'LineWidth', 2, ...
    'MarkerEdgeColor', cJ1, 'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
plot(iTau2, tau2(iTau2), 'o', ...
    'MarkerSize', 8, 'LineWidth', 2, ...
    'MarkerEdgeColor', cJ2, 'MarkerFaceColor', 'w', 'HandleVisibility', 'off');

text(iTau1+4, tau1(iTau1)-0.15, ...
    sprintf('max |J1| = %.2f N·m', maxTau1), ...
    'Color', cJ1, 'FontWeight', 'bold', 'FontSize', 12);
text(iTau2+4, tau2(iTau2)-0.15, ...
    sprintf('max |J2| = %.2f N·m', maxTau2), ...
    'Color', cJ2, 'FontWeight', 'bold', 'FontSize', 12);
legend('Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 12);

%% ===== Bottom: Bending moment plot =====
subplot(3,1,3); hold on; grid on;
set(gca, 'FontSize', 12);
title('Gravity-Induced Base Bending Moment', 'FontWeight', 'bold', 'FontSize', 15);
xlabel('Path Sample', 'FontSize', 13);
ylabel('Moment [N·m]', 'FontSize', 13);

plot(Mbase,    'Color', cTotal,   'LineWidth', 2.8);
plot(Mlinks,   '--', 'Color', cLinks,   'LineWidth', 2.0);
plot(Mpayload, '--', 'Color', cPayload, 'LineWidth', 2.0);

plot(iM, Mbase(iM), 'o', ...
    'MarkerSize', 8, 'LineWidth', 2, ...
    'MarkerEdgeColor', cTotal, 'MarkerFaceColor', 'w');

text(iM+4, Mbase(iM)-0.2, sprintf('max = %.2f N·m', maxM), ...
    'Color', cTotal, 'FontWeight', 'bold', 'FontSize', 12);

% Direct labels
xLab = round(0.82*nPts);
text(xLab, Mbase(xLab)+0.15,    'Total',   'Color', cTotal,   'FontWeight', 'bold', 'FontSize', 12);
text(xLab, Mlinks(xLab)+0.15,   'Links',   'Color', cLinks,   'FontWeight', 'bold', 'FontSize', 12);
text(xLab, Mpayload(xLab)-0.35, 'Payload', 'Color', cPayload, 'FontWeight', 'bold', 'FontSize', 12);

% ---------------- PRINT VALUES FOR SLIDES ----------------
fprintf('\n--- Presentation Table Values ---\n');
fprintf('Max |J1 torque|:    %.2f N·m\n', maxTau1);
fprintf('Max |J2 torque|:    %.2f N·m\n', maxTau2);
fprintf('Max bending moment: %.2f N·m\n', maxM);
fprintf('Payload assumption: %.1f kg\n', mp);
fprintf('---------------------------------\n\n');