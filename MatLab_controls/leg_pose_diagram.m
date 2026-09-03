function leg_pose_diagram()
%LEG_POSE_DIAGRAM  Draws the actual 5-bar leg geometry (body-frame, not
%   tilted) for a few hip-motor commands, to sanity-check what a given
%   (phi1,phi4) pair physically looks like -- requested 2026-09-02 after
%   fk(p,-10deg,-10deg) came back with thL=48deg (leg swept forward), not
%   straight down as expected for a height setpoint.
%
%   Draws, side by side:
%     1) phi1=phi4=+16deg   (the "collapsed start" from collapse_recovery_demo)
%     2) phi1=phi4=-10deg   (the "height setpoint" as literally specified)
%     3) ik(p, L, 0) at the SAME L that (2) produced -- i.e. what a
%        genuinely straight-down leg looks like at that same length, for
%        direct comparison against (2)
%
%   Each panel plots A-B-C-D-E (hip-thigh-shin-shin-thigh-hip), the A-E hip
%   line (dashed), and marks the hip pivots + wheel/axle point C. Frame is
%   the leg's own body-relative (x-right, y-up) frame from fk()/ik() --
%   NOT NED, see FiveBarIK.hpp's header note -- this is deliberately a
%   "phi=0" picture (no body tilt) so the leg geometry alone is legible.

wb = wheeled_biped();
p  = wb.params();

cases = struct('name', {}, 'phi1', {}, 'phi4', {});
cases(1) = struct('name', 'collapsed start: phi1=phi4=+16deg', ...
                   'phi1', deg2rad(16), 'phi4', deg2rad(16));
cases(2) = struct('name', 'height setpoint: phi1=phi4=-10deg', ...
                   'phi1', deg2rad(-10), 'phi4', deg2rad(-10));

[L2, ~] = wb.fk(p, cases(2).phi1, cases(2).phi4);
[phi1s, phi4s] = wb.ik(p, L2, 0);   % genuinely straight-down at the SAME L as case 2
cases(3) = struct('name', sprintf('straight-down @ same L=%.3fm: ik(L,0)', L2), ...
                   'phi1', phi1s, 'phi4', phi4s);

figure('Position', [100 100 1500 550], 'Visible', 'off');
for i = 1:3
    subplot(1,3,i);
    drawOne(p, cases(i).phi1, cases(i).phi4, cases(i).name);
end

outFile = fullfile(fileparts(mfilename('fullpath')), 'leg_pose_diagram.png');
exportgraphics(gcf, outFile, 'Resolution', 150);
fprintf('plot saved to %s\n', outFile);

for i = 1:3
    [L0, thL] = wb.fk(p, cases(i).phi1, cases(i).phi4);
    fprintf('%-45s phi1=%7.2fdeg phi4=%7.2fdeg -> L0=%.4f m  thL=%6.2f deg\n', ...
            cases(i).name, rad2deg(cases(i).phi1), rad2deg(cases(i).phi4), L0, rad2deg(thL));
end
end

function drawOne(p, phi1, phi4, ttl)
wb = wheeled_biped();
A = [-p.l5/2; 0];
E = [ p.l5/2; 0];
B = A - p.l1*[cos(phi1); -sin(phi1)];   % phi1 zero points -x, CW-positive -- see fk()'s header comment
D = E + p.l4*[cos(phi4);  sin(phi4)];   % phi4 zero points +x, CCW-positive -- see fk()'s header comment
[~, ~, C] = wb.fk(p, phi1, phi4);

hold on; axis equal; grid on;
plot([A(1) E(1)], [A(2) E(2)], 'k--');            % hip line
plot([A(1) B(1) C(1) D(1) E(1)], [A(2) B(2) C(2) D(2) E(2)], ...
     '-o', 'Color', [0.1 0.4 0.8], 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'w');
th = linspace(0, 2*pi, 60);
plot(C(1) + p.R*cos(th), C(2) + p.R*sin(th), 'Color', [0.6 0.6 0.6]);   % wheel outline
plot(A(1), A(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(E(1), E(2), 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
text(A(1), A(2)-0.03, 'A (rear hip)', 'HorizontalAlignment', 'center', 'FontSize', 8);
text(E(1), E(2)-0.03, 'E (front hip)', 'HorizontalAlignment', 'center', 'FontSize', 8);
text(C(1), C(2)-p.R-0.02, 'C (wheel/axle)', 'HorizontalAlignment', 'center', 'FontSize', 8);
yline(0, ':', 'Color', [0.7 0.7 0.7], 'HandleVisibility', 'off');
xlim([-0.35 0.35]); ylim([-0.4 0.15]);
xlabel('x [m]'); ylabel('y [m] (up)');
title(ttl, 'FontSize', 9, 'Interpreter', 'none');
end
