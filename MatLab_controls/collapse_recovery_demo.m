function [t, S, U, Lref] = collapse_recovery_demo(makePlot)
%COLLAPSE_RECOVERY_DEMO  Scenario requested 2026-09-02: robot starts fully
%   collapsed (both hip motors commanded to +16 deg, front CCW-positive /
%   rear CW-positive convention -- see wheeled_biped.m's CONVENTIONS
%   block) and tipped over 30 deg. It then:
%     - t in [0, T_HEIGHT)  : balances to zero pitch / zero velocity while
%                             the legs extend toward a height setpoint of
%                             -10 deg on both hip motors
%     - t >= T_HEIGHT        : tracks a square-wave forward-velocity
%                             reference, VEL_FREQ Hz, +/-VEL_AMP m/s
%   Uses the VELOCITY-CONTROLLED-WHEEL reduced model (linearModelVel /
%   simulateVel, discrete 100 Hz LQR) -- matches CANMotor.cpp's ODrive
%   velocity-mode wheel actuation, NOT PidBalanceController's torque
%   cascade. Hips stay torque-controlled (Tp), unchanged.
%
%   KINEMATIC NOTE: phi1's zero reference was fixed 2026-09-02 (see fk()'s
%   header comment), and phi1=phi4=X is now an EXACT straight-down pose
%   (thL=0) for every X, confirmed in wheeled_biped.m's selfcheck(). With
%   the CW/CCW assignment also corrected the same day (front CCW-positive,
%   rear CW-positive), L0 now grows monotonically as X DECREASES:
%   phi1=phi4=+16deg -> L0=0.098m, -10deg -> L0=0.152m -- so -10deg genuinely
%   is a TALLER stance than the +16deg "collapsed start", matching the
%   "height setpoint" framing below as intended.
%
%   L(t) is treated as an idealized, instantaneously-tracked SCHEDULE
%   PARAMETER (exponential crossfade, time constant TAU_L) -- see
%   simulateVel()'s header for why: there is no separately modeled leg-
%   length actuator/inertia in this file (controls_plan.md section 6 --
%   a real height controller is its own design problem). This is fine for
%   seeing how the gain-scheduled balance loop rides through a commanded
%   height change, not a validated model of the crouch/extend transient
%   itself.
%
%   [t,S,U,Lref] = collapse_recovery_demo()      run + plot (default)
%   [t,S,U,Lref] = collapse_recovery_demo(false)  run, no plot
%   S columns: [theta, phi, xdot, thetadot, phidot] (rad, rad, m/s, rad/s, rad/s)
%   U columns: [ax (m/s^2), Tp (N.m)]
%   Lref: @(t) -> commanded leg length [m], same trajectory fed to the sim

if nargin < 1, makePlot = true; end

wb = wheeled_biped();
p  = wb.params();

% ---- scenario constants ----
PHI1_START_DEG = 16;    PHI4_START_DEG = 16;    % collapsed pose
PHI1_TARGET_DEG = -10;  PHI4_TARGET_DEG = -10;  % height setpoint
PHI0_DEG   = 30;    % initial body tip
T_HEIGHT   = 4.0;   % height-setpoint phase duration                 [s]
TAU_L      = 1.0;   % leg-extension time constant (exp crossfade)    [s]
VEL_AMP    = 0.3;   % square-wave velocity amplitude                 [m/s]
VEL_FREQ   = 0.1;   % square-wave frequency                          [Hz]
N_PERIODS  = 2;      % how many square-wave periods to show after T_HEIGHT
dtCk       = 1/100; % matches main.cpp's ControlThread rate

% ---- leg geometry: collapsed start and height-setpoint target ----
[L_start, thL_start] = wb.fk(p, deg2rad(PHI1_START_DEG),  deg2rad(PHI4_START_DEG));
[L_cmd,   thL_cmd]   = wb.fk(p, deg2rad(PHI1_TARGET_DEG), deg2rad(PHI4_TARGET_DEG));
fprintf('collapsed start : phi1=phi4=%+.0fdeg -> L0=%.4f m, thL=%.2f deg\n', ...
        PHI1_START_DEG, L_start, rad2deg(thL_start));
fprintf('height setpoint : phi1=phi4=%+.0fdeg -> L0=%.4f m, thL=%.2f deg\n', ...
        PHI1_TARGET_DEG, L_cmd, rad2deg(thL_cmd));

% ---- initial condition: theta = phi - thL (NED sign convention) ----
phi0   = deg2rad(PHI0_DEG);
theta0 = phi0 - thL_start;
s0 = [theta0; phi0; 0; 0; 0];   % [theta;phi;xdot;thetadot;phidot], at rest

% ---- reference trajectories ----
Lref = @(t) L_cmd + (L_start - L_cmd) * exp(-max(t,0) / TAU_L);
vRef = @(t) (t >= T_HEIGHT) .* VEL_AMP .* sign(sin(2*pi*VEL_FREQ*(t - T_HEIGHT)));

% ---- gain-scheduled discrete LQR (velocity-controlled wheel) ----
Qv = diag([40 2 800 8]);   % [theta, thetadot, phi, phidot] -- placeholder, see wheeled_biped.m
Rv = diag([1 3]);          % [ax, Tp]                         -- placeholder, see wheeled_biped.m
sched = wb.schedule(p, Qv, Rv, linspace(0.08, 0.34, 25), 3, 'velwheel', dtCk);   % widened
                                                                                   % to cover L_cmd
                                                                                   % (~0.107m, below
                                                                                   % the old 0.16m
                                                                                   % floor -- was
                                                                                   % extrapolating)

tf = T_HEIGHT + N_PERIODS / VEL_FREQ;
opts = struct('dt', dtCk, 'noiseStd', zeros(1,5), 'vRef', vRef);
[t, S, U] = wb.simulateVel(p, sched, Lref, s0, tf, [], opts);

fprintf('\nfinal state: theta=%.2fdeg phi=%.2fdeg xdot=%.3f m/s (t=%.1fs)\n', ...
        rad2deg(S(end,1)), rad2deg(S(end,2)), S(end,3), t(end));
maxAbsTheta = rad2deg(max(abs(S(:,1))));
maxAbsPhi   = rad2deg(max(abs(S(:,2))));
fprintf('worst excursion over the run: |theta|max=%.1fdeg |phi|max=%.1fdeg\n', ...
        maxAbsTheta, maxAbsPhi);

if ~makePlot, return; end

Lt = arrayfun(Lref, t);
vt = arrayfun(vRef, t);

figure('Position', [100 100 900 900], 'Visible', 'off');

subplot(4,1,1);
plot(t, rad2deg(S(:,1)), 'b', t, rad2deg(S(:,2)), 'r', 'LineWidth', 1.2);
ylabel('deg'); legend('\theta (leg)', '\phi (body)', 'Location', 'best');
title('Collapsed-start / 30 deg tip-over recovery, height setpoint then 0.1 Hz square-wave velocity');
grid on; xline(T_HEIGHT, '--k', 'HandleVisibility', 'off');

subplot(4,1,2);
plot(t, S(:,3), 'b', t, vt, 'k--', 'LineWidth', 1.2);
ylabel('m/s'); legend('xdot (actual)', 'vRef', 'Location', 'best');
grid on; xline(T_HEIGHT, '--k', 'HandleVisibility', 'off');

subplot(4,1,3);
plot(t, Lt, 'm', 'LineWidth', 1.2);
ylabel('L [m]'); grid on; xline(T_HEIGHT, '--k', 'HandleVisibility', 'off');

subplot(4,1,4);
plot(t, U(:,1), 'b', t, U(:,2), 'r', 'LineWidth', 1.2);
ylabel('u'); xlabel('t [s]');
legend('ax [m/s^2]', 'Tp [N.m]', 'Location', 'best');
grid on; xline(T_HEIGHT, '--k', 'HandleVisibility', 'off');

outFile = fullfile(fileparts(mfilename('fullpath')), 'collapse_recovery_demo.png');
exportgraphics(gcf, outFile, 'Resolution', 150);
fprintf('\nplot saved to %s\n', outFile);
end
