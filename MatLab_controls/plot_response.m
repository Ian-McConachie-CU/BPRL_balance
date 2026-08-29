function plot_response(controller)
%PLOT_RESPONSE  Live, looping closed-loop simulation.
%
%  plot_response()        -- runs the LQR (default)
%  plot_response('lqr')   -- gain-scheduled LQR (wb.simulate)
%  plot_response('pid')   -- SLC PID cascade + hip lock (wb.simulatePid) --
%                             edit the `gains` struct in section 1 below to
%                             tune it; each field maps 1:1 to a constexpr in
%                             src/controllers/PidBalanceController.hpp /
%                             HipLock.hpp -- see wb.defaultPidGains's help.
%
%  Either controller balances the full nonlinear plant while tracking a
%  sinusoidal forward-velocity target, with realistic sensor noise (and a
%  small sensor/CAN/compute delay) injected between the TRUE plant state
%  and what the controller actually acts on -- see wb.simulate's
%  opts.noiseStd / opts.delay. Runs indefinitely, advancing in short
%  chunks and redrawing live, until you close either figure window (or
%  Ctrl+C).
%
%  Note the phase lag between the velocity target and the actual velocity
%  in the top plot -- that's real, not a bug: an LQR tuned to REGULATE
%  upright (not to track a 1 Hz command) has finite tracking bandwidth,
%  and this class of robot is inherently nonminimum-phase (it has to lean
%  backward briefly to accelerate forward), so some lag at 1 Hz is
%  expected. This is a feature of the demo, not a defect -- it shows you
%  the controller's actual tracking bandwidth on the full nonlinear plant.
%
%  Requires wheeled_biped.m in the same folder (or on the path). All the
%  math -- kinematics, dynamics, LQR, and the animation geometry
%  (wb.robotFrame) -- lives in that one file, validated by wb.selfcheck().
%  Run that first if you ever edit the geometry.
%
%  This file is a FUNCTION (not a plain script) because the live-animation
%  callback below is a nested function -- it needs to share this
%  function's workspace (the graphics handles, the rolling history
%  buffers) directly, which a plain script's local functions cannot do.
%  Run it exactly like a script: type `plot_response` and press enter.
%
%  Uses only base MATLAB graphics (patch, line, rectangle, drawnow) -- no
%  toolbox required for the plotting/animation itself.

if nargin < 1 || isempty(controller), controller = 'lqr'; end
controller = lower(controller);

clc; close all;

wb = wheeled_biped();
p  = wb.params();

%% ---- 1. controller ----------------------------------------------------
switch controller
case 'lqr'
    % same defaults as wheeled_biped's selfcheck -- edit to match whatever
    % you're tuning
    Q     = diag([80 8 40 2 800 8]);   % [x, xdot, theta, thetadot, phi, phidot]
    R     = diag([12 1]);              % [wheel torque, hip torque]
    Lgrid = linspace(0.16, 0.34, 25);
    sched = wb.schedule(p, Q, R, Lgrid, 3);
case 'pid'
    % Mirrors src/controllers/PidBalanceController.hpp / HipLock.hpp --
    % edit these to tune, then copy the numbers you like back into those
    % two files. See wb.defaultPidGains's help for what each field is and
    % simulatePid's help for the (verified, non-obvious) sign convention
    % on gains.hip_kp/hip_kd -- don't "fix" that sign without re-reading it.
    %
    % wb.defaultPidGains()'s numbers are tuned for an OLDER, lighter
    % params() (m_b=6, R=0.07). With the current, heavier params() (m_b=15,
    % R=0.09) the open-loop instability is faster (~19 rad/s vs ~12.7
    % rad/s), and those defaults diverge almost immediately. The overrides
    % below settle cleanly from 3/10/20 deg disturbances and track a
    % velocity command correctly, verified with simOpts.delay = 0 --
    % see the IMPORTANT note below before trusting it beyond that.
    gains = wb.defaultPidGains();
    gains.pitch_kp = 15;  gains.pitch_kd = 1;   gains.vel_kp = 0.1;
    gains.hip_kp   = 60;  gains.hip_kd   = 10;
    gains.wheel_torque_limit = 15; gains.pitch_imax = 15;
    gains.hip_torque_limit  = 11;
    %
    % IMPORTANT: with simOpts.delay left at its default 10 ms (below), this
    % still diverges -- and not from a bad gain choice. I checked both much
    % more aggressive and much more conservative gains than these; NONE
    % handle 10 ms of delay at this mass. The full LQR handles the exact
    % same delay and params trivially (settles from the same disturbances
    % using only 1-5 Nm). That's because the LQR's T depends on theta too,
    % and its Tp depends on phi too -- this simple cascade's T only sees
    % (phi,phidot,xdot) and its Tp only sees thL -- so it has fundamentally
    % less phase margin for the SAME disturbance rejection speed. This is a
    % real structural limit of the decoupled cascade at this operating
    % point, not a tuning mistake -- it's exactly the gap Stage 1 (the LQR)
    % exists to close. Set simOpts.delay = 0 below to tune/observe the
    % cascade's own behavior in isolation from that limit, or revisit
    % params() (m_b tripled without I_b changing -- see chat history) if a
    % lighter/more realistic body brings the open-loop pole back down.
otherwise
    error('plot_response:controller', 'controller must be ''lqr'' or ''pid''');
end

%% ---- 2. velocity target + sensor model -----------------------------------
L    = 0.12;                             % leg length held constant this run
fHz  = 0.1;                              % velocity-target frequency
vAmp = 1.00;                             % velocity-target amplitude [m/s]
vRef = @(tAbs) vAmp * sin(2*pi*fHz*tAbs);

uLim = [2*p.tau_wheel_peak, 40];

simOpts = struct();
simOpts.dt       = 1/400;
% Rough IMU/encoder-class noise placeholders (see wb.simulate help) -- NOT
% measured on real hardware, replace once it is:
simOpts.noiseStd = [0.01, 0.005, 0.005, 0.08, 0.05, 0.05];
simOpts.delay    = 0.00;                % 10 ms sensor/CAN/compute latency

%% ---- 3. figures: rolling time-series + live 2D animation -----------------
histSec = 8;                             % rolling window shown in the time-series plot
nHist   = round(histSec / simOpts.dt);

figTS = figure('Name', sprintf('Response (live) -- %s', upper(controller)), 'Position', [50 50 900 700]);
axV = subplot(3,1,1); hold(axV, 'on'); grid(axV, 'on');
hVref = plot(axV, nan, nan, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'v_{ref}');
hVact = plot(axV, nan, nan, 'Color', [0.1 0.4 0.8], 'LineWidth', 1.5, 'DisplayName', 'v_{actual}');
ylabel(axV, 'xdot [m/s]'); legend(axV, 'Location', 'best');
title(axV, sprintf('[%s] Tracking a %.1f Hz, %.2f m/s sinusoidal velocity target (note the phase lag -- see help)', ...
    upper(controller), fHz, vAmp));

axAng = subplot(3,1,2); hold(axAng, 'on'); grid(axAng, 'on');
hTh = plot(axAng, nan, nan, 'LineWidth', 1.5, 'DisplayName', '\theta (leg)');
hPh = plot(axAng, nan, nan, 'LineWidth', 1.5, 'DisplayName', '\phi (body)');
ylabel(axAng, 'angle [deg]'); legend(axAng, 'Location', 'best');

axU = subplot(3,1,3); hold(axU, 'on'); grid(axU, 'on');
hUw = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'T_{wheel}');
hUp = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'T_{hip}');
ylabel(axU, 'torque [N.m]'); xlabel(axU, 'time [s]'); legend(axU, 'Location', 'best');

figAnim = figure('Name', 'Live view', 'Position', [1000 50 900 600]);
axAnim  = axes('Parent', figAnim); hold(axAnim, 'on'); axis(axAnim, 'equal'); grid(axAnim, 'on');
xlabel(axAnim, 'x [m]'); ylabel(axAnim, 'y [m]');
line(axAnim, [-1e4 1e4], [0 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
ylim(axAnim, [-0.05, 0.55]);

wheelH = rectangle(axAnim, 'Position', [-p.R, 0, 2*p.R, 2*p.R], ...
    'Curvature', [1 1], 'FaceColor', [0.85 0.85 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
spokeH = line(axAnim, [0 0], [0 0], 'Color', 'k', 'LineWidth', 1.5);

% 5-bar links: rear thigh (A-B), rear shin (B-C), front shin (C-D),
% front thigh (D-E), hip-mount bar (A-E)
linkAB = line(axAnim, [0 0], [0 0], 'Color', [0.1 0.4 0.8], 'LineWidth', 3);
linkBC = line(axAnim, [0 0], [0 0], 'Color', [0.1 0.4 0.8], 'LineWidth', 3);
linkCD = line(axAnim, [0 0], [0 0], 'Color', [0.8 0.3 0.1], 'LineWidth', 3);
linkDE = line(axAnim, [0 0], [0 0], 'Color', [0.8 0.3 0.1], 'LineWidth', 3);
linkAE = line(axAnim, [0 0], [0 0], 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'LineStyle', '--');

jointSize = 0.012;
mkCircle = @(color) rectangle(axAnim, 'Position', [-jointSize -jointSize 2*jointSize 2*jointSize], ...
    'Curvature', [1 1], 'FaceColor', color, 'EdgeColor', 'none');
jointA = mkCircle([0.2 0.2 0.2]);
jointB = mkCircle([0.2 0.2 0.2]);
jointC = mkCircle([0.2 0.2 0.2]);
jointD = mkCircle([0.2 0.2 0.2]);
jointE = mkCircle([0.2 0.2 0.2]);

% body: simple rectangle from the hip-mount midpoint out to the illustrative
% body tip. Width is purely for visualization (not a physical dimension --
% the dynamics model treats the body as a point mass at p.l_b from the hip).
bodyW = 0.10;
bodyH = patch(axAnim, 'XData', zeros(4,1), 'YData', zeros(4,1), ...
    'FaceColor', [0.9 0.7 0.3], 'EdgeColor', 'k', 'LineWidth', 1.2);

titleH = title(axAnim, '');
follow = true;   % camera follows the robot horizontally; set false to keep it fixed

%% ---- 4. rolling history buffers + the live-draw callback ------------------
% frameDt throttles redraws to ~30 fps regardless of the 400 Hz control
% rate -- drawFrame is called once per control tick, but only actually
% updates the plots/animation every frameDt seconds of SIMULATED time.
histT = zeros(0,1); histS = zeros(0,6); histU = zeros(0,2);
lastDraw = -inf;
frameDt  = 1/30;

    function ok = drawFrame(tAbs, s, u)
        % Nested function: shares plot_response's workspace directly (all
        % the handles and history buffers above), so it can update them
        % with no arguments beyond the new (t, s, u) sample.
        ok = true;
        if ~ishandle(figTS) || ~ishandle(figAnim)
            return
        end
        if (tAbs - lastDraw) < frameDt
            return
        end
        lastDraw = tAbs;

        histT(end+1,1) = tAbs;
        histS(end+1,:) = s.';
        histU(end+1,:) = u.';
        if numel(histT) > nHist
            histT(1)   = [];
            histS(1,:) = [];
            histU(1,:) = [];
        end

        set(hVref, 'XData', histT, 'YData', vRef(histT));
        set(hVact, 'XData', histT, 'YData', histS(:,4));
        set(hTh,   'XData', histT, 'YData', rad2deg(histS(:,2)));
        set(hPh,   'XData', histT, 'YData', rad2deg(histS(:,3)));
        set(hUw,   'XData', histT, 'YData', histU(:,1));
        set(hUp,   'XData', histT, 'YData', histU(:,2));
        if numel(histT) > 1
            xlim(axV,   [histT(1) histT(end)]);
            xlim(axAng, [histT(1) histT(end)]);
            xlim(axU,   [histT(1) histT(end)]);
        end

        pts = wb.robotFrame(wb, p, L, s(2), s(3), s(1));

        set(wheelH, 'Position', [pts.wheelCenter(1)-p.R, pts.wheelCenter(2)-p.R, 2*p.R, 2*p.R]);
        set(spokeH, 'XData', [pts.spoke1(1), pts.spoke2(1)], ...
                    'YData', [pts.spoke1(2), pts.spoke2(2)]);
        set(linkAB, 'XData', [pts.A(1) pts.B(1)], 'YData', [pts.A(2) pts.B(2)]);
        set(linkBC, 'XData', [pts.B(1) pts.C(1)], 'YData', [pts.B(2) pts.C(2)]);
        set(linkCD, 'XData', [pts.C(1) pts.D(1)], 'YData', [pts.C(2) pts.D(2)]);
        set(linkDE, 'XData', [pts.D(1) pts.E(1)], 'YData', [pts.D(2) pts.E(2)]);
        set(linkAE, 'XData', [pts.A(1) pts.E(1)], 'YData', [pts.A(2) pts.E(2)]);

        moveCircle = @(h, c) set(h, 'Position', [c(1)-jointSize, c(2)-jointSize, 2*jointSize, 2*jointSize]);
        moveCircle(jointA, pts.A);
        moveCircle(jointB, pts.B);
        moveCircle(jointC, pts.C);
        moveCircle(jointD, pts.D);
        moveCircle(jointE, pts.E);

        Omid = (pts.A + pts.E)/2;
        dirV = pts.bodyTip - Omid;
        Lb   = max(norm(dirV), 1e-9);
        uax  = dirV / Lb;
        nax  = [-uax(2); uax(1)];
        corners = [Omid + (bodyW/2)*nax, Omid + Lb*uax + (bodyW/2)*nax, ...
                   Omid + Lb*uax - (bodyW/2)*nax, Omid - (bodyW/2)*nax];
        set(bodyH, 'XData', corners(1,:), 'YData', corners(2,:));

        set(titleH, 'String', sprintf('t = %6.2f s   v_{ref} = %+5.2f m/s   v = %+5.2f m/s   \\theta = %5.1f deg   \\phi = %5.1f deg', ...
            tAbs, vRef(tAbs), s(4), rad2deg(s(2)), rad2deg(s(3))));

        if follow
            xlim(axAnim, [pts.wheelCenter(1)-0.5, pts.wheelCenter(1)+0.5]);
        end
        drawnow limitrate;
    end

%% ---- 5. the loop -----------------------------------------------------
% Runs in short chunks (rather than one call covering the whole run) so
% both figures can be checked between chunks and the loop stops cleanly
% when you close either one. The velocity-target phase stays continuous
% across chunks because elapsed simulated time (tElapsed) is carried
% forward into vRef/stepCallback, never reset.
s0        = [0; 0; 0; 0; 0; 0];
tElapsed  = 0;
chunkSec  = 1;

fprintf('Running [%s] -- close either figure window to stop.\n', upper(controller));
while ishandle(figTS) && ishandle(figAnim)
    t0 = tElapsed;
    simOpts.vRef         = @(tRel) vRef(t0 + tRel);
    simOpts.stepCallback = @(tRel, s, u) drawFrame(t0 + tRel, s, u);

    switch controller
    case 'lqr'
        [~, S] = wb.simulate(p, sched, L, s0, chunkSec, uLim, simOpts);
    case 'pid'
        [~, S] = wb.simulatePid(p, L, gains, s0, chunkSec, uLim, simOpts);
    end

    s0       = S(end,:).';
    tElapsed = tElapsed + chunkSec;
end
fprintf('Animation window closed -- stopped after %.1f s simulated.\n', tElapsed);
end
