function plot_response(controller)
%PLOT_RESPONSE  Live, looping closed-loop simulation.
%
%  plot_response()          -- runs the LQR (default)
%  plot_response('lqr')     -- gain-scheduled LQR (wb.simulate)
%  plot_response('pid')     -- SLC PID cascade + hip lock (wb.simulatePid) --
%                               edit the `gains` struct in section 1 below to
%                               tune it; each field maps 1:1 to a constexpr in
%                               src/controllers/PidBalanceController.hpp /
%                               HipLock.hpp -- see wb.defaultPidGains's help.
%  plot_response('collapse') -- VELOCITY-CONTROLLED-WHEEL scenario (wb.simulateVel,
%                               discrete 100 Hz LQR, matches CANMotor.cpp's ODrive
%                               velocity mode): starts fully collapsed and tipped
%                               over 30 deg, balances through a height-setpoint leg
%                               move, then tracks a square-wave velocity command --
%                               same scenario as collapse_recovery_demo.m (the
%                               headless/batch version), live-animated here instead.
%                               Edit section 1's 'collapse' case to change the
%                               angles/timing/amplitude.
%
%  'lqr'/'pid' balance the full nonlinear plant while tracking a sinusoidal
%  forward-velocity target; 'collapse' runs the scenario above. All three
%  inject realistic sensor noise (and a small sensor/CAN/compute delay)
%  between the TRUE plant state and what the controller actually acts on --
%  see wb.simulate's/wb.simulateVel's opts.noiseStd / opts.delay. Runs
%  indefinitely, advancing in short chunks and redrawing live, until you
%  close either figure window (or Ctrl+C).
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

velMode = strcmp(controller, 'collapse');   % velocity-controlled-wheel model
                                             % (5-state, no x) vs the 6-state
                                             % torque model 'lqr'/'pid' use --
                                             % branched throughout below

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
case 'collapse'
    % Requested 2026-09-02 -- see collapse_recovery_demo.m for the headless/
    % batch version of this same scenario (that file has the fuller write-up
    % on the phi1/phi4 convention this depends on -- front CCW-positive,
    % rear CW-positive, confirmed 2026-09-02). "-10 deg on both hips" is a
    % genuinely TALLER stance than "+16 deg" (L=0.152m vs 0.098m), matching
    % the height-setpoint framing below as intended.
    PHI1_START_DEG  = 16;   PHI4_START_DEG  = 16;    % collapsed pose
    PHI1_TARGET_DEG = -10;  PHI4_TARGET_DEG = -10;   % height setpoint
    PHI0_DEG  = 30;    % initial body tip                              [deg]
    T_HEIGHT  = 4.0;   % height-setpoint phase duration                [s]
    TAU_L     = 1.0;   % leg-extension time constant (exp crossfade)   [s]
    VEL_AMP   = 0.3;   % square-wave velocity amplitude                [m/s]
    VEL_FREQ  = 0.1;   % square-wave frequency                        [Hz]
    dtCk      = 1/100; % matches main.cpp's ControlThread rate

    [L_START, thL_start] = wb.fk(p, deg2rad(PHI1_START_DEG),  deg2rad(PHI4_START_DEG));
    [L_CMD,   ~]          = wb.fk(p, deg2rad(PHI1_TARGET_DEG), deg2rad(PHI4_TARGET_DEG));
    fprintf('collapsed start : phi1=phi4=%+.0fdeg -> L0=%.4f m\n', PHI1_START_DEG, L_START);
    fprintf('height setpoint : phi1=phi4=%+.0fdeg -> L0=%.4f m\n', PHI1_TARGET_DEG, L_CMD);

    Qv = diag([40 2 800 8]);   % [theta, thetadot, phi, phidot] -- placeholder, see wheeled_biped.m
    Rv = diag([1 3]);          % [ax, Tp]                         -- placeholder, see wheeled_biped.m
    Lgrid = linspace(min(0.08, L_CMD*0.9), 0.34, 25);   % widened to cover L_CMD, not just [0.16,0.34]
    sched = wb.schedule(p, Qv, Rv, Lgrid, 3, 'velwheel', dtCk);

    phi0Coll   = deg2rad(PHI0_DEG);
    theta0Coll = phi0Coll - thL_start;
otherwise
    error('plot_response:controller', 'controller must be ''lqr'', ''pid'', or ''collapse''');
end

%% ---- 2. velocity target + sensor model -----------------------------------
if velMode
    % square-wave height-then-velocity scenario -- see section 1's
    % 'collapse' case for the constants (T_HEIGHT, TAU_L, VEL_AMP, VEL_FREQ)
    Lref = @(tAbs) L_CMD + (L_START - L_CMD) * exp(-max(tAbs,0) / TAU_L);
    vRef = @(tAbs) (tAbs >= T_HEIGHT) .* VEL_AMP .* sign(sin(2*pi*VEL_FREQ*(tAbs - T_HEIGHT)));
    uLim = [20, 40];   % [ax_max (m/s^2), Thip_max (N.m)] -- NOT torque, see linearModelVel's header
    simOpts = struct();
    simOpts.dt       = dtCk;
    simOpts.noiseStd = [0.005, 0.005, 0.08, 0.05, 0.05];   % theta,phi,xdot,thetadot,phidot
    simOpts.delay    = 0.00;
    s0 = [theta0Coll; phi0Coll; 0; 0; 0];   % [theta;phi;xdot;thetadot;phidot], at rest
    s0Init  = s0;    % saved so the whole scenario can restart from it -- see section 5's loopSec
    loopSec = 10;     % restart the scenario (state AND reference-function phase) every this many
                       % seconds of simulated time, so it repeats as a demo loop instead of
                       % settling once and tracking the square wave forever
else
    L    = 0.2;                             % leg length held constant this run
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
    s0 = [0; 0; 0; 0; 0; 0];
end

%% ---- 3. figures: rolling time-series + live 2D animation -----------------
histSec = 8;                             % rolling window shown in the time-series plot
nHist   = round(histSec / simOpts.dt);

figTS = figure('Name', sprintf('Response (live) -- %s', upper(controller)), 'Position', [50 50 900 700]);
axV = subplot(3,1,1); hold(axV, 'on'); grid(axV, 'on');
hVref = plot(axV, nan, nan, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'v_{ref}');
hVact = plot(axV, nan, nan, 'Color', [0.1 0.4 0.8], 'LineWidth', 1.5, 'DisplayName', 'v_{actual}');
ylabel(axV, 'xdot [m/s]'); legend(axV, 'Location', 'best');
if velMode
    title(axV, sprintf('[%s] Collapsed start, %d deg tip-over, height setpoint then %.1f Hz square-wave velocity', ...
        upper(controller), PHI0_DEG, VEL_FREQ));
else
    title(axV, sprintf('[%s] Tracking a %.1f Hz, %.2f m/s sinusoidal velocity target (note the phase lag -- see help)', ...
        upper(controller), fHz, vAmp));
end

axAng = subplot(3,1,2); hold(axAng, 'on'); grid(axAng, 'on');
hTh = plot(axAng, nan, nan, 'LineWidth', 1.5, 'DisplayName', '\theta (leg)');
hPh = plot(axAng, nan, nan, 'LineWidth', 1.5, 'DisplayName', '\phi (body)');
ylabel(axAng, 'angle [deg]'); legend(axAng, 'Location', 'best');

axU = subplot(3,1,3); hold(axU, 'on'); grid(axU, 'on');
if velMode
    hUw = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'a_x [m/s^2]');
    hUp = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'T_{hip} [N.m]');
    ylabel(axU, 'u');
else
    hUw = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'T_{wheel}');
    hUp = plot(axU, nan, nan, 'LineWidth', 1.5, 'DisplayName', 'T_{hip}');
    ylabel(axU, 'torque [N.m]');
end
xlabel(axU, 'time [s]'); legend(axU, 'Location', 'best');

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
histT = zeros(0,1); histS = zeros(0,3); histU = zeros(0,2);   % [theta,phi,xdot] normalized
lastDraw = -inf;
frameDt  = 1/30;
xAcc = 0;   % integrated wheel position, velMode only -- s has no x state
            % there (see linearModelVel's header), so this is a LOCAL
            % accumulator purely for animation/camera-following, not fed
            % back into the dynamics or controller.

    function ok = drawFrame(tAbs, s, u)
        % Nested function: shares plot_response's workspace directly (all
        % the handles and history buffers above), so it can update them
        % with no arguments beyond the new (t, s, u) sample.
        ok = true;
        if ~ishandle(figTS) || ~ishandle(figAnim)
            return
        end

        if velMode
            thetaNow = s(1); phiNow = s(2); xdotNow = s(3);
            xAcc  = xAcc + xdotNow * simOpts.dt;   % integrate every call, not just drawn frames
            xNow  = xAcc;
            Lnow  = Lref(tAbs);
        else
            xNow = s(1); thetaNow = s(2); phiNow = s(3); xdotNow = s(4);
            Lnow = L;
        end

        if (tAbs - lastDraw) < frameDt
            return
        end
        lastDraw = tAbs;

        histT(end+1,1) = tAbs;
        histS(end+1,:) = [thetaNow, phiNow, xdotNow];
        histU(end+1,:) = u.';
        if numel(histT) > nHist
            histT(1)   = [];
            histS(1,:) = [];
            histU(1,:) = [];
        end

        set(hVref, 'XData', histT, 'YData', vRef(histT));
        set(hVact, 'XData', histT, 'YData', histS(:,3));
        set(hTh,   'XData', histT, 'YData', rad2deg(histS(:,1)));
        set(hPh,   'XData', histT, 'YData', rad2deg(histS(:,2)));
        set(hUw,   'XData', histT, 'YData', histU(:,1));
        set(hUp,   'XData', histT, 'YData', histU(:,2));
        if numel(histT) > 1
            xlim(axV,   [histT(1) histT(end)]);
            xlim(axAng, [histT(1) histT(end)]);
            xlim(axU,   [histT(1) histT(end)]);
        end

        pts = wb.robotFrame(wb, p, Lnow, thetaNow, phiNow, xNow);

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

        set(titleH, 'String', sprintf('t = %6.2f s   v_{ref} = %+5.2f m/s   v = %+5.2f m/s   \\theta = %5.1f deg   \\phi = %5.1f deg   L = %.3f m', ...
            tAbs, vRef(tAbs), xdotNow, rad2deg(thetaNow), rad2deg(phiNow), Lnow));

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
% forward into vRef/stepCallback -- EXCEPT for 'collapse', which restarts
% the whole scenario (state s0 AND tElapsed, so the height-setpoint/
% square-wave phase timing restarts too, not just the physical state)
% every loopSec seconds, turning the one-shot recovery scenario into a
% repeating demo loop -- see section 2's loopSec/s0Init.
% s0 already set per-controller in section 2 (5-state for 'collapse', 6-state otherwise)
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
    case 'collapse'
        Lchunk = @(tRel) Lref(t0 + tRel);   % chunk-relative wrapper, matches simOpts.vRef's pattern
        [~, S] = wb.simulateVel(p, sched, Lchunk, s0, chunkSec, uLim, simOpts);
    end

    s0       = S(end,:).';
    tElapsed = tElapsed + chunkSec;

    if velMode && tElapsed >= loopSec
        % Restart the scenario from scratch: physical state back to the
        % collapsed/tipped initial condition, elapsed time back to 0 (so
        % Lref/vRef re-run their own t=0..T_HEIGHT.. phase rather than
        % just continuing the square wave from wherever it left off), and
        % the animation's rolling history/camera accumulator cleared so
        % there's no visual jump-cut artifact in the time-series plots.
        s0       = s0Init;
        tElapsed = 0;
        histT = zeros(0,1); histS = zeros(0,3); histU = zeros(0,2);
        lastDraw = -inf;
        xAcc = 0;
    end
end
fprintf('Animation window closed -- stopped after %.1f s simulated.\n', tElapsed);
end
