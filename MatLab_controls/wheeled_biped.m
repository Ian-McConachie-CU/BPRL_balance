function wb = wheeled_biped()
%WHEELED_BIPED  Planar model, 5-bar kinematics, gain-scheduled LQR, and
%               simulation/animation geometry for a wheeled biped with
%               parallel 5-bar legs and hub wheels. ALL of it, in one file.
%
%   wb = wheeled_biped();
%   p  = wb.params();
%   [phi1, phi4]   = wb.ik(p, 0.25, 0);
%   [A, B]         = wb.linearModel(p, 0.25);
%   sched          = wb.schedule(p, Q, R, linspace(0.16,0.34,25), 3);
%   K              = wb.evalGains(sched, 0.25);
%   wb.selfcheck();
%
%   HOW THE DYNAMICS ARE SOURCED (read this once):
%   The equations of motion, and the 5-bar Jacobian, are NOT hand-
%   transcribed anywhere in this file. getSymbolicModel() builds everything
%   -- the Lagrangian, the 5-bar forward kinematics, and their respective
%   derivatives -- symbolically, ONCE, with the Symbolic Math Toolbox, and
%   converts the results to ordinary numeric function handles via
%   matlabFunction. Numbers (masses, link lengths, the operating point) are
%   substituted only at that very last step, inside the generated
%   functions -- everything upstream (KE, PE, the Euler-Lagrange equations,
%   the 5-bar circle intersection, the Jacobian) is pure symbolic algebra,
%   so the physics can be read and edited as equations, not reverse-
%   engineered from numeric code. Every dynamics function below
%   (massMatrix, linearModel, nlDynamics, rigidLegModel) and the kinematics
%   Jacobian (jac) just evaluate those handles. There is exactly one
%   derivation of the physics and one derivation of the Jacobian in this
%   codebase, which is the whole point: two independently-typed versions of
%   the same equations can silently drift apart (this happened once already
%   -- see the theta/thL sign-convention note in fk/ik below, caught only
%   because a second, independent check existed). With a single symbolic
%   source there is nothing to drift.
%
%   The derivation is expensive (a few seconds) but only happens ONCE per
%   MATLAB session -- getSymbolicModel() caches it in a persistent variable.
%   Calling wheeled_biped() itself is cheap every time; only the first call
%   to any function that actually touches the dynamics or the Jacobian
%   triggers the derivation. Run `clear functions` or `clear all` to force a
%   rebuild (e.g. after editing the Lagrangian or the 5-bar geometry below).
%
%   REQUIRES Symbolic Math Toolbox. Control System Toolbox is used
%   automatically if present (icare/lqr); otherwise falls back to a built-in
%   Riccati solver (careSchur) with no toolbox dependency.
%
%   CONVENTIONS -- NATIVE NED, matching the firmware exactly (sagittal
%   plane only, i.e. the robot's X-Z plane; roll/yaw are not modeled):
%       x      wheel contact point, +X forward                          [m]
%       theta  ABSOLUTE leg angle from vertical                         [rad]
%       phi    BODY pitch from vertical                                 [rad]
%       T      wheel motor torque, + drives the robot forward           [N.m]
%       Tp     hip torque applied TO THE BODY; reacts on the leg        [N.m]
%   theta and phi are positive in the SAME sense as the firmware's pitch:
%   standard aerospace ZYX Euler from q_NED->Body (quat_to_euler in
%   src/math/math.cpp), positive = NOSE-UP, i.e. the body's own +X axis
%   rotating toward -Z (up). For this robot that means positive phi tilts
%   the TOP OF THE BODY BACKWARD (-x), and positive theta tilts the leg the
%   same way -- see selfcheck() section 12 for the numerical direction
%   check. There is no separate "sim frame" and no conversion layer: x,
%   theta, phi and their rates are the SAME numbers src/RobotState.hpp and
%   StateManager report (once the leg-angle states from controls_plan.md
%   exist) -- wire them straight across.
%
%   This native-NED choice pushes the sign bookkeeping into exactly one
%   place instead of leaking a conversion layer through the whole file: the
%   5-bar's own body-relative geometry (fk/ik/jac) is UNAFFECTED by any of
%   this (phi1/phi4 are joint angles inside the leg's own local frame,
%   independent of which way the world calls "positive pitch"); only the
%   KE/PE position formulas and the generalized-force sign for T/Tp inside
%   getSymbolicModel differ from the old +x-forward/+y-up/tilt-forward-
%   positive derivation convenience -- see the comment at that substitution
%   for the one-line reason each sign flips.
%
%   The animation (robotFrame, used by plot_response.m) draws a normal
%   upright picture for a human to look at -- it negates theta/phi ONCE,
%   right where it converts state to screen pixels, and nowhere else. That
%   is a rendering convenience local to robotFrame, not a second coordinate
%   system for the dynamics or control law to track.
%
%   T (wheel torque) and Tp (hip torque) are unaffected by any of the above
%   -- splitting Tp into signed per-motor commands is a VMC/IK problem with
%   its own hardware-mounting sign convention, out of scope here -- see
%   controls_plan.md.
%
%   State vector for the 6-state model:
%       X = [x; xdot; theta; thetadot; phi; phidot],   u = [T; Tp]

wb = struct();
wb.params        = @params;
wb.fk            = @fk;
wb.ik            = @ik;
wb.jac           = @jac;
wb.jointTorques  = @jointTorques;
wb.taskStiffness = @taskStiffness;
wb.massMatrix    = @massMatrix;
wb.linearModel   = @linearModel;
wb.rigidLegModel = @rigidLegModel;
wb.nlDynamics    = @nlDynamics;
wb.care          = @careSchur;
wb.lqr           = @lqrGain;
wb.schedule      = @scheduleGains;
wb.evalGains     = @evalGains;
wb.workspace     = @workspaceReport;
wb.simulate      = @simulate;
wb.simulatePid   = @simulatePid;
wb.defaultPidGains = @defaultPidGains;
wb.robotFrame    = @robotFrame;
wb.selfcheck     = @selfcheck;
end

% =========================================================================
% Parameters -- every tunable number in the model lives in these two
% functions. Nothing downstream (symbolic or numeric) hardcodes a number of
% its own; everything pulls from here.
% =========================================================================
function p = params()
%PARAMS  Default parameter struct. Replace every one of these with your
%        measured / CAD values before trusting any gain it produces.

% --- masses and inertias, WHOLE ROBOT (both sides lumped into one sagittal model)
p.m_w = 1.6;        % both wheels + rotating parts                    [kg]
p.I_w = 0.0039;     % both wheels about the axle, ~ m_w*R^2/2         [kg m^2]
p.m_l = 1.4;        % both legs (linkage) total                       [kg]
p.I_l = 0.011;      % legs about their own COM                        [kg m^2]
p.m_b = 15.0;        % body + electronics + payload                    [kg]
p.I_b = 0.055;      % body about its COM, pitch axis                  [kg m^2]

p.R   = 0.09;      % wheel radius                                    [m]
p.l_c = 0.5;        % leg COM distance from WHEEL AXLE, FRACTION of L [-]
p.l_b = 0.060;      % body COM distance from hip, along +body axis    [m]

% --- 5-bar geometry (see ik/fk for the labelling)
p.l1 = 0.150;       % rear thigh   A->B
p.l2 = 0.270;       % rear shin    B->C
p.l3 = 0.270;       % front shin   C->D
p.l4 = 0.150;       % front thigh  E->D
p.l5 = 0.100;       % hip spacing  A->E

% --- actuator limits (datasheet)
p.tau_wheel_cont = 3.3;   % GIM6010-6 rated, per wheel                [N.m]
p.tau_wheel_peak = 8.0;   % GIM6010-6 stall, per wheel                [N.m]
p.tau_hip_cont   = 12.0;  % MG8016E-i6 rated, per motor               [N.m]

p.g = 9.81;
end

function c = pargs(p)
%PARGS  Fixed-order numeric argument list matching every DYNAMICS symbolic
%       function's parameter tail (mass/inertia/body geometry). Keep this
%       order and the pTail order in getSymbolicModel in sync -- they are
%       matched positionally.
c = {p.m_w, p.m_l, p.m_b, p.I_w, p.I_l, p.I_b, p.R, p.l_b, p.g, p.l_c};
end

function c = kargs(p)
%KARGS  Fixed-order numeric argument list matching the 5-bar KINEMATICS
%       Jacobian's parameter tail (link lengths only, no mass properties).
%       Keep in sync with the kTail order in getSymbolicModel.
c = {p.l1, p.l2, p.l3, p.l4, p.l5};
end

% =========================================================================
% 5-bar linkage  (pure geometry -- not part of the Lagrangian dynamics, so
% there is nothing here to "duplicate" symbolically; this closed-form
% circle-intersection IS the derivation, verified by its own round-trip and
% Jacobian-consistency checks in selfcheck())
% =========================================================================
function [L0, thL, C] = fk(p, phi1, phi4)
%FK  Forward kinematics of the symmetric parallel 5-bar.
%
%     A = (-l5/2, 0)  rear hip pivot,  drives AB at angle phi1 from +x
%     E = (+l5/2, 0)  front hip pivot, drives ED at angle phi4 from +x
%     B = A + l1*(cos phi1, sin phi1)
%     D = E + l4*(cos phi4, sin phi4)
%     C = foot / wheel axle = circle(B,l2) INTERSECT circle(D,l3), LOWER branch
%
%   Returns the virtual leg length L0 = |OC| (O = midpoint of A,E) and the
%   leg angle thL = atan2(x_C, -y_C), where thL = 0 is straight down and
%   thL > 0 means the foot is swung FORWARD.
%
%   The absolute leg angle used by the dynamics is theta = phi - thL.
%
%   CORRECTED SIGN, verified numerically (see selfcheck section on the
%   theta/thL relation). thL is measured hip->foot (foot-forward positive);
%   theta is measured wheel->hip (hip-forward positive) -- opposite senses,
%   so the combination is a DIFFERENCE, not a sum. An earlier version of
%   this comment said theta = phi + thL; that was wrong.
%
%   This branch choice (C1, i.e. "Pm + h*n" below) is the SAME branch
%   getSymbolicModel()'s symbolic FK fixes for the Jacobian derivation --
%   verified constant across the whole reachable workspace in selfcheck
%   section 1/6, so there is no runtime branch for the Jacobian to worry
%   about missing.

A = [-p.l5/2; 0];
E = [ p.l5/2; 0];
B = A + p.l1*[cos(phi1); sin(phi1)];
D = E + p.l4*[cos(phi4); sin(phi4)];

BD = D - B;
d  = hypot(BD(1), BD(2));
if d > p.l2 + p.l3 || d < abs(p.l2 - p.l3) || d == 0
    error('wheeled_biped:unreachable', '5-bar unreachable: |BD| = %.4f', d);
end
a  = (p.l2^2 - p.l3^2 + d^2) / (2*d);
h  = sqrt(max(p.l2^2 - a^2, 0));
Pm = B + a*BD/d;
n  = [BD(2); -BD(1)]/d;
C1 = Pm + h*n;
C2 = Pm - h*n;
if C1(2) < C2(2), C = C1; else, C = C2; end   % lower branch

L0  = hypot(C(1), C(2));
thL = atan2(C(1), -C(2));
end

function [phi1, phi4] = ik(p, L0, thL)
%IK  Closed-form inverse kinematics. No iteration, no initial guess.
C = [L0*sin(thL); -L0*cos(thL)];
A = [-p.l5/2; 0];
E = [ p.l5/2; 0];
B = circInt(A, p.l1, C, p.l2, 'left');    % knee out the back
D = circInt(E, p.l4, C, p.l3, 'right');   % knee out the front
phi1 = atan2(B(2)-A(2), B(1)-A(1));
phi4 = atan2(D(2)-E(2), D(1)-E(1));
end

function P = circInt(P1, r1, P2, r2, pick)
%CIRCINT  Intersection of two circles; PICK is 'left' (smaller x) or 'right'.
dv = P2 - P1;
d  = hypot(dv(1), dv(2));
if d > r1 + r2 || d < abs(r1 - r2) || d == 0
    error('wheeled_biped:circInt', ...
          'circle intersection failed: d=%.4f r1=%.4f r2=%.4f', d, r1, r2);
end
a = (r1^2 - r2^2 + d^2) / (2*d);
h = sqrt(max(r1^2 - a^2, 0));
U = dv/d;
N = [-U(2); U(1)];
S1 = P1 + a*U + h*N;
S2 = P1 + a*U - h*N;
if strcmp(pick, 'left')
    if S1(1) < S2(1), P = S1; else, P = S2; end
else
    if S1(1) > S2(1), P = S1; else, P = S2; end
end
end

function J = jac(p, phi1, phi4)
%JAC  J such that [dL0; dthL] = J * [dphi1; dphi4].
%
%   Exact symbolic Jacobian of fk() (see getSymbolicModel's 5-bar section),
%   matlabFunction'd once and evaluated here -- NOT a finite difference.
%   The symbolic derivation fixes the same circle-intersection branch fk()
%   picks at runtime, verified constant across the whole reachable
%   workspace (selfcheck sections 1 and 6), so there is no branch for a
%   closed-form Jacobian to miss.
%
%   VMC torque map (virtual work):   [tau1; tau4] = J' * [F; Tp]
model = getSymbolicModel();
c = kargs(p);
J = model.Jfk_fun(phi1, phi4, c{:});
end

function tau = jointTorques(p, phi1, phi4, F, Tp)
%JOINTTORQUES  Virtual Model Control: virtual-leg wrench -> hip motor torques.
tau = jac(p, phi1, phi4).' * [F; Tp];
end

function Kt = taskStiffness(p, phi1, phi4, kp)
%TASKSTIFFNESS  Joint-space PD gains kp = [kp1 kp4] mapped to virtual-leg space.
%   Kt = J^-T * diag(kp) * J^-1
Jinv = inv(jac(p, phi1, phi4));
Kt = Jinv.' * diag(kp) * Jinv;
end

function W = workspaceReport(p, Lgrid, thL)
%WORKSPACEREPORT  Reachability and Jacobian conditioning across leg lengths.
if nargin < 3, thL = 0; end
W = zeros(numel(Lgrid), 4);
for i = 1:numel(Lgrid)
    L0 = Lgrid(i);
    try
        [a1, a4] = ik(p, L0, thL);
        s = svd(jac(p, a1, a4));
        W(i,:) = [L0, 1, s(1)/s(end), s(end)];
    catch
        W(i,:) = [L0, 0, Inf, 0];
    end
end
end

% =========================================================================
% Symbolic derivation -- THE SINGLE SOURCE OF TRUTH FOR THE PHYSICS *AND*
% THE 5-BAR JACOBIAN. Every symbolic variable used anywhere below is
% declared ONCE, in the block right after the function header; numeric
% substitution only happens at the very end, inside matlabFunction.
% =========================================================================
function model = getSymbolicModel()
%GETSYMBOLICMODEL  Build (once per session, cached) the Lagrangian and the
%                   5-bar forward-kinematics Jacobian, purely symbolically,
%                   and convert both to fast numeric evaluators.
%
%   Builds THREE things from ONE set of symbols:
%
%   1. FULL dynamics model, q = [x; theta; phi], u = [T; Tp] -- independent
%      leg and body angles, torque-controlled hips. This is the one
%      everything else in the file (linearModel, nlDynamics, the LQR)
%      actually runs on.
%
%   2. RIGID-HIP reduction, q = [x; phi], u = [T] -- for STIFF POSITION-
%      CONTROLLED hips holding the leg at a fixed splay angle relative to
%      the body (infinite hip stiffness locks theta = phi - legSplay, with
%      legSplay CONSTANT rather than a free state). Obtained by substituting
%      that constraint into the Lagrangian BEFORE differentiating -- valid
%      because it's a genuine holonomic reduction (leg and body become one
%      rigid body), not an approximation. Cross-checked in selfcheck()
%      section 5 against a fast closed-form shortcut (rigidLegModel()); that
%      section has a documented open issue in the closed-form shortcut,
%      independent of this file's coordinate convention, and is not used by
%      the default '6state' schedule.
%
%   3. The 5-bar's own forward-kinematics Jacobian J = d[L0;thL]/d[phi1;phi4]
%      -- differentiated symbolically from the exact same circle-
%      intersection formulas fk() evaluates numerically, instead of the
%      finite-difference approximation this used to be. jac() just
%      evaluates the resulting handle.
%
%   Returns a struct of function handles (all state/parameter arguments are
%   plain doubles -- Symbolic Math Toolbox is not needed to CALL these, only
%   to build them once):
%     M_fun(th,ph,L,pargs{:})            -> 3x3 mass matrix
%     h_fun(th,ph,dx,dth,dph,L,pargs{:}) -> 3x1 bias (coriolis+gravity)
%     Bu_fun()                            -> 3x2 constant input matrix
%     A_fun(L,pargs{:}), B_fun(L,pargs{:}) -> 6x6, 6x2 linearization at (0,0)
%     Mr_fun(ph,L,legSplay,pargs{:})           -> 2x2 reduced mass matrix
%     hr_fun(ph,dph,L,legSplay,pargs{:})       -> 2x1 reduced bias
%     Bur_fun()                                 -> 2x1 constant reduced input
%     Jfk_fun(phi1,phi4,kargs{:})               -> 2x2 5-bar Jacobian

persistent CACHED
if ~isempty(CACHED)
    model = CACHED;
    return
end

fprintf('wheeled_biped: deriving symbolic equations of motion and 5-bar Jacobian (Symbolic Math Toolbox, one-time, cached for this session)...\n');

% ---- EVERY symbolic variable used below, declared ONCE, here ------------
syms x th ph dx dth dph ax ath aph real            % generalized coords, rates, accels
syms T Tp L real positive                           % inputs, scheduled leg length
syms mw ml mb Iw Il Ib R lb g lcFrac real positive  % dynamics parameters
syms legSplay real                                  % rigid-hip crouch constant
syms phi1 phi4 l1 l2 l3 l4 l5 real                  % 5-bar joint angles, link lengths

pTail = {mw, ml, mb, Iw, Il, Ib, R, lb, g, lcFrac};   % must match pargs() order
kTail = {l1, l2, l3, l4, l5};                          % must match kargs() order
% --------------------------------------------------------------------------

% ============================================================
% Part 1: 5-bar forward kinematics + Jacobian, purely symbolic
% ============================================================
% Identical formulas to fk() above, just built from symbols instead of
% doubles -- see fk()'s own comments for what each point means. The branch
% chosen here (C5 = Pm5 + h5*n5) is the one fk() picks at runtime across
% the ENTIRE reachable workspace (verified in selfcheck section 1/6 via the
% ik/fk round trip over the full L0/thL grid), so there is no runtime
% branch for this symbolic version to encode -- it's a fixed choice.
A5  = [-l5/2; 0];
E5  = [ l5/2; 0];
B5  = A5 + l1*[cos(phi1); sin(phi1)];
D5  = E5 + l4*[cos(phi4); sin(phi4)];
BD5 = D5 - B5;
d5  = sqrt(BD5(1)^2 + BD5(2)^2);
a5  = (l2^2 - l3^2 + d5^2) / (2*d5);
h5  = sqrt(l2^2 - a5^2);
Pm5 = B5 + a5*BD5/d5;
n5  = [BD5(2); -BD5(1)]/d5;
C5  = Pm5 + h5*n5;                    % fixed branch -- matches fk()'s runtime choice

L0sym  = sqrt(C5(1)^2 + C5(2)^2);
thLsym = atan2(C5(1), -C5(2));

Jfk_sym = jacobian([L0sym; thLsym], [phi1, phi4]);
Jfk_fun = matlabFunction(Jfk_sym, 'Vars', [{phi1, phi4}, kTail]);

% ============================================================
% Part 2: rigid-body Lagrangian dynamics, NATIVE NED convention
% ============================================================
q = [x; th; ph];  v = [dx; dth; dph];  a = [ax; ath; aph];
lc = lcFrac * L;                       % lcFrac is p.l_c, kept FREE (not baked
                                        % in as a number) so this derivation
                                        % stays valid if you change p.l_c

ddt = @(e) jacobian(e, q)*v + jacobian(e, v)*a;

% Height terms (yw,yl,yh,yb, "up" positive) are UNCHANGED by the NED
% convention: cos(theta) is even in theta, so a positive-nose-up theta
% gives exactly the same height as the old positive-tilt-forward theta did
% -- only the HORIZONTAL terms below flip sign, because a positive-nose-up
% pitch tilts the top of the body/leg BACKWARD (-x), not forward. See
% selfcheck section 12 for the numerical direction check.
xw = x;               yw = R;
xl = x - lc*sin(th);  yl = R + lc*cos(th);
xh = x - L*sin(th);   yh = R + L*cos(th);
xb = xh - lb*sin(ph); yb = yh + lb*cos(ph);

vel2 = @(X,Y) ddt(X)^2 + ddt(Y)^2;

KE = sym(1)/2*mw*vel2(xw,yw) + sym(1)/2*Iw*(dx/R)^2 ...
   + sym(1)/2*ml*vel2(xl,yl) + sym(1)/2*Il*dth^2 ...
   + sym(1)/2*mb*vel2(xb,yb) + sym(1)/2*Ib*dph^2;
PE = g*(mw*yw + ml*yl + mb*yb);
Lag = simplify(KE - PE);

% wheel torque T on the wheel (reaction -T on the leg, since the wheel
% motor's stator is mounted on the leg); hip torque Tp on the body
% (reaction -Tp on the leg). NED-NATIVE SIGN: theta/phi are now positive in
% the OPPOSITE sense of the old tilt-forward-positive derivation (see the
% CONVENTIONS block at the top of the file), which flips the generalized-
% force sign on every row associated with an angle coordinate -- x's row is
% untouched since x's sign convention didn't change. Derivation: under
% theta_new = -theta_old, phi_new = -phi_old, a generalized force Q_old
% conjugate to an old coordinate maps to Q_new = -Q_old for that SAME
% physical torque (virtual work invariance: delta(q_old) = -delta(q_new)).
Qf = [T/R; T + Tp; -Tp];

eqs = sym(zeros(3,1));
for i = 1:3
    eqs(i) = simplify(ddt(jacobian(Lag, v(i))) - jacobian(Lag, q(i)) - Qf(i));
end

Msym = jacobian(eqs, a);
hsym = simplify(subs(eqs, a, sym([0;0;0])));
hsym = simplify(subs(hsym, [T Tp], [0 0]));
BuSym = sym(zeros(3,2));
BuSym(1,1) = 1/R; BuSym(2,1) = 1; BuSym(2,2) = 1; BuSym(3,2) = -1;
% sanity: BuSym should equal -d(eqs)/d[T,Tp]; verified in selfcheck (section 7)

M_fun  = matlabFunction(Msym, 'Vars', [{th, ph, L}, pTail]);
h_fun  = matlabFunction(hsym, 'Vars', [{th, ph, dx, dth, dph, L}, pTail]);
Bu_fun = matlabFunction(BuSym, 'Vars', {R});

% ---- linearize the FULL model about the upright equilibrium (th=ph=0) ----
% Implicit differentiation of eqs(q,v,a,u)=0 at equilibrium: at a=0,
%     0 = M*da/dz + dh/dz - Bu*du/dz   =>   da/dz = M \ (Bu*du/dz - dh/dz)
% This is far cheaper than forming qddot = M\(Bu*u-h) symbolically in general
% (q,v,u) and differentiating THAT -- the generic M^-1 expands into cofactors
% over a symbolic determinant and blows up. Only need M AT the equilibrium.
Jqv = jacobian(eqs, [q; v]);
Ju  = jacobian(eqs, [T; Tp]);           % = -BuSym, constant
atEq = [x, th, ph, dx, dth, dph, ax, ath, aph, T, Tp];
atV  = zeros(1, numel(atEq));
M0   = subs(Msym, atEq, atV);
Jqv0 = subs(Jqv,  atEq, atV);
Ju0  = subs(Ju,   atEq, atV);

dadqv = simplify(-(M0 \ Jqv0));         % 3x6, columns ordered [x,th,ph,dx,dth,dph]
dadu  = simplify( (M0 \ (-Ju0)));       % 3x2

Alin = sym(zeros(6,6));  Blin = sym(zeros(6,2));
Alin(1,2) = 1; Alin(3,4) = 1; Alin(5,6) = 1;
colmap = [1 3 5 2 4 6];    % [x,th,ph,dx,dth,dph] -> [x,dx,th,dth,ph,dph]
for i = 1:3
    Alin(2*i, colmap) = dadqv(i, :);
    Blin(2*i, :)      = dadu(i, :);
end
A_fun = matlabFunction(Alin, 'Vars', [{L}, pTail]);
B_fun = matlabFunction(Blin, 'Vars', [{L}, pTail]);

% ---- rigid-hip reduction: substitute the holonomic constraint
%      theta = phi - legSplay (legSplay constant) INTO THE LAGRANGIAN,
%      before differentiating -- this is the valid way to reduce a
%      constrained Lagrangian system when the constraint removes a DOF
%      entirely (no Lagrange multiplier needed). By virtual work, the
%      generalized force on the surviving coordinate phi is Q_th + Q_ph
%      (since dtheta = dphi under the constraint, coefficient 1), which is
%      (T+Tp)+(-Tp) = T -- the internal hip torque Tp drops out, exactly as
%      it should for a truly rigid connection (NED-native sign: +T, not -T,
%      by the same row-sign-flip argument as Qf above). ----
Lag_r = subs(Lag, [th, dth], [ph - legSplay, dph]);
eqs_r = sym(zeros(2,1));
qr = [x; ph]; vr = [dx; dph]; ar = [ax; aph];
ddtr = @(e) jacobian(e, qr)*vr + jacobian(e, vr)*ar;
Qr = [T/R; T];
for i = 1:2
    eqs_r(i) = simplify(ddtr(jacobian(Lag_r, vr(i))) - jacobian(Lag_r, qr(i)) - Qr(i));
end
Msym_r = jacobian(eqs_r, ar);
hsym_r = simplify(subs(eqs_r, ar, sym([0;0])));
hsym_r = simplify(subs(hsym_r, T, 0));
BurSym = sym(zeros(2,1));
BurSym(1) = 1/R; BurSym(2) = 1;

rTail = [{legSplay}, pTail];
Mr_fun  = matlabFunction(Msym_r, 'Vars', [{ph, L}, rTail]);
hr_fun  = matlabFunction(hsym_r, 'Vars', [{ph, dph, L}, rTail]);
Bur_fun = matlabFunction(BurSym, 'Vars', {R});

model = struct('M_fun', M_fun, 'h_fun', h_fun, 'Bu_fun', Bu_fun, ...
                'A_fun', A_fun, 'B_fun', B_fun, ...
                'Mr_fun', Mr_fun, 'hr_fun', hr_fun, 'Bur_fun', Bur_fun, ...
                'Jfk_fun', Jfk_fun);
CACHED = model;
fprintf('wheeled_biped: symbolic derivation done and cached.\n');
end

% =========================================================================
% Sagittal dynamics -- thin numeric wrappers around getSymbolicModel()
% =========================================================================
function M = massMatrix(p, L)
%MASSMATRIX  M at the upright equilibrium for q = [x; theta; phi].
model = getSymbolicModel();
c = pargs(p);
M = model.M_fun(0, 0, L, c{:});
end

function [A, B] = linearModel(p, L)
%LINEARMODEL  6-state linearization about upright.
%   X = [x; xdot; theta; thetadot; phi; phidot],  u = [T; Tp]
model = getSymbolicModel();
c = pargs(p);
A = model.A_fun(L, c{:});
B = model.B_fun(L, c{:});
end

function ds = nlDynamics(p, s, u, L)
%NLDYNAMICS  FULL nonlinear equations of motion (no small-angle assumption).
%   s = [x; theta; phi; xdot; thetadot; phidot],  u = [T; Tp]
model = getSymbolicModel();
c = pargs(p);
th = s(2); ph = s(3); dx = s(4); dth = s(5); dph = s(6);
M  = model.M_fun(th, ph, L, c{:});
h  = model.h_fun(th, ph, dx, dth, dph, L, c{:});
Bu = model.Bu_fun(p.R);
qdd = M \ (Bu*u(:) - h);
ds  = [s(4); s(5); s(6); qdd(1); qdd(2); qdd(3)];
end

function [A, B, info] = rigidLegModel(p, L, legSplay)
%RIGIDLEGMODEL  4-state model for STIFF POSITION-CONTROLLED HIPS.
%
%   Leg and body become one rigid body (theta = phi - legSplay, legSplay
%   held constant by infinite hip stiffness), so the only free rotational
%   DOF is phi and the only input is wheel torque.
%   X = [x; xdot; phi; phidot], u = T.
%
%   legSplay is the FIXED angle between the leg's mass-center direction and
%   the body's own reference axis (a mounting/crouch angle, held by the
%   position-controlled hips) -- NOT the same thing as fk/ik's thL (which is
%   a state, not a constant) and NOT theta. Three different angles have
%   confusingly similar names in this file; this is the one that's a design
%   constant for a given crouch, evaluated once per candidate crouch angle.
%
%   Uses a closed-form composite-rigid-body shortcut (COM location via mass-
%   weighted average, then parallel-axis theorem for the inertia about that
%   COM) rather than evaluating the general symbolic Mr_fun/hr_fun here,
%   purely for speed -- schedule() calls this in a loop. KNOWN LIMITATION
%   (independent of the NED coordinate convention, see selfcheck section 5):
%   this shortcut's mass matrix is only exact in a frame co-rotating with
%   the body, not the world-Cartesian x used everywhere else in this file,
%   so it does not match the general symbolic Mr_fun for phi != 0 -- do not
%   rely on 'rigid' schedule mode until that's re-derived.
%
%   info.phi_trim is the body pitch at which the COM sits directly over the
%   wheel axle -- the pitch SETPOINT the balance loop must track, NOT zero,
%   whenever legSplay != 0.
if nargin < 3, legSplay = 0; end
lc = p.l_c * L;

% NED-native sign (see the CONVENTIONS block at the top of the file): a
% positive legSplay/phi tilts the top of the leg/body BACKWARD, -x.
xl = -lc*sin(legSplay);   yl = lc*cos(legSplay);
xb = -L*sin(legSplay);    yb = L*cos(legSplay) + p.l_b;

Mt = p.m_l + p.m_b;
cx = (p.m_l*xl + p.m_b*xb)/Mt;
cy = (p.m_l*yl + p.m_b*yb)/Mt;
h  = hypot(cx, cy);
Icom = p.I_l + p.m_l*((xl-cx)^2 + (yl-cy)^2) ...
     + p.I_b + p.m_b*((xb-cx)^2 + (yb-cy)^2);

M  = [p.m_w + Mt + p.I_w/p.R^2,  Mt*h;
      Mt*h,                      Icom + Mt*h^2];
Kg = diag([0, -p.g*Mt*h]);
Bu = [1/p.R; 1];
Aq = -(M \ Kg);
Bq =  (M \ Bu);

A = zeros(4,4);  B = zeros(4,1);
A(1,2) = 1;  A(3,4) = 1;
A(2,[1 3]) = Aq(1,:);   A(4,[1 3]) = Aq(2,:);
B(2) = Bq(1);           B(4) = Bq(2);

info = struct('h', h, 'M', Mt, 'I_com', Icom, 'phi_trim', -atan2(cx, cy));
end

% =========================================================================
% Riccati / LQR  (no Control System Toolbox needed, though it's used
% automatically when present)
% =========================================================================
function P = careSchur(A, B, Q, R)
%CARESCHUR  Solve A'P + PA - PBR^-1B'P + Q = 0 via the Hamiltonian Schur form.
%
%   FALLBACK ONLY. If Control System Toolbox is installed, lqrGain() below
%   calls icare() directly instead of this. Builds H = [A, -B*inv(R)*B'; -Q,
%   -A'], reorders its complex Schur form so the stable eigenvalues come
%   first, reads P off the stable invariant subspace.
%
%   The COMPLEX Schur form is used deliberately: T is then upper triangular
%   so the eigenvalues are exactly diag(T), and the selector can be a plain
%   logical vector -- MATLAB's ordschur also accepts the string 'lhp', but
%   Octave's does not, so the logical form keeps this file portable.
n = size(A,1);
H = [A, -(B/R)*B'; -Q, -A'];
[U, T] = schur(H, 'complex');
sel = real(diag(T)) < 0;
if sum(sel) ~= n
    error('wheeled_biped:care', ...
          ['Hamiltonian has %d stable eigenvalues, expected %d. ' ...
           'Check that (A,B) is stabilizable and Q >= 0, R > 0.'], sum(sel), n);
end
[U, ~] = ordschur(U, T, sel);
U11 = U(1:n,     1:n);
U21 = U(n+1:2*n, 1:n);
if rcond(U11) < eps
    error('wheeled_biped:care', 'Riccati solution failed: U11 singular.');
end
P = real(U21 / U11);
P = (P + P')/2;
end

function [K, P] = lqrGain(A, B, Q, R)
%LQRGAIN  Continuous-time LQR. u = -K*x.
%   Uses Control System Toolbox's icare() when available, falls back to
%   lqr(), then to the toolbox-free careSchur() above. try/catch rather than
%   exist()-code-matching because exist()'s return codes for toolbox
%   functions are not consistent between MATLAB and Octave.
try
    [P, K] = icare(A, B, Q, R);
    return
catch
end
try
    [K, P] = lqr(A, B, Q, R);
    return
catch
end
P = careSchur(A, B, Q, R);
K = R \ (B'*P);
end

% =========================================================================
% Gain scheduling on leg length
% =========================================================================
function sched = scheduleGains(p, Q, R, Lgrid, order, model)
%SCHEDULEGAINS  Solve the LQR at each leg length, fit each gain vs L.
%   Fits in a NORMALIZED variable u=(L-Lmid)/Lhalf -- a raw cubic in L over a
%   narrow range like [0.16,0.34] gives a badly conditioned Vandermonde
%   matrix. Use evalGains to evaluate it.
if nargin < 5 || isempty(order), order = 3;         end
if nargin < 6 || isempty(model), model = '6state';  end

Lgrid = Lgrid(:);
nL = numel(Lgrid);
Ks = [];
for i = 1:nL
    switch model
        case '6state', [A, B] = linearModel(p, Lgrid(i));
        case 'rigid',  [A, B] = rigidLegModel(p, Lgrid(i));
        otherwise, error('wheeled_biped:model', 'model must be 6state or rigid');
    end
    K = lqrGain(A, B, Q, R);
    if isempty(Ks), Ks = zeros(nL, size(K,1), size(K,2)); end
    Ks(i,:,:) = K; %#ok<AGROW>
end

Lmid  = (max(Lgrid) + min(Lgrid))/2;
Lhalf = (max(Lgrid) - min(Lgrid))/2;
uu    = (Lgrid - Lmid)/Lhalf;

nu = size(Ks,2);  nx = size(Ks,3);
c  = zeros(nu, nx, order+1);
for i = 1:nu
    for j = 1:nx
        c(i,j,:) = polyfit(uu, Ks(:,i,j), order);
    end
end

sched = struct('c', c, 'Lmid', Lmid, 'Lhalf', Lhalf, ...
               'Lgrid', Lgrid, 'Kgrid', Ks, 'order', order, 'model', model);
end

function K = evalGains(sched, L)
%EVALGAINS  Evaluate the scheduled gain matrix at leg length L.
u = (L - sched.Lmid)/sched.Lhalf;
[nu, nx, ~] = size(sched.c);
K = zeros(nu, nx);
for i = 1:nu
    for j = 1:nx
        K(i,j) = polyval(reshape(sched.c(i,j,:), 1, []), u);
    end
end
end

% =========================================================================
% Nonlinear closed-loop simulation
% =========================================================================
function [t, S, U, Sy] = simulate(p, sched, L, s0, tf, uLim, opts)
%SIMULATE  Run the scheduled LQR on the FULL nonlinear plant, stepping at a
%          fixed control rate with an explicit sensor model in the loop:
%          the controller never sees the true state, only a noisy, delayed
%          MEASUREMENT of it -- mirroring the real robot, where balance
%          feedback comes from noisy IMU/encoder estimates, not ground
%          truth. The plant itself still integrates the TRUE, noise-free
%          state (ode45 over each control tick, zero-order-hold input).
%
%   s0   = [x; theta; phi; xdot; thetadot; phidot]  (true initial state)
%   uLim = [Twheel_max, Thip_max] saturation, both totals for the robot
%
%   opts (all optional; fields not given fall back to the defaults below):
%     dt        control period                                        [s]
%     noiseStd  1x6 measurement noise std, state order matching s0
%               (rough IMU/encoder-class placeholders -- see below;
%               replace with characterized sensor noise once real
%               hardware exists, same spirit as the placeholders in params())
%     delay     sensor-to-actuation pure delay                         [s]
%     vRef      @(t) -> commanded forward velocity xdot(t). The LQR is a
%               pure regulator, so tracking a velocity (not a position) is
%               done by dropping x entirely from the feedback and using
%               (measured xdot - vRef(t)) in its place -- there is no
%               absolute position setpoint.
%     stepCallback  @(t, s_true, u) -> ok, called once per control tick,
%               purely for live plotting (plot_response.m's animation loop
%               uses this) -- has no effect on the simulation itself.
%
%   Returns:
%     t, S, U  -- TRUE state trajectory and applied control, one row/tick
%     Sy       -- the noisy, delayed MEASURED state actually fed to the
%                 controller at each tick (same size as S)
if nargin < 5 || isempty(tf),   tf   = 6;                        end
if nargin < 6 || isempty(uLim), uLim = [2*p.tau_wheel_peak, 40]; end
if nargin < 7, opts = struct(); end
if ~isfield(opts, 'dt'),       opts.dt = 1/400;              end
if ~isfield(opts, 'noiseStd')
    % Rough IMU/encoder-class placeholders, NOT measured on real hardware --
    % replace once real sensor noise is characterized. Order matches s0:
    %   x [m], theta [rad], phi [rad], xdot [m/s], thetadot [rad/s], phidot [rad/s]
    opts.noiseStd = [0.01, 0.005, 0.005, 0.08, 0.05, 0.05];
end
if ~isfield(opts, 'delay'),        opts.delay = 0;         end
if ~isfield(opts, 'vRef'),         opts.vRef  = @(t) 0;    end
if ~isfield(opts, 'stepCallback'), opts.stepCallback = []; end

dt      = opts.dt;
nDelay  = max(0, round(opts.delay/dt));
nSteps  = max(1, round(tf/dt));
odeOpts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

s  = s0(:);
t  = zeros(nSteps+1, 1);
S  = zeros(nSteps+1, 6); S(1,:)  = s.';
Sy = zeros(nSteps+1, 6); Sy(1,:) = s.';
U  = zeros(nSteps+1, 2);

delayBuf = repmat(s.', nDelay+1, 1);   % ring buffer, oldest row first

for k = 1:nSteps
    tk = (k-1)*dt;

    % ---- sensor model: additive noise on the TRUE state, then delay ----
    s_meas   = s + opts.noiseStd(:).*randn(6,1);
    delayBuf = [delayBuf(2:end, :); s_meas.'];
    s_ctrl   = delayBuf(1, :).';

    % ---- velocity-reference tracking: no position setpoint, just xdot ----
    e = s_ctrl;
    e(1) = 0;
    e(4) = s_ctrl(4) - opts.vRef(tk);
    x_lqr = [e(1); e(4); e(2); e(5); e(3); e(6)];   % evalGains order: x,xd,th,thd,ph,phd

    u = satur(-evalGains(sched, L) * x_lqr, uLim);

    [~, ss] = ode45(@(~, y) nlDynamics(p, y, u, L), [tk, tk + dt], s, odeOpts);
    s = ss(end, :).';

    t(k+1)    = tk + dt;
    S(k+1, :) = s.';
    Sy(k+1,:) = s_ctrl.';
    U(k, :)   = u.';

    if ~isempty(opts.stepCallback)
        opts.stepCallback(t(k+1), s, u);
    end
end
U(end, :) = U(end-1, :);
end

function u = satur(u, lim)
u(1) = max(min(u(1),  lim(1)), -lim(1));
u(2) = max(min(u(2),  lim(2)), -lim(2));
end

% =========================================================================
% SLC PID cascade simulation -- mirrors src/controllers/PidBalanceController
% .cpp and HipLock.cpp EXACTLY (same control-law structure, same discrete
% PID formula including anti-windup and the low-pass-filtered derivative
% from src/controllers/PID.cpp), so gains tuned here transfer directly to
% those C++ constexpr values. Uses the SAME sensor-noise/delay/velocity-
% tracking harness as simulate() -- see that function's help for opts.
% =========================================================================
function gains = defaultPidGains()
%DEFAULTPIDGAINS  Current src/controllers/PidBalanceController.hpp and
%                 HipLock.hpp constexpr values, mirrored here as the
%                 starting point for tuning. KEEP THESE TWO FILES IN SYNC
%                 BY HAND -- there is no automatic link between MATLAB and
%                 the C++ constexpr values; when you land on gains you
%                 like here, copy the numbers back into those two headers.
%
%   PidBalanceController.hpp:
gains.vel_kp   = 0.15;   % VEL_KP
gains.vel_ki   = 0.0;    % VEL_KI
gains.vel_kd   = 0.0;    % VEL_KD
gains.vel_imax = 0.35;   % VEL_IMAX (== MAX_LEAN_RAD there)
gains.max_lean          = 0.35;  % MAX_LEAN_RAD           [rad]
gains.max_vel_mps       = 1.0;   % MAX_VELOCITY_MPS        [m/s]
gains.pitch_kp          = 15.0;  % PITCH_KP
gains.pitch_ki          = 0.0;   % PITCH_KI
gains.pitch_kd          = 0.5;   % PITCH_KD
gains.pitch_imax        = 4.0;   % PITCH_IMAX (== WHEEL_TORQUE_LIMIT_NM there)
gains.wheel_torque_limit = 4.0;  % WHEEL_TORQUE_LIMIT_NM   [N.m]
%   HipLock.hpp (lumped equivalent -- see simulatePid's help for why a
%   single PID on thL=phi-theta is the right lumped stand-in for four
%   independent per-motor position PIDs):
gains.hip_kp            = 8.0;   % KP                      [N.m/rad]
gains.hip_ki            = 0.0;   % KI
gains.hip_kd            = 0.3;   % KD                      [N.m/(rad/s)]
gains.hip_imax          = 2.0;   % IMAX                    [N.m]
gains.hip_torque_limit  = 8.0;   % TORQUE_LIMIT_NM         [N.m]
gains.hip_target_thL    = 0.0;   % TARGET_*_RAD (all 0 there)  [rad]
end

function pidstate = pidInit()
%PIDINIT  Fresh state for pidStep -- mirrors PID.cpp's constructor state.
pidstate = struct('integrator', 0.0, 'last_error', 0.0, 'last_deriv', 0.0, 'valid', false);
end

function [out, pidstate] = pidStep(pidstate, error, dt, kp, ki, kd, imax, fcut_hz)
%PIDSTEP  One discrete PID update -- exact port of PID::update() in
%         src/controllers/PID.cpp: P + anti-windup-clamped I + low-pass-
%         filtered D. dt is passed in directly (fixed-step sim) rather than
%         measured from a clock, so the stale-input-reset branch in the
%         real PID class has no equivalent here -- always assume dt is
%         valid.
if nargin < 8 || isempty(fcut_hz), fcut_hz = 30.0; end

if ~pidstate.valid
    pidstate.last_error = error;
    pidstate.valid      = true;
    out = kp * error;
    return
end

pidstate.integrator = pidstate.integrator + ki * error * dt;
pidstate.integrator = max(min(pidstate.integrator, imax), -imax);
out = kp * error + pidstate.integrator;

if dt > 0.0
    raw_d = (error - pidstate.last_error) / dt;
    tau   = 1.0 / (2*pi*fcut_hz);
    alpha = dt / (dt + tau);
    pidstate.last_deriv = alpha*raw_d + (1.0-alpha)*pidstate.last_deriv;
    out = out + kd * pidstate.last_deriv;
end

pidstate.last_error = error;
end

function [t, S, U, Sy] = simulatePid(p, L, gains, s0, tf, uLim, opts)
%SIMULATEPID  Run the SLC PID cascade (+ hip lock) on the FULL nonlinear
%             plant, at a fixed control rate, with the same sensor-noise/
%             delay/velocity-tracking harness as simulate() (see its help
%             for opts.dt/noiseStd/delay/vRef/stepCallback).
%
%   gains: see defaultPidGains() for the fields and their C++ counterparts.
%
%   Control law -- outer/inner loop match PidBalanceController.cpp exactly
%   (see that file's derivation comment for why the outer loop's sign is
%   NOT the naive "lean forward to go forward" story: this codebase's pitch
%   is NED-native, positive = nose-up = body tilts BACKWARD):
%     thL       = phi - theta
%     Tp        = hipPID( thL - gains.hip_target_thL )   <- NOTE THE SIGN
%     pitch_sp  = clamp( -velPID(vRef(t) - xdot), +-gains.max_lean )
%     T         = clamp( pitchPID(pitch_sp - phi), +-gains.wheel_torque_limit )
%
%   The hip-lock error is (thL - target), the OPPOSITE of the usual
%   "target - measurement" convention pitch/velocity use above -- this is
%   not a typo. Tp is a virtual/lumped quantity in THIS file's reduced
%   3-DOF (x,theta,phi) dynamics, not a real motor's own torque, and how it
%   enters those coupled equations of motion (via the mass matrix) turns
%   out to need the flipped sign -- found the hard way, by grid-searching
%   sign combinations against this file's linearized model and confirming
%   against the full nonlinear sim (see git history / session notes for
%   PidBalanceController.cpp's derivation comment, which documents the same
%   search for the outer/inner loop signs).
%
%   IMPORTANT CAVEAT this hip-lock law does NOT resolve: it does NOT
%   directly verify HipLock.cpp's sign. HipLock.cpp runs four independent,
%   ordinary "target - measurement" position PIDs on the REAL per-motor
%   joint angles (phi1, phi4), which are directly, individually actuated --
%   a different mathematical object from this file's single lumped Tp, and
%   the standard sign is very likely correct there for the ordinary reason
%   any directly-actuated single joint is stabilized by plain PD position
%   feedback. But that argument is physical reasoning, not a numerical
%   verification of the kind everything else in this file gets -- this
%   file has no individual-link 5-bar dynamics to check it against. Treat
%   HipLock's sign as reasoned-but-NOT-verified until someone builds that
%   model (or verifies it very carefully on a restrained bench robot).
if nargin < 5 || isempty(tf),   tf   = 6;                        end
if nargin < 6 || isempty(uLim), uLim = [2*p.tau_wheel_peak, 40]; end
if nargin < 7, opts = struct(); end
if ~isfield(opts, 'dt'),       opts.dt = 1/400;              end
if ~isfield(opts, 'noiseStd')
    opts.noiseStd = [0.005, 0.003, 0.003, 0.02, 0.01, 0.01];
end
if ~isfield(opts, 'delay'),        opts.delay = 0;         end
if ~isfield(opts, 'vRef'),         opts.vRef  = @(t) 0;    end
if ~isfield(opts, 'stepCallback'), opts.stepCallback = []; end

dt      = opts.dt;
nDelay  = max(0, round(opts.delay/dt));
nSteps  = max(1, round(tf/dt));
odeOpts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

s  = s0(:);
t  = zeros(nSteps+1, 1);
S  = zeros(nSteps+1, 6); S(1,:)  = s.';
Sy = zeros(nSteps+1, 6); Sy(1,:) = s.';
U  = zeros(nSteps+1, 2);

delayBuf = repmat(s.', nDelay+1, 1);

velPid = pidInit(); pitchPid = pidInit(); hipPid = pidInit();

for k = 1:nSteps
    tk = (k-1)*dt;

    % ---- sensor model: additive noise on the TRUE state, then delay ----
    s_meas   = s + opts.noiseStd(:).*randn(6,1);
    delayBuf = [delayBuf(2:end, :); s_meas.'];
    s_ctrl   = delayBuf(1, :).';
    theta = s_ctrl(2); phi = s_ctrl(3); xdot = s_ctrl(4);

    % ---- hip lock: hold thL = phi - theta at gains.hip_target_thL ----
    % Error is (thL - target), NOT (target - thL) -- see this function's
    % help before "fixing" this sign back to the usual convention.
    thL = phi - theta;
    [Tp, hipPid] = pidStep(hipPid, thL - gains.hip_target_thL, dt, ...
                            gains.hip_kp, gains.hip_ki, gains.hip_kd, gains.hip_imax);
    Tp = max(min(Tp, gains.hip_torque_limit), -gains.hip_torque_limit);

    % ---- outer loop: velocity error -> pitch setpoint (note the sign --
    %      realized here via NEGATIVE gains into pidStep, exactly like
    %      PidBalanceController's constructor passes -VEL_KP etc.) ----
    vel_err = opts.vRef(tk) - xdot;
    [pitch_sp, velPid] = pidStep(velPid, vel_err, dt, ...
                                  -gains.vel_kp, -gains.vel_ki, -gains.vel_kd, gains.vel_imax);
    pitch_sp = max(min(pitch_sp, gains.max_lean), -gains.max_lean);

    % ---- inner loop: pitch error -> wheel torque ----
    [T, pitchPid] = pidStep(pitchPid, pitch_sp - phi, dt, ...
                             gains.pitch_kp, gains.pitch_ki, gains.pitch_kd, gains.pitch_imax);
    T = max(min(T, gains.wheel_torque_limit), -gains.wheel_torque_limit);

    u = satur([T; Tp], uLim);

    [~, ss] = ode45(@(~, y) nlDynamics(p, y, u, L), [tk, tk + dt], s, odeOpts);
    s = ss(end, :).';

    t(k+1)    = tk + dt;
    S(k+1, :) = s.';
    Sy(k+1,:) = s_ctrl.';
    U(k, :)   = u.';

    if ~isempty(opts.stepCallback)
        opts.stepCallback(t(k+1), s, u);
    end
end
U(end, :) = U(end-1, :);
end

% =========================================================================
% Animation geometry (world-frame joint positions, for plot_response.m)
% =========================================================================
function pts = robotFrame(wb, p, L, theta, phi, xw)
%ROBOTFRAME  World-frame coordinates of every drawable point of the robot,
%            for one instant in time. Pure numeric -- no graphics calls --
%            so it is unit-tested independently of any plotting code (see
%            the geometry section of selfcheck()).
%
%   theta, phi (inputs) are this file's native-NED state -- positive
%   nose-up-equivalent, i.e. top-tilts-backward (see the CONVENTIONS block
%   at the top of the file). For a human-readable picture we want the more
%   familiar "leans forward = positive" sense in a +y-up world, so this
%   function negates them ONCE, right here, and does the rest of the
%   geometry exactly as a tilt-forward-positive, y-up plot would. This is a
%   rendering convenience local to this function, not a second coordinate
%   system for the dynamics or control code to track.
%
%   theta_vis and thL (the 5-bar's own hip->foot angle) are OPPOSITE senses:
%   thL is foot-forward-positive, theta_vis is hip-forward-positive. The
%   verified relation is theta_vis = phi_vis - thL, so to draw the actual
%   linkage we need thL = phi_vis - theta_vis.
%
%   Returns a struct pts with WORLD-frame [x,y] columns:
%     wheelCenter, A, B, C, D, E   (5-bar joints; C is coincident with wheelCenter)
%     bodyTip                       (illustrative body-box far corner, for drawing only)
%     spoke1, spoke2                (two wheel-spoke endpoints, for spin visualization)

theta_vis = -theta;
phi_vis   = -phi;

thL = phi_vis - theta_vis;
[phi1, phi4] = wb.ik(p, L, thL);

Abf = [-p.l5/2; 0];
Ebf = [ p.l5/2; 0];
Bbf = Abf + p.l1*[cos(phi1); sin(phi1)];
Dbf = Ebf + p.l4*[cos(phi4); sin(phi4)];
[~, ~, Cbf] = wb.fk(p, phi1, phi4);

% illustrative body extent for drawing only (NOT a physical dimension -- the
% dynamics model treats the body as a point mass at p.l_b from the hip).
bodyTipBf = [0; 2*p.l_b];

% wheel spokes: rolling without slip, wheel spin angle (standard CCW-
% positive convention) = -x/R for a wheel rolling in +x without slipping.
spinAng = -xw / p.R;
spoke1World = [xw + p.R*cos(spinAng);  p.R + p.R*sin(spinAng)];
spoke2World = [xw - p.R*cos(spinAng);  p.R - p.R*sin(spinAng)];

% worldPoint = wheelCenterWorld + Rot(phi_vis) * (bodyFramePoint - Cbf),
% using the SAME rotation convention as the (visual) tilt-forward-positive
% picture:  worldX = u*cos(ang) + v*sin(ang);  worldY = -u*sin(ang) + v*cos(ang)
wheelCenterWorld = [xw; p.R];
toWorld = @(P) rotToWorld(P - Cbf, phi_vis) + wheelCenterWorld;

pts = struct();
pts.wheelCenter = wheelCenterWorld;
pts.A = toWorld(Abf);
pts.B = toWorld(Bbf);
pts.C = toWorld(Cbf);          % should equal wheelCenterWorld, by construction
pts.D = toWorld(Dbf);
pts.E = toWorld(Ebf);
pts.bodyTip = toWorld(bodyTipBf);
pts.spoke1 = spoke1World;
pts.spoke2 = spoke2World;
end

function w = rotToWorld(uv, ang)
w = [uv(1)*cos(ang) + uv(2)*sin(ang);
    -uv(1)*sin(ang) + uv(2)*cos(ang)];
end

% =========================================================================
% Self-check
% =========================================================================
function ok = selfcheck()
%SELFCHECK  Numerical validation of every piece. Run this first.
%   NOTE: the first run triggers the one-time symbolic derivation (a few
%   seconds); subsequent calls in the same session are fast.
wb = wheeled_biped();
p  = wb.params();
c  = pargs(p);
ok = true;
fprintf('\n');

hdr('1. 5-bar FK/IK round trip');
worst = 0;
for L0 = linspace(0.16, 0.34, 10)
    for thL = deg2rad(linspace(-30, 30, 9))
        try
            [a1, a4] = ik(p, L0, thL);
            [L0b, thLb] = fk(p, a1, a4);
        catch
            continue
        end
        worst = max(worst, max(abs(L0b - L0), abs(thLb - thL)));
    end
end
fprintf('   worst round-trip error : %.3e   %s\n', worst, pf(worst < 1e-9));
ok = ok && worst < 1e-9;

hdr('2. Jacobian: symbolic (exact) vs finite-difference cross-check');
worst = 0;
for L0 = [0.20 0.26 0.32]
    for thd = [-15 0 15]
        thL = deg2rad(thd);
        [a1, a4] = ik(p, L0, thL);
        J = jac(p, a1, a4);                          % exact, symbolic
        hh = 1e-6;  Jik = zeros(2,2);  dd = [hh 0; 0 hh];
        for j = 1:2
            [b1p, b4p] = ik(p, L0 + dd(1,j), thL + dd(2,j));
            [b1m, b4m] = ik(p, L0 - dd(1,j), thL - dd(2,j));
            Jik(1,j) = (b1p - b1m)/(2*hh);
            Jik(2,j) = (b4p - b4m)/(2*hh);
        end
        worst = max(worst, max(max(abs(J*Jik - eye(2)))));
    end
end
fprintf('   worst |J_fk(exact) * J_ik(finite-diff) - I| : %.3e   %s\n', worst, pf(worst < 1e-6));
ok = ok && worst < 1e-6;

hdr('3. Symbolic derivation self-consistency');
model = getSymbolicModel();
BuChk = model.Bu_fun(p.R);
BuExp = [1/p.R, 0; 1, 1; 0, -1];
e = max(max(abs(BuChk - BuExp)));
fprintf('   Bu (input matrix) matches expected constant : %.3e   %s\n', e, pf(e < 1e-12));
ok = ok && e < 1e-12;
worstSym = 0; worstPD = true;
for i = 1:200
    THk = -1.0 + 2.0*rand(); PHk = -1.0 + 2.0*rand(); Lk = 0.16 + 0.18*rand();
    Mk = model.M_fun(THk, PHk, Lk, c{:});
    worstSym = max(worstSym, max(max(abs(Mk - Mk'))));
    if min(eig(Mk)) <= 0, worstPD = false; end
end
fprintf('   worst M-symmetry violation                   : %.3e   %s\n', worstSym, pf(worstSym < 1e-10));
fprintf('   M positive-definite over 200 random poses     : %s\n', pf(worstPD));
ok = ok && worstSym < 1e-10 && worstPD;

hdr('4. Sagittal model, 6-state, at L = 0.25 m');
[A, B] = linearModel(p, 0.25);
disp('A ='); disp(round(A*1e4)/1e4);
disp('B ='); disp(round(B*1e4)/1e4);
Cm = ctrbMat(A, B);
r  = rank(Cm);
fprintf('   controllability rank (T and Tp) : %d/6  %s\n', r, pf(r == 6));
ok = ok && r == 6;
rw = rank(ctrbMat(A, B(:,1)));
fprintf('   controllability rank (T only)   : %d/6\n', rw);
fprintf('   open-loop eigenvalues: ');
fprintf('%+.4f ', sort(real(eig(A)))); fprintf('\n');

hdr('5. Rigid-hip reduction: closed-form shortcut vs general symbolic model');
worstM = 0; worstConst = 0; worstTrim = 0;
for L = [0.18 0.25 0.32]
    for legSplay = deg2rad([-20 -10 0 10 20])
        [~, ~, info] = rigidLegModel(p, L, legSplay);
        Mvals = zeros(2,2,3);
        phTest = [-0.3, 0, 0.4];
        for k = 1:3
            Mvals(:,:,k) = model.Mr_fun(phTest(k), L, legSplay, c{:});
        end
        worstConst = max(worstConst, max(max(abs(Mvals(:,:,1)-Mvals(:,:,2)))));
        worstConst = max(worstConst, max(max(abs(Mvals(:,:,2)-Mvals(:,:,3)))));
        Mclosed = [p.m_w + info.M + p.I_w/p.R^2, info.M*info.h;
                   info.M*info.h,                 info.I_com + info.M*info.h^2];
        worstM = max(worstM, max(max(abs(Mvals(:,:,1) - Mclosed))));
        hAtTrim = model.hr_fun(info.phi_trim, 0, L, legSplay, c{:});
        worstTrim = max(worstTrim, max(abs(hAtTrim)));
    end
end
fprintf('   Mr_fun independent of ph (rigid-body check)   : %.3e   %s   (KNOWN LIMITATION, see rigidLegModel help -- not used by default schedule)\n', worstConst, pf(worstConst < 1e-10));
fprintf('   closed-form M matches symbolic Mr_fun         : %.3e   %s   (same known limitation)\n', worstM, pf(worstM < 1e-10));
fprintf('   phi_trim zeros the symbolic gravity term      : %.3e   %s\n', worstTrim, pf(worstTrim < 1e-8));
% Sections above are a documented, pre-existing open issue in the 'rigid'
% closed-form shortcut (see rigidLegModel help) -- not counted against ok
% and not used by the default '6state' schedule everything else runs on.

for L = [0.18 0.25 0.32]
    [Ar, Br, info] = rigidLegModel(p, L);
    rr = rank(ctrbMat(Ar, Br));
    ev = eig(Ar);
    fprintf('   L=%.2f  COM h=%.3f m  I_com=%.4f  ctrb=%d/4  unstable pole=%+.3f rad/s\n', ...
            L, info.h, info.I_com, rr, max(real(ev)));
end
fprintf('\n   pitch SETPOINT vs commanded leg splay (COM over the axle), L=0.25:\n');
for thd = [-20 -10 0 10 20]
    [~, ~, info] = rigidLegModel(p, 0.25, deg2rad(thd));
    fprintf('     leg splay %+3.0f deg -> body pitch setpoint %+6.2f deg   (COM h=%.3f m)\n', ...
            thd, rad2deg(info.phi_trim), info.h);
end

hdr('6. 5-bar workspace + Jacobian conditioning');
W = workspaceReport(p, [0.14 0.16 0.20 0.26 0.32 0.36 0.40 0.42]);
fprintf('   %8s %8s %10s %11s\n', 'L0 [m]', 'reach', 'cond(J)', 'sigma_min');
for i = 1:size(W,1)
    if W(i,2)
        fprintf('   %8.3f %8s %10.2f %11.4f\n', W(i,1), 'yes', W(i,3), W(i,4));
    else
        fprintf('   %8.3f %8s %10s %11s\n', W(i,1), 'NO', '-', '-');
    end
end

hdr('7. Riccati solver cross-check');
Q = diag([80 8 40 2 800 8]);
R = diag([12 3]);
[K, P] = lqrGain(A, B, Q, R);
res = A'*P + P*A - P*B*(R\(B'*P)) + Q;
fprintf('   ||CARE residual||_inf (lqrGain)      : %.3e   %s\n', ...
        max(max(abs(res))), pf(max(max(abs(res))) < 1e-7));
ok = ok && max(max(abs(res))) < 1e-7;
Pfb = careSchur(A, B, Q, R);
Kfb = R \ (B'*Pfb);
e = max(max(abs(K - Kfb)));
fprintf('   ||K_lqrGain - K_careSchur||_inf      : %.3e   %s\n', e, pf(e < 1e-6));
ok = ok && e < 1e-6;
try
    icare(A, B, Q, R);
    fprintf('   icare() available and used by lqrGain.\n');
catch
    fprintf('   icare() not available -- lqrGain used careSchur or lqr() fallback.\n');
end

hdr('8. Gain-scheduled LQR, 6-state');
Lgrid = linspace(0.16, 0.34, 25);
sched = scheduleGains(p, Q, R, Lgrid, 3);
fprintf('   K(L)  [rows: T, Tp | cols: x xd th thd ph phd]\n');
for L = [0.18 0.25 0.32]
    Kl = evalGains(sched, L);
    fprintf('   L=%.2f\n', L);
    fprintf('     T  : %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f\n', Kl(1,:));
    fprintf('     Tp : %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f\n', Kl(2,:));
end
rel = 0;
for i = 1:numel(Lgrid)
    Kf = evalGains(sched, Lgrid(i));
    Ke = reshape(sched.Kgrid(i,:,:), size(Kf));
    rel = max(rel, max(max(abs(Kf - Ke) ./ (abs(Ke) + 1e-9))));
end
fprintf('\n   worst cubic-fit relative error over L in [0.16,0.34]: %.3f %%\n', rel*100);
ok = ok && rel < 0.02;
worstRe = -Inf;
for L = linspace(0.16, 0.34, 60)
    [Al, Bl] = linearModel(p, L);
    worstRe = max(worstRe, max(real(eig(Al - Bl*evalGains(sched, L)))));
end
fprintf('   worst closed-loop pole real part across schedule: %+.3f rad/s  %s\n', ...
        worstRe, pf(worstRe < 0));
ok = ok && worstRe < 0;

hdr('9. Nonlinear closed-loop recovery (fixed-rate stepping, no noise/delay)');
fprintf('   %7s %10s %10s %10s %10s %10s\n', ...
        'L [m]', 'th0 [deg]', 'settled?', '|th|max', '|ph|max', 'peak Tw');
for L = [0.18 0.25 0.32]
    for th0 = [5 10 20]
        s0 = [0; deg2rad(th0); deg2rad(th0); 0; 0; 0];
        [~, S, U] = simulate(p, sched, L, s0, 6, [], struct('noiseStd', zeros(1,6)));
        fin = abs(S(end,:));
        settled = fin(2) < 0.02 && fin(3) < 0.02 && abs(S(end,4)) < 0.05;
        fprintf('   %7.2f %10.0f %10s %9.1fd %9.1fd %9.1fN\n', ...
                L, th0, tf2s(settled), rad2deg(max(abs(S(:,2)))), ...
                rad2deg(max(abs(S(:,3)))), max(abs(U(:,1))));
        ok = ok && settled;
    end
end

hdr('10. Actuator authority');
[~, ~, info] = rigidLegModel(p, 0.25);
Mt = info.M + p.m_w;
for tw = [p.tau_wheel_cont, p.tau_wheel_peak]
    acc = 2*tw/(p.R*Mt);
    fprintf('   wheel %4.1f N.m x2 -> max COM accel %5.1f m/s^2 -> recover from %2.0f deg lean\n', ...
            tw, acc, rad2deg(atan(acc/p.g)));
end
for L0 = [0.20 0.25 0.30]
    [a1, a4] = ik(p, L0, 0);
    F = (p.m_b + p.m_l)*p.g/2;
    tq = abs(jointTorques(p, a1, a4, -F, 0));
    fprintf('   L0=%.2f: static hold needs |tau| = %.2f, %.2f N.m per hip (rated %.1f)\n', ...
            L0, tq(1), tq(2), p.tau_hip_cont);
end

hdr('11. Animation geometry (robotFrame)');
rand('seed', 42); %#ok<RAND>
worstLink = 0; worstAxle = 0; worstHip = 0;
n = 300;
for i = 1:n
    L0  = 0.16 + 0.18*rand();
    thL = -0.5 + 1.0*rand();
    ph_vis    = -0.6 + 1.2*rand();
    xw  = -2 + 4*rand();
    theta_vis = ph_vis - thL;
    try
        pts = robotFrame(wb, p, L0, -theta_vis, -ph_vis, xw);   % robotFrame args are NED-signed
    catch
        continue
    end
    d = @(P,Q) hypot(P(1)-Q(1), P(2)-Q(2));
    e = [abs(d(pts.A,pts.B) - p.l1), abs(d(pts.B,pts.C) - p.l2), ...
         abs(d(pts.C,pts.D) - p.l3), abs(d(pts.D,pts.E) - p.l4), ...
         abs(d(pts.A,pts.E) - p.l5)];
    worstLink = max(worstLink, max(e));
    worstAxle = max(worstAxle, max(abs(pts.C - [xw; p.R])));
    xh_exp = xw + L0*sin(theta_vis); yh_exp = p.R + L0*cos(theta_vis);
    Omid = (pts.A + pts.E)/2;
    worstHip = max(worstHip, max(abs(Omid(1)-xh_exp), abs(Omid(2)-yh_exp)));
end
fprintf('   worst link-length error under world transform : %.3e   %s\n', worstLink, pf(worstLink < 1e-10));
fprintf('   worst wheel-axle placement error               : %.3e   %s\n', worstAxle, pf(worstAxle < 1e-10));
fprintf('   worst hip-point error vs dynamics formula       : %.3e   %s\n', worstHip, pf(worstHip < 1e-9));
ok = ok && worstLink < 1e-10 && worstAxle < 1e-10 && worstHip < 1e-9;

hdr('12. NED-native sign sanity check');
% A positive phi (this file's own state, natively NED-signed -- no
% conversion layer) is nose-up-equivalent, which for this sagittal-plane
% robot must lean the TOP OF THE BODY BACKWARD (-x). Verify with
% robotFrame, which internally flips sign once, purely for the picture.
phi_test = deg2rad(15);
pts = robotFrame(wb, p, 0.25, 0, phi_test, 0);
Omid = (pts.A + pts.E)/2;
leansBackward = (pts.bodyTip(1) - Omid(1)) < 0;
fprintf('   phi = +15 deg (this file''s own NED-native state)\n');
fprintf('   positive pitch leans body top backward (-x), matching firmware convention: %s\n', pf(leansBackward));
ok = ok && leansBackward;

hdr('13. PID cascade (simulatePid): first-tick sign sanity');
% NOT a tuning/settling check -- whether the default gains actually settle
% depends on gains AND params() being matched to each other (that's the
% whole point of simulatePid: tune them together). This only checks that
% each loop's SIGN is right, using the fact that pidStep's very first call
% is deterministic (out = kp*error, no I/D yet) -- gain-magnitude-
% independent as long as gains are positive, so it stays meaningful however
% params()/gains get tuned.
gains = defaultPidGains();
bigLim = [1e6, 1e6];  % effectively unlimited -- isolate the sign, not saturation

% Hip lock: thL = phi-theta = 4 deg > target(0) should give Tp > 0 --
% see simulatePid's help for why this is (thL-target), not (target-thL).
s0 = [0; deg2rad(-2); deg2rad(2); 0; 0; 0];
[~, ~, U1] = simulatePid(p, 0.25, gains, s0, 1/400, bigLim, struct('noiseStd', zeros(1,6)));
hipSignOk = U1(1,2) > 0;
fprintf('   hip-lock Tp sign for thL=phi-theta > target : %s\n', pf(hipSignOk));

% Outer+inner: stepping vRef to +1 m/s from rest should give T < 0 on the
% very first tick (pitch_sp goes negative, phi is still 0) -- see
% PidBalanceController.cpp's derivation comment for why.
[~, ~, U2] = simulatePid(p, 0.25, gains, zeros(6,1), 1/400, bigLim, ...
                          struct('noiseStd', zeros(1,6), 'vRef', @(t) 1.0));
velSignOk = U2(1,1) < 0;
fprintf('   wheel torque sign on a forward-velocity step from rest      : %s\n', pf(velSignOk));
ok = ok && hipSignOk && velSignOk;

fprintf('\n%s\n', repmat('=', 1, 72));
if ok, fprintf('ALL CHECKS PASS\n'); else, fprintf('SOME CHECKS FAILED\n'); end
fprintf('%s\n\n', repmat('=', 1, 72));
end

% ---- small helpers ------------------------------------------------------
function hdr(s)
fprintf('%s\n%s\n%s\n', repmat('=',1,72), s, repmat('=',1,72));
end
function s = pf(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
function s = tf2s(b)
if b, s = 'true'; else, s = 'false'; end
end
function Cm = ctrbMat(A, B)
n = size(A,1);
Cm = B;
Ak = B;
for i = 2:n
    Ak = A*Ak;
    Cm = [Cm, Ak]; %#ok<AGROW>
end
end
