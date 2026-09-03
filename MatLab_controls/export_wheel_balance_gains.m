% export_wheel_balance_gains.m — one-shot script: computes the gain-
% scheduled LQR from wheeled_biped.m's VELOCITY-CONTROLLED-WHEEL reduced
% model (linearModelVel, 4-state [theta,thetadot,phi,phidot], u=[ax,Tp] --
% matches CANMotor.cpp's ODrive wheel velocity mode, NOT the old torque-
% wheel 6-state model) and writes it out as a constexpr C++ header,
% src/controllers/WheelBalanceGainTable.hpp, for WheelBalanceLQR to
% consume from both StandUpController and LqrBalanceController.
%
% Run once, offline, whenever params()/Q/R change -- NOT part of the
% firmware build. Supersedes the earlier export_lqr_gains.m /
% LqrGainTable.hpp (torque-wheel model, deleted 2026-09-02 once this
% replaced it — see controls_plan.md section 5).

wb = wheeled_biped();
p  = wb.params();

% Same Q/R already exercised by this session's single-point sanity check
% (10deg initial theta/phi tilt settling to <0.001deg within 4s, closed-
% loop discrete eigenvalue magnitudes all < 1) -- not a fresh, untested
% guess.
Q = diag([40 2 800 8]);
R = diag([1 3]);

% Covers the collapsed stand-up start (~0.098m at phi1=phi4=+16deg) through
% the height stick's max extension (L_STAND=0.152m +/- HEIGHT_RANGE_M=0.05m
% -> 0.20m), comfortably inside the confirmed-reachable/well-conditioned
% [0.08, 0.34] workspace (cond(J) 4.28-12 throughout, see this session's
% workspace check) -- see StandUpController.hpp / LqrBalanceController.hpp
% for the constants this must cover.
Lgrid = linspace(0.09, 0.22, 25);
order = 3;
dt    = 1/100;   % ControlThread recomputes at 100 Hz -- see its own comment

sched = wb.schedule(p, Q, R, Lgrid, order, 'velwheel', dt);

c     = sched.c;          % [nu x nx x (order+1)], MATLAB polyval order (highest degree first)
Lmid  = sched.Lmid;
Lhalf = sched.Lhalf;
[nu, nx, nc] = size(c);

assert(nu == 2 && nx == 4, 'unexpected gain matrix shape');
assert(nc == order + 1, 'unexpected coefficient count');

% Stability re-check across the grid before ever writing anything out --
% mirrors wb.selfcheck()'s own closed-loop pole check, done here against
% the DISCRETE design specifically (selfcheck's own section 8 only checks
% the continuous '6state' design).
worstMag = -Inf;
for L = linspace(min(Lgrid), max(Lgrid), 60)
    [Av, Bv] = wb.linearModelVel(p, L);
    [Ad, Bd] = wb.discretize(Av, Bv, dt);
    K = wb.evalGains(sched, L);
    worstMag = max(worstMag, max(abs(eig(Ad - Bd*K))));
end
fprintf('worst closed-loop discrete eigenvalue magnitude across grid: %.4f (must be < 1)\n', worstMag);
assert(worstMag < 1, 'discrete closed loop unstable somewhere on the grid -- fix Q/R before exporting');

fprintf('Exporting sched: Lmid=%.6f Lhalf=%.6f order=%d nu=%d nx=%d\n', ...
        Lmid, Lhalf, order, nu, nx);

out_path = fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'controllers', 'WheelBalanceGainTable.hpp');
fid = fopen(out_path, 'w');
assert(fid > 0, 'could not open output file');

fprintf(fid, '#pragma once\n\n');
fprintf(fid, '/*\n');
fprintf(fid, ' * WheelBalanceGainTable.hpp -- GENERATED FILE, do not hand-edit.\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Produced by MatLab_controls/export_wheel_balance_gains.m from\n');
fprintf(fid, ' * wheeled_biped.m''s wb.schedule(p, Q, R, Lgrid, order, ''velwheel'', dt) --\n');
fprintf(fid, ' * discrete-time (dt = 1/100 s, matching ControlThread''s 100 Hz compute\n');
fprintf(fid, ' * rate) LQR design against the VELOCITY-CONTROLLED-WHEEL reduced model,\n');
fprintf(fid, ' * gain-scheduled against leg length L, one cubic polynomial fit per gain\n');
fprintf(fid, ' * entry over a normalized u = (L - LMID) / LHALF.\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Rows: [ax (wheel acceleration cmd, m/s^2 -- NOT a torque, see\n');
fprintf(fid, ' * WheelBalanceLQR.hpp), Tp (hip torque, N.m)]. Columns (state order matches\n');
fprintf(fid, ' * wheeled_biped.m''s linearModelVel exactly): [theta, thetadot, phi, phidot].\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Regenerate by running export_wheel_balance_gains.m in MATLAB whenever\n');
fprintf(fid, ' * params()/Q/R change -- see controls_plan.md section 5.\n');
fprintf(fid, ' *\n');
fprintf(fid, ' * Q = diag([%s]),  R = diag([%s]),  Lgrid = linspace(%.3f, %.3f, %d),  order = %d\n', ...
        strjoin(arrayfun(@(x) sprintf('%g', x), diag(Q), 'UniformOutput', false), ' '), ...
        strjoin(arrayfun(@(x) sprintf('%g', x), diag(R), 'UniformOutput', false), ' '), ...
        min(Lgrid), max(Lgrid), numel(Lgrid), order);
fprintf(fid, ' */\n');
fprintf(fid, 'namespace WheelBalanceGainTable {\n');
fprintf(fid, '    constexpr int   NU    = %d;\n', nu);
fprintf(fid, '    constexpr int   NX    = %d;\n', nx);
fprintf(fid, '    constexpr int   ORDER = %d;   // cubic -- ORDER+1 = %d coefficients per entry, highest degree first (MATLAB polyval order)\n', order, order+1);
fprintf(fid, '    constexpr float LMID  = %.9ff;\n', Lmid);
fprintf(fid, '    constexpr float LHALF = %.9ff;\n', Lhalf);
fprintf(fid, '\n');
fprintf(fid, '    // C[row][col][coeff], coeff 0 = highest degree (matches MATLAB polyval convention)\n');
fprintf(fid, '    constexpr float C[NU][NX][ORDER + 1] = {\n');
for i = 1:nu
    fprintf(fid, '        {\n');
    for j = 1:nx
        coeffs = reshape(c(i,j,:), 1, []);
        fprintf(fid, '            { %s },\n', strjoin(arrayfun(@(v) sprintf('%.9ef', v), coeffs, 'UniformOutput', false), ', '));
    end
    fprintf(fid, '        },\n');
end
fprintf(fid, '    };\n');
fprintf(fid, '}\n');
fclose(fid);

fprintf('Wrote %s\n', out_path);

% Print evalGains at a few L for a manual sanity cross-check against the
% values this same script's stdout log should be diffed against.
for L = [0.10 0.152 0.20]
    Kl = wb.evalGains(sched, L);
    fprintf('L=%.3f\n  ax : %s\n  Tp : %s\n', L, ...
        sprintf('%8.3f', Kl(1,:)), sprintf('%8.3f', Kl(2,:)));
end
