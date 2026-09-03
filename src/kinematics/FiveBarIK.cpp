#include "src/kinematics/FiveBarIK.hpp"
#include <cmath>

namespace {

struct Vec2 { float x, y; };

inline Vec2 operator+(Vec2 a, Vec2 b) { return { a.x + b.x, a.y + b.y }; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return { a.x - b.x, a.y - b.y }; }
inline Vec2 operator*(float s, Vec2 v) { return { s * v.x, s * v.y }; }
inline float norm2(Vec2 v) { return std::hypot(v.x, v.y); }

// Intersection of two circles (P1,r1) and (P2,r2); pick_left selects the
// smaller-x solution, matching wheeled_biped.m's circInt('left'/'right').
// Returns false if the circles don't intersect (out of range / degenerate).
bool circ_int(Vec2 P1, float r1, Vec2 P2, float r2, bool pick_left, Vec2& out)
{
    Vec2 dv = P2 - P1;
    float d = norm2(dv);
    if (d > r1 + r2 || d < std::fabs(r1 - r2) || d == 0.0f) return false;

    float a = (r1*r1 - r2*r2 + d*d) / (2.0f*d);
    float h = std::sqrt(std::fmax(r1*r1 - a*a, 0.0f));
    Vec2 U = (1.0f/d) * dv;
    Vec2 N = { -U.y, U.x };
    Vec2 S1 = P1 + a*U + h*N;
    Vec2 S2 = P1 + a*U - h*N;

    if (pick_left) out = (S1.x < S2.x) ? S1 : S2;
    else           out = (S1.x > S2.x) ? S1 : S2;
    return true;
}

}  // namespace

bool fk(const FiveBarParams& p, float phi1, float phi4, FiveBarPose& out)
{
    Vec2 A = { -p.l5/2.0f, 0.0f };
    Vec2 E = {  p.l5/2.0f, 0.0f };
    Vec2 B = A - Vec2{ p.l1*std::cos(phi1), -p.l1*std::sin(phi1) };   // phi1 zero points -x, CW-positive -- see header comment
    Vec2 D = E + Vec2{ p.l4*std::cos(phi4),  p.l4*std::sin(phi4) };   // phi4 zero points +x, CCW-positive -- see header comment

    Vec2  BD = D - B;
    float d  = norm2(BD);
    if (d > p.l2 + p.l3 || d < std::fabs(p.l2 - p.l3) || d == 0.0f) return false;

    float a  = (p.l2*p.l2 - p.l3*p.l3 + d*d) / (2.0f*d);
    float h  = std::sqrt(std::fmax(p.l2*p.l2 - a*a, 0.0f));
    Vec2  Pm = B + (a/d) * BD;
    Vec2  n  = (1.0f/d) * Vec2{ BD.y, -BD.x };
    Vec2  C1 = Pm + h*n;
    Vec2  C2 = Pm - h*n;
    // Lower branch, in the solver's own plain-Cartesian (y-up) frame --
    // "lower" means smaller y, matching wheeled_biped.m's fk() exactly.
    Vec2  C  = (C1.y < C2.y) ? C1 : C2;

    out.L0  = norm2(C);
    out.thL = std::atan2(C.x, -C.y);
    return true;
}

bool ik(const FiveBarParams& p, float L0, float thL, float& phi1, float& phi4)
{
    Vec2 C = { L0*std::sin(thL), -L0*std::cos(thL) };
    Vec2 A = { -p.l5/2.0f, 0.0f };
    Vec2 E = {  p.l5/2.0f, 0.0f };

    Vec2 B, D;
    if (!circ_int(A, p.l1, C, p.l2, /*pick_left=*/true,  B)) return false;   // knee out the back
    if (!circ_int(E, p.l4, C, p.l3, /*pick_left=*/false, D)) return false;  // knee out the front

    phi1 = std::atan2(B.y - A.y, A.x - B.x);   // zero points -x, CW-positive -- see header comment
    phi4 = std::atan2(D.y - E.y, D.x - E.x);   // zero points +x, CCW-positive
    return true;
}

FiveBarJac jac(const FiveBarParams& p, float phi1, float phi4)
{
    FiveBarJac J = {};
    constexpr float EPS = 1e-4f;   // rad

    FiveBarPose p1_hi, p1_lo, p4_hi, p4_lo;
    bool ok = fk(p, phi1 + EPS, phi4, p1_hi) && fk(p, phi1 - EPS, phi4, p1_lo)
            && fk(p, phi1, phi4 + EPS, p4_hi) && fk(p, phi1, phi4 - EPS, p4_lo);
    if (!ok) return J;   // near a reach limit -- leave zeroed rather than divide on a bad sample

    J.m[0][0] = (p1_hi.L0  - p1_lo.L0)  / (2.0f*EPS);
    J.m[1][0] = (p1_hi.thL - p1_lo.thL) / (2.0f*EPS);
    J.m[0][1] = (p4_hi.L0  - p4_lo.L0)  / (2.0f*EPS);
    J.m[1][1] = (p4_hi.thL - p4_lo.thL) / (2.0f*EPS);
    return J;
}

FiveBarVel jac_to_vel(const FiveBarJac& J, float phi1_dot, float phi4_dot)
{
    FiveBarVel v;
    v.L0_dot  = J.m[0][0]*phi1_dot + J.m[0][1]*phi4_dot;
    v.thL_dot = J.m[1][0]*phi1_dot + J.m[1][1]*phi4_dot;
    return v;
}

FootPointNed foot_point_ned(const FiveBarPose& pose, const FiveBarVel& vel)
{
    const float s = std::sin(pose.thL), c = std::cos(pose.thL);
    FootPointNed f;
    f.x     = pose.L0 * s;
    f.z     = pose.L0 * c;                                    // NED Z-down sign flip -- see header note
    f.x_dot = vel.L0_dot*s + pose.L0*vel.thL_dot*c;
    f.z_dot = vel.L0_dot*c - pose.L0*vel.thL_dot*s;
    return f;
}
