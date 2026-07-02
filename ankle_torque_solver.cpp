/*
 * 2-DOF Ankle Joint Torque Solver (C++17)
 * ========================================
 * Exact port of the verified Mech class from the original HTML simulation.
 * Maps desired ankle torques to required motor torques via Jacobian transpose.
 *
 * Compile: g++ -O2 -std=c++17 -o ankle_torque_solver ankle_torque_solver.cpp -lm
 * Run:     ./ankle_torque_solver
 */
#include <cstdio>
#include <cmath>
#include <array>

static constexpr double DEG = M_PI / 180.0;
static constexpr double RAD = 180.0 / M_PI;

// ═══════════════════════════════════════════════════
// 3D MATH
// ═══════════════════════════════════════════════════
struct Vec3 {
    double x, y, z;
    Vec3 operator+(Vec3 b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double dot(Vec3 b) const { return x*b.x + y*b.y + z*b.z; }
    double lsq() const { return dot(*this); }
};

using Mat3 = std::array<std::array<double,3>,3>;

Mat3 Rx(double a) {
    double c = cos(a), s = sin(a);
    return {{{1,0,0},{0,c,-s},{0,s,c}}};
}
Mat3 Ry(double a) {
    double c = cos(a), s = sin(a);
    return {{{c,0,s},{0,1,0},{-s,0,c}}};
}
Mat3 mm(const Mat3& A, const Mat3& B) {
    Mat3 R{};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k=0;k<3;k++)
        R[i][j] += A[i][k] * B[k][j];
    return R;
}
Vec3 mv(const Mat3& M, Vec3 v) {
    return {M[0][0]*v.x+M[0][1]*v.y+M[0][2]*v.z,
            M[1][0]*v.x+M[1][1]*v.y+M[1][2]*v.z,
            M[2][0]*v.x+M[2][1]*v.y+M[2][2]*v.z};
}

// ═══════════════════════════════════════════════════
// MECHANISM MODEL — EXACT PORT FROM ORIGINAL HTML
// ═══════════════════════════════════════════════════
struct Params {
    double m1z, m2z, r1, r2, L1, L2, E, ujZ, ah, oy;
};

struct IKResult { double t1, t2; bool ok; };
struct SolveResult { double th; bool ok; };
struct JacResult { double J[2][2]; double det; double cn; };
struct TorqueResult { double m1, m2, t1, t2; double det, cn; bool ok; };

class Mech {
public:
    double m1z, m2z, r1, r2, L1, L2, E, ujZ, ah, oy;
    Vec3 a1b, a2b, ac;

    Mech(const Params& p) { set(p); }

    void set(const Params& p) {
        m1z=p.m1z; m2z=p.m2z; r1=p.r1; r2=p.r2;
        L1=p.L1; L2=p.L2; E=p.E; ujZ=p.ujZ; ah=p.ah; oy=p.oy;
        a1b = {p.E/2, oy, p.ah};    // RIGHT
        a2b = {-p.E/2, oy, p.ah};   // LEFT
        ac = {0, 0, p.ujZ};        // U-joint center
    }

    Mat3 rot(double p, double r) const { return mm(Rx(p), Ry(r)); }
    Vec3 ankW(Vec3 bp, double p, double r) const { return mv(rot(p,r), bp) + ac; }
    Vec3 ct1(double th) const { return {r1*sin(th), oy, m1z + r1*cos(th)}; }
    Vec3 ct2(double th) const { return {-r2*sin(th), oy, m2z + r2*cos(th)}; }

    SolveResult _solve(double mz, double r, double L, Vec3 aw, double xDir, int branch=1) const {
        double dx = -aw.x, dy = oy - aw.y, dz = mz - aw.z;
        double a = 2*xDir*r*dx;
        double b = 2*r*dz;
        double c = L*L - r*r - dx*dx - dy*dy - dz*dz;
        double R = sqrt(a*a + b*b);
        if (R < 1e-12) return {0, false};
        double ratio = c / R;
        if (fabs(ratio) > 1) return {0, false};
        double psi = atan2(a, b);
        double delta = acos(fmax(-1, fmin(1, ratio)));
        double th = (branch == 0) ? (psi - delta) : (psi + delta);
        return {th, true};
    }

    IKResult ik(double pitch, double roll, int b1=1, int b2=1) const {
        Vec3 a1 = ankW(a1b, pitch, roll);
        Vec3 a2 = ankW(a2b, pitch, roll);
        auto s1 = _solve(m1z, r1, L1, a1, +1, b1);
        auto s2 = _solve(m2z, r2, L2, a2, -1, b2);
        return {s1.th, s2.th, s1.ok && s2.ok};
    }

    JacResult jacobian(double t1, double t2, double p, double r) const {
        double cp = cos(p), sp = sin(p);
        double cr = cos(r), sr = sin(r);

        Vec3 tip1 = ct1(t1), tip2 = ct2(t2);
        Vec3 a1 = ankW(a1b, p, r), a2 = ankW(a2b, p, r);

        Vec3 dc1_dt1 = {r1 * cos(t1), 0.0, -r1 * sin(t1)};
        Vec3 dc2_dt2 = {-r2 * cos(t2), 0.0, -r2 * sin(t2)};

        double den1 = (tip1 - a1).dot(dc1_dt1);
        double den2 = (tip2 - a2).dot(dc2_dt2);

        auto get_da_dp = [&](Vec3 bp) -> Vec3 {
            return {
                0.0,
                (cp*sr)*bp.x + (-sp)*bp.y + (-cp*cr)*bp.z,
                (sp*sr)*bp.x + (cp)*bp.y + (-sp*cr)*bp.z
            };
        };

        auto get_da_dr = [&](Vec3 bp) -> Vec3 {
            return {
                -sr*bp.x + cr*bp.z,
                (sp*cr)*bp.x + (sp*sr)*bp.z,
                (-cp*cr)*bp.x + (-cp*sr)*bp.z
            };
        };

        Vec3 da1_dp = get_da_dp(a1b);
        Vec3 da1_dr = get_da_dr(a1b);
        Vec3 da2_dp = get_da_dp(a2b);
        Vec3 da2_dr = get_da_dr(a2b);

        JacResult res{};
        res.cn = 1e18;

        if (fabs(den1) > 1e-12 && fabs(den2) > 1e-12) {
            res.J[0][0] = (tip1 - a1).dot(da1_dp) / den1;
            res.J[0][1] = (tip1 - a1).dot(da1_dr) / den1;
            res.J[1][0] = (tip2 - a2).dot(da2_dp) / den2;
            res.J[1][1] = (tip2 - a2).dot(da2_dr) / den2;

            res.det = res.J[0][0]*res.J[1][1] - res.J[0][1]*res.J[1][0];
            double nJ = sqrt(res.J[0][0]*res.J[0][0]+res.J[0][1]*res.J[0][1]+
                             res.J[1][0]*res.J[1][0]+res.J[1][1]*res.J[1][1]);
            if (fabs(res.det) > 1e-12) {
                double iJ[2][2] = {
                    {res.J[1][1]/res.det, -res.J[0][1]/res.det},
                    {-res.J[1][0]/res.det, res.J[0][0]/res.det}
                };
                res.cn = nJ * sqrt(iJ[0][0]*iJ[0][0]+iJ[0][1]*iJ[0][1]+
                                   iJ[1][0]*iJ[1][0]+iJ[1][1]*iJ[1][1]);
            }
        }
        return res;
    }

    TorqueResult solveTorques(double pitch, double roll, double tp, double tr) const {
        auto ik_res = ik(pitch, roll);
        if (!ik_res.ok) return {0,0,0,0,0,0,false};
        auto jac = jacobian(ik_res.t1, ik_res.t2, pitch, roll);
        if (fabs(jac.det) < 1e-10) return {0,0,ik_res.t1,ik_res.t2,jac.det,jac.cn,false};

        // τ_motor = (Jᵀ)⁻¹ · τ_ankle
        double m1 = ( jac.J[1][1]*tp - jac.J[1][0]*tr) / jac.det;
        double m2 = (-jac.J[0][1]*tp + jac.J[0][0]*tr) / jac.det;
        return {m1, m2, ik_res.t1, ik_res.t2, jac.det, jac.cn, true};
    }
};

// ═══════════════════════════════════════════════════
// ROBSTRIDE MOTOR DATABASE
// ═══════════════════════════════════════════════════
struct Motor { const char* name; double rated; double peak; };
static Motor MOTORS[] = {
    {"RS00",  5, 14},
    {"RS02",  6, 17},
    {"RS03", 20, 60},
    {"RS04", 40, 120},
};

int main() {
    Params par = {23.5, 16.0, 2.5, 2.5, 20.7, 12.7, 2.3, 3.6, 0.0, 3.8};
    Mech mech(par);

    auto ik0 = mech.ik(0, 0);
    printf("=================================================================\n");
    printf("  Ankle Torque Solver — Verification (C++)\n");
    printf("=================================================================\n");
    printf("  Neutral IK angles: θ₁ = %.1f°, θ₂ = %.1f° %s\n",
           ik0.t1*RAD, ik0.t2*RAD, ik0.ok ? "✓" : "✗");
    printf("\n");

    struct TestCase {
        double pitch, roll, tp, tr;
        const char* desc;
    } tests[] = {
        {0,   0,   5,  5,  "Neutral pose, light load"},
        {15,  10,  5,  5,  "Pitch=15° Roll=10°, light load"},
        {0,   0,   20, 20, "Neutral pose, heavy load"},
        {15,  10,  20, 20, "Pitch=15° Roll=10°, heavy load"},
        {-10, 5,   10, 10, "Pitch=-10° Roll=5°, medium load"},
    };

    for (auto& tc : tests) {
        auto res = mech.solveTorques(tc.pitch*DEG, tc.roll*DEG, tc.tp, tc.tr);
        printf("  [%s]\n", tc.desc);
        printf("    Pose: Pitch=%.0f°, Roll=%.0f°\n", tc.pitch, tc.roll);
        printf("    Load: τ_pitch=%.0f N·m, τ_roll=%.0f N·m\n", tc.tp, tc.tr);
        if (res.ok) {
            printf("    Motor ① θ₁=%.1f°  →  τ₁ = %+.3f N·m\n", res.t1*RAD, res.m1);
            printf("    Motor ② θ₂=%.1f°  →  τ₂ = %+.3f N·m\n", res.t2*RAD, res.m2);
            printf("    Jacobian det = %.6f, κ = %.1f\n", res.det, res.cn);

            double eff_max = fmax(fabs(res.m1), fabs(res.m2));
            for (auto& m : MOTORS) {
                const char* status;
                if (eff_max <= m.rated) status = "🟢 Safe (within rated)";
                else if (eff_max <= m.peak) status = "🟡 Caution (within peak)";
                else status = "🔴 OVERLOAD";
                printf("      %s: %s\n", m.name, status);
            }
        } else {
            printf("    ✗ FAILED (unreachable or singular)\n");
        }
        printf("\n");
    }

    printf("=================================================================\n");
    return 0;
}
