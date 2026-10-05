// Host-side verification of drivers::AttitudeReference against synthetic truth.
#define _USE_MATH_DEFINES
#include "drivers/imu_attitude.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

using drivers::Quat;
static const double D2R = M_PI / 180.0, R2D = 180.0 / M_PI;

static Quat mul(const Quat& a, const Quat& b) {
    return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
             a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
             a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
             a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
static Quat conj(const Quat& q) { return { q.w, -q.x, -q.y, -q.z }; }
static Quat axang(double ax, double ay, double az, double deg) {
    double n = std::sqrt(ax*ax + ay*ay + az*az), h = 0.5 * deg * D2R;
    return { std::cos(h), ax/n*std::sin(h), ay/n*std::sin(h), az/n*std::sin(h) };
}
static Quat zxy(double p, double r, double y) {   // Three.js 'ZXY': Rz(y) Rx(p) Ry(r)
    return mul(mul(axang(0,0,1,y), axang(1,0,0,p)), axang(0,1,0,r));
}
static double ang_err_deg(const Quat& a, const Quat& b) {
    double d = std::fabs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
    if (d > 1) d = 1;
    return 2.0 * std::acos(d) * R2D;
}
// gravity reaction (world +Z) in sensor frame for body->world q_s
static void up_sensor(const Quat& q, double g[3]) {
    g[0] = 9.81 * 2.0*(q.x*q.z - q.w*q.y);
    g[1] = 9.81 * 2.0*(q.y*q.z + q.w*q.x);
    g[2] = 9.81 * (1.0 - 2.0*(q.x*q.x + q.y*q.y));
}

static int fails = 0;
static void check(bool ok, const char* what, double v) {
    printf("  [%s] %-58s %.4f\n", ok ? "PASS" : "FAIL", what, v);
    if (!ok) ++fails;
}

struct Rig {
    Quat q_m_true;     // vehicle -> sensor (snap * misalignment), unknown to the filter
    bool report_conj;  // sensor reports world->body
    drivers::AttitudeReference ref;
    drivers::AttitudeOutput last;
    void feed(const Quat& q_vehicle_world, int n, double gyro_mag = 0.0) {
        Quat q_s = mul(q_vehicle_world, conj(q_m_true));   // sensor -> world
        double g[3]; up_sensor(q_s, g);
        double w[3] = { gyro_mag, 0, 0 };
        Quat rep = report_conj ? conj(q_s) : q_s;
        for (int i = 0; i < n; ++i) last = ref.update(rep, g, w, 0.01);
    }
};

static void scenario(const char* name, Quat snap, Quat misalign, bool conj_rep, double heading0) {
    printf("\n=== %s ===\n", name);
    Rig rig;
    rig.q_m_true = mul(misalign, snap);   // misalignment applied in sensor frame
    rig.report_conj = conj_rep;
    const Quat yaw0 = axang(0,0,1,heading0);

    // Boot: vehicle level & still for 1.5 s
    rig.feed(yaw0, 150);
    check(rig.last.referenced, "referenced after boot still period", rig.ref.reference_count());
    printf("  mount = %s\n", drivers::mount_class_name(rig.last.mount));
    check(ang_err_deg(rig.last.q_rel, Quat{}) < 0.05, "rest attitude == identity (deg)", ang_err_deg(rig.last.q_rel, Quat{}));
    check(std::fabs(rig.last.tilt_x_deg) < 0.05 && std::fabs(rig.last.tilt_y_deg) < 0.05,
          "rest tilt_x/tilt_y ~ 0", std::fabs(rig.last.tilt_x_deg) + std::fabs(rig.last.tilt_y_deg));

    // Arbitrary attitudes incl. near and at gimbal lock
    const double cases[][3] = { {20,0,0},{0,30,0},{0,0,45},{-35,60,200},{89.0,40,10},{89.99,40,10},
                                {90,-25,300},{-90,70,120},{-89.97,-150,33},{0,180,0},{45,170,350} };
    double worst_q = 0, worst_rebuild = 0;
    for (auto& c : cases) {
        Quat truth = zxy(c[0], c[1], c[2]);
        rig.feed(mul(yaw0, truth), 3, 1.0);   // moving (no re-reference)
        double e = ang_err_deg(rig.last.q_rel, truth);
        Quat rebuilt = zxy(rig.last.tilt_x_deg, rig.last.tilt_y_deg, rig.last.rot_z_deg);
        double eb = ang_err_deg(rebuilt, truth);
        if (e > worst_q) worst_q = e;
        if (eb > worst_rebuild) worst_rebuild = eb;
        printf("    truth p=%7.2f r=%7.2f y=%7.2f -> out p=%7.2f r=%7.2f y=%7.2f | q err %.4f  euler-rebuild err %.4f\n",
               c[0], c[1], c[2], rig.last.tilt_x_deg, rig.last.tilt_y_deg, rig.last.rot_z_deg, e, eb);
    }
    check(worst_q < 0.05, "worst quaternion error over cases (deg)", worst_q);
    check(worst_rebuild < 0.1, "worst ZXY-rebuild error incl. gimbal lock (deg)", worst_rebuild);

    // Continuous sweep through gimbal lock: tilt_x 0 -> 120 deg with roll 30, yaw 50.
    // Check the output quaternion never jumps and Euler rebuild stays exact.
    double max_step = 0, max_rb = 0; Quat prev = rig.last.q_rel;
    for (int k = 0; k <= 1200; ++k) {
        double p = 0.1 * k;
        Quat truth = zxy(p, 30, 50);
        rig.feed(mul(yaw0, truth), 1, 1.0);
        double step = ang_err_deg(rig.last.q_rel, prev);
        double dot = rig.last.q_rel.w*prev.w + rig.last.q_rel.x*prev.x + rig.last.q_rel.y*prev.y + rig.last.q_rel.z*prev.z;
        if (dot < 0) step = 999;  // hemisphere flip would be a visible glitch for slerp
        if (k > 0 && step > max_step) max_step = step;
        prev = rig.last.q_rel;
        double rb = ang_err_deg(zxy(rig.last.tilt_x_deg, rig.last.tilt_y_deg, rig.last.rot_z_deg), truth);
        if (rb > max_rb) { max_rb = rb; if (rb > 0.1) printf("    sweep k=%d p=%.1f out p=%.3f r=%.3f y=%.3f rb=%.4f\n", k, p, rig.last.tilt_x_deg, rig.last.tilt_y_deg, rig.last.rot_z_deg, rb); }
    }
    check(max_step < 0.2, "sweep through 90 deg: max step between samples (deg)", max_step);
    check(max_rb < 0.1, "sweep through 90 deg: max Euler-rebuild error (deg)", max_rb);

    // Tare while tilted on a new mount (vehicle laid on its side and still): must re-identify
    Quat side = mul(yaw0, axang(1,0,0,90));
    rig.feed(side, 50);
    uint32_t rc0 = rig.ref.reference_count();
    rig.ref.request_tare();
    rig.feed(side, 2);
    check(rig.ref.reference_count() == rc0 + 1, "tare re-referenced", rig.ref.reference_count());
    check(ang_err_deg(rig.last.q_rel, Quat{}) < 0.05, "post-tare attitude == identity (deg)", ang_err_deg(rig.last.q_rel, Quat{}));
    printf("  mount after tare on side = %s\n", drivers::mount_class_name(rig.last.mount));

    // No auto-remount by default: holding on its side 10 s must NOT snap back
    rig.ref.request_tare(); rig.feed(yaw0, 20);   // back to upright reference
    rc0 = rig.ref.reference_count();
    rig.feed(side, 1000);
    check(rig.ref.reference_count() == rc0, "held on side 10 s: no silent re-reference", rig.ref.reference_count());
    check(std::fabs(std::fabs(rig.last.tilt_x_deg) - 90.0) < 0.1, "held on side shows |tilt_x| = 90", rig.last.tilt_x_deg);
}

int main() {
    const double S = std::sqrt(0.5);
    Quat none{};
    // Perpendicular bench mount (sensor +X up) with a 3.5 deg hand-mounting error
    scenario("PERPENDICULAR X_UP, 3.5 deg misalignment, body->world", Quat{S,0,S,0}, axang(0,1,0,3.5), false, 37);
    scenario("PERPENDICULAR Y_DOWN, 2 deg misalignment, world->body (conj)", Quat{S,S,0,0}, axang(1,0,0,-2.0), true, 210);
    scenario("PERPENDICULAR X_DOWN, conj", Quat{S,0,-S,0}, none, true, 0);
    scenario("PARALLEL Z_UP (flight), 1.5 deg misalignment", none, axang(1,1,0,1.5), false, 123);
    scenario("PARALLEL Z_DOWN", Quat{0,1,0,0}, none, false, 300);
    printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
