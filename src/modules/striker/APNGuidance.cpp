#include "APNGuidance.h"
#include <cmath>

APNGuidance::APNGuidance(double n_gain) : N(n_gain) {}

Vec3 APNGuidance::computeCommand(const Vec3& R, const Vec3& V, const Vec3& a_t_est) {
    double R_mag = R.length();
    if (R_mag < 0.001) return Vec3(0, 0, 0);

    double V_closing = -R.dot(V) / R_mag;
    
    // Line of Sight (LOS) Rotation Vector
    // Omega = (R x V) / R^2
    Vec3 omega = R.cross(V) / (R_mag * R_mag);

    // --- Augmented Proportional Navigation ---
    // Standard Vector PN: a_pn = N * V_closing * (Omega x Unit(R_vector))
    // Direction of acceleration is perpendicular to R and Omega.
    
    Vec3 R_unit = R.normalized();
    Vec3 acc_pn = omega.cross(R_unit) * (N * V_closing);

    // 2. Augmented Term (Target Acceleration perpendicular to LOS)
    // We want the component of a_t that is perpendicular to R.
    // a_t_perp = a_t - (a_t . R_unit) * R_unit
    
    double at_proj = a_t_est.dot(R_unit);
    Vec3 at_perp = a_t_est - (R_unit * at_proj);

    // Total Command
    // a_cmd = a_pn + (N/2) * a_t_perp
    Vec3 total_acc = acc_pn + at_perp * (N / 2.0);

    return total_acc;
}

void APNGuidance::setGain(double n) {
    N = n;
}
