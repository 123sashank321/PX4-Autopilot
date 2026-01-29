#ifndef APN_GUIDANCE_H
#define APN_GUIDANCE_H

#include "Vec3.h"

class APNGuidance {
public:
    double N; // Navigation Constant

    APNGuidance(double n_gain = 3.0);

    /**
     * Computes the acceleration command vector using Augmented Proportional Navigation.
     * 
     * @param R Relative position vector (Target - Interceptor)
     * @param V Relative velocity vector (Target - Interceptor)
     * @param a_t_est Estimated target acceleration vector
     * @return Commanded acceleration vector
     */
    Vec3 computeCommand(const Vec3& R, const Vec3& V, const Vec3& a_t_est);
    
    // Setters
    void setGain(double n);
};

#endif
