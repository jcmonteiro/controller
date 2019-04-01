#include "SettingsPPI.hpp"
#include <linear_system/HelperFunctions.hpp>
#include <cmath>
#include <stdexcept>


using namespace controller;


void SettingsPPI::ppi2pid(double kp_out, double kp_in, double ki_in, double &kp, double &ki, double &kd)
{
    kp = (kp_out + 1) * ki_in;
    ki = kp_out * ki_in;
    kd = kp_in;
}

double SettingsPPI::getSuggestedDerivativeCutoff() const
{
    return 10 * wc_inner;
}

double SettingsPPI::getSuggestedSampling() const
{
    return 2 * M_PI / std::max(wc_inner, wc_outer) / 100;
}

SettingsPPI SettingsPPI::createFromSpec(double cutoff_outer, double cutoff_inner, double damping)
{
    double wn    = linear_system::cutoff2resonant(cutoff_inner, damping);
    double kp_ou = cutoff_outer;
    double kp_in = 2*damping*wn;
    double ki_in = wn*wn;
    double kp, ki, kd;
    ppi2pid(kp_ou, kp_in, ki_in, kp, ki, kd);
    SettingsPPI ret(kp, ki, kd);
    ret.wc_outer = cutoff_outer;
    ret.wc_inner = cutoff_inner;
    return ret;
}
