#include "SettingsPID.hpp"
#include <linear_system/HelperFunctions.hpp>
#include <cmath>
#include <stdexcept>


using namespace controller;


SettingsPID::SettingsPID(double kp, double ki, double kd) :
    kp(kp), ki(ki), kd(kd),
    weight_reference(1),
    gain_antiwidnup(0)
{
}

double SettingsPID::getSuggestedDerivativeCutoff() const
{
    return 4 * kd;
}

double SettingsPID::getSuggestedSampling() const
{
    return std::max(2 * M_PI / getSuggestedDerivativeCutoff() / 100, 1e-5);
}



SettingsPIDSecondOrder::SettingsPIDSecondOrder(double kp, double ki, double kd) :
    SettingsPID(kp, ki, kd), far_pole(0)
{
}

double SettingsPIDSecondOrder::getSettlingTime() const
{
    double damp = getDamping();
    // underdamped
    if (damp < 0.99)
        return -std::log(0.02*std::sqrt(1 - damp*damp))/damp/getNaturalFrequency();
    // critically damped
    else if (damp < 1.01)
        return 5.8 / getNaturalFrequency();
    // overdamped
    else if (damp < 1.1)
        return (5.8 + (damp - 1.01) * 1.1 / 0.09) / getNaturalFrequency();
    // "more" overdamped
    else if (damp < 5)
        return (6.9 + (damp - 1.1) * (38.8 - 6.9) / 4.9) / getNaturalFrequency();
    // "really" overdamped
    else
        return 38.8 / getNaturalFrequency();
}

double SettingsPIDSecondOrder::getOvershoot() const
{
    double damp = getDamping();
    // underdamped
    if (damp < 0.99)
        return std::exp( -damp * M_PI / std::sqrt(1 - damp*damp) );
    else
        return 0;
}

double SettingsPIDSecondOrder::getDamping() const
{
    return damp;
}

double SettingsPIDSecondOrder::getFarPole() const
{
    return far_pole;
}

double SettingsPIDSecondOrder::getNaturalFrequency() const
{
    return wn;
}

double SettingsPIDSecondOrder::getCutoffFrequency() const
{
    return linear_system::resonant2cutoff(getNaturalFrequency(), getDamping());
}

double SettingsPIDSecondOrder::getSuggestedDerivativeCutoff() const
{
    return 10 * getCutoffFrequency();
}

double SettingsPIDSecondOrder::getSuggestedSampling() const
{
    double sampling;
    sampling = 2 * M_PI / std::max(getFarPole(), getCutoffFrequency()) / 100;
    return std::max(sampling, 1e-5);
}

SettingsPIDSecondOrder SettingsPIDSecondOrder::createFromSpecT(double overshoot, double settling_time)
{
    if (overshoot < 0)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createT) overshoot must lie in [0, 0.5]");
    else if (overshoot > 0.5)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createT) overshoot must lie in [0, 0.5]");
    if (settling_time <= 0)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createT) settling time must be positive");
    double damp, cutoff;
    if (overshoot > 0)
    {
        double tmp = std::log(overshoot);
        damp = -tmp / std::sqrt(M_PI*M_PI + tmp*tmp);
        cutoff = linear_system::resonant2cutoff(-std::log(0.02 * std::sqrt(1-damp*damp)) / damp / settling_time, damp);
    }
    else
    {
        damp = 1;
        cutoff = 5.8337 / settling_time;
    }
    return createFromSpecF(damp, cutoff, 10);
}

SettingsPIDSecondOrder SettingsPIDSecondOrder::createFromSpecF(double damping, double cutoff, double far_pole_ratio)
{
    if (damping <= 0)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createF) damping must lie in (0, 2]");
    else if (damping > 2)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createF) damping must lie in (0, 2]");
    if (cutoff <= 0)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createF) cutoff frequency must be positive");
    if (far_pole_ratio <= 0)
        throw std::logic_error("[ERROR] (SettingsPIDSecondOrder::createF) far pole ratio must be positive");
    double wn = linear_system::cutoff2resonant(cutoff, damping);
    double kp = wn*wn * (1 + 2*damping*damping*far_pole_ratio);
    double ki = far_pole_ratio * damping * wn*wn*wn;
    double kd = damping * wn * (far_pole_ratio + 2);
    SettingsPIDSecondOrder ret(kp,ki,kd);
    ret.gain_antiwidnup = 1 / ki;
    ret.weight_reference = 1;
    ret.far_pole = far_pole_ratio * damping * wn;
    ret.damp = damping;
    ret.wn = wn;
    ret.wc = cutoff;
    return ret;
}

SettingsPIDSecondOrder SettingsPIDSecondOrder::createFromSpecF(double damping, double cutoff)
{
    double wn = linear_system::cutoff2resonant(cutoff, damping);
    double kp = wn*wn;
    double ki = 0;
    double kd = 2 * damping * wn;
    SettingsPIDSecondOrder ret(kp,ki,kd);
    ret.gain_antiwidnup = 0;
    ret.weight_reference = 1;
    ret.damp = damping;
    ret.wn = wn;
    ret.wc = cutoff;
    return ret;
}
