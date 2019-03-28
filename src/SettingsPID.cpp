#include "SettingsPID.hpp"
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
    return 10 * kd;
}

double SettingsPID::getSuggestedSampling() const
{
    return 2 * M_PI / getSuggestedDerivativeCutoff() / 5;
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
    double kp = getKp();
    double ki = getKi();
    double kd = getKd();
    // PD
    if (ki == 0)
        return kd / 2 / getNaturalFrequency();
    // PI
    else if (kd == 0)
        return kp / 2 / getNaturalFrequency();
    // PID
    else
    {
        return (kd - getFarPole()) / 2 / getNaturalFrequency();
    }
}

double SettingsPIDSecondOrder::getFarPole() const
{
    if (far_pole != 0)
        return far_pole;

    // otherwise, estimate the position
    double ki = getKi();
    double kd = getKd();
    if (ki > 0 && kd > 0)
    {
        double kp = getKp();
        double tmp = std::pow(ki/2 - (kd*kp)/6 + std::pow(kd , 3)/27 , 1.0/3.0);
        return kd/3 + tmp - (kp/3 - kd*kd/9) / tmp;
    }
    else
        return 0;
}

double SettingsPIDSecondOrder::getNaturalFrequency() const
{
    double ki = getKi();
    double kd = getKd();
    // PD
    if (ki == 0)
        return std::sqrt(getKp());
    // PI
    else if (kd == 0)
        return std::sqrt(getKi());
    // PID
    else
        return std::sqrt(getKi() / getFarPole());
}

double SettingsPIDSecondOrder::getCutoffFrequency() const
{
    return getDamping() * getNaturalFrequency();
}

double SettingsPIDSecondOrder::getSuggestedDerivativeCutoff() const
{
    return 10 / getCutoffFrequency();
}

double SettingsPIDSecondOrder::getSuggestedSampling() const
{
    double far_pole = getFarPole();
    double sampling;
    if (far_pole == 0 || far_pole < 1)
        sampling = M_PI_2 / getCutoffFrequency();
    else
        sampling = M_PI_2 / getCutoffFrequency() / far_pole;
    return std::min(sampling, getSettlingTime() / 100);
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
        cutoff = -std::log(0.02 * std::sqrt(1-damp*damp)) / settling_time;
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
    double wn = cutoff / damping;
    double kp = wn*wn * (1 + 2*damping*damping*far_pole_ratio);
    double ki = far_pole_ratio * damping * wn*wn*wn;
    double kd = damping * wn * (far_pole_ratio + 2);
    SettingsPIDSecondOrder ret(kp,ki,kd);
    ret.gain_antiwidnup = 1 / ki;
    ret.weight_reference = 1;
    ret.far_pole = far_pole_ratio * damping * wn;
    return ret;
}

SettingsPIDSecondOrder SettingsPIDSecondOrder::createFromSpecF(double damping, double cutoff)
{
    double wn = cutoff / damping;
    double kp = wn*wn;
    double ki = 0;
    double kd = 2 * damping * wn;
    SettingsPIDSecondOrder ret(kp,ki,kd);
    ret.gain_antiwidnup = 0;
    ret.weight_reference = 1;
    return ret;
}
