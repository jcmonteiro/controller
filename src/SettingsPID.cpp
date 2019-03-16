#include "SettingsPID.hpp"
#include <cmath>
#include <stdexcept>


using namespace controller;


SettingsPID::SettingsPID() :
    far_pole(0),
    kp(0), ki(0), kd(0),
    weight_reference(1),
    gain_antiwidnup(0)
{
}

double SettingsPID::getSettlingTime()
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

double SettingsPID::getOvershoot()
{
    double damp = getDamping();
    // underdamped
    if (damp < 0.99)
        return std::exp( -damp * M_PI / std::sqrt(1 - damp*damp) );
    else
        return 0;
}

double SettingsPID::getDamping()
{
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

double SettingsPID::getFarPole()
{
    if (far_pole != 0)
        return far_pole;

    // otherwise, estimate the position
    if (ki > 0 && kd > 0)
    {
        double tmp = std::pow(ki/2 - (kd*kp)/6 + std::pow(kd , 3)/27 , 1.0/3.0);
        return kd/3 + tmp - (kp/3 - kd*kd/9) / tmp;
    }
    else
        return 0;
}

double SettingsPID::getNaturalFrequency()
{
    // PD
    if (ki == 0)
        return std::sqrt(kp);
    // PI
    else if (kd == 0)
        return std::sqrt(ki);
    // PID
    else
        return std::sqrt(ki / getFarPole());
}

double SettingsPID::getCutoffFrequency()
{
    return getDamping() * getNaturalFrequency();
}

double SettingsPID::getSuggestedSampling()
{
    double far_pole = getFarPole();
    double sampling;
    if (far_pole == 0 || far_pole < 1)
        sampling = M_PI_2 / getCutoffFrequency();
    else
        sampling = M_PI_2 / getCutoffFrequency() / far_pole;
    return std::min(sampling, getSettlingTime() / 100);
}

SettingsPID SettingsPID::createFromSpecT(double overshoot, double settling_time)
{
    if (overshoot < 0)
        throw std::logic_error("[ERROR] (SettingsPID::createT) overshoot must lie in [0, 0.5]");
    else if (overshoot > 0.5)
        throw std::logic_error("[ERROR] (SettingsPID::createT) overshoot must lie in [0, 0.5]");
    if (settling_time <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createT) settling time must be positive");
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

SettingsPID SettingsPID::createFromSpecF(double damping, double cutoff, double far_pole_ratio)
{
    if (damping <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) damping must lie in (0, 2]");
    else if (damping > 2)
        throw std::logic_error("[ERROR] (SettingsPID::createF) damping must lie in (0, 2]");
    if (cutoff <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) cutoff frequency must be positive");
    if (far_pole_ratio <= 0)
        throw std::logic_error("[ERROR] (SettingsPID::createF) far pole ratio must be positive");
    SettingsPID ret;
    double wn = cutoff / damping;
    ret.kp = wn*wn * (1 + 2*damping*damping*far_pole_ratio);
    ret.ki = far_pole_ratio * damping * wn*wn*wn;
    ret.kd = damping * wn * (far_pole_ratio + 2);
    ret.gain_antiwidnup = 1 / ret.ki;
    ret.weight_reference = 1;
    ret.far_pole = far_pole_ratio * damping * wn;
    return ret;
}

SettingsPID SettingsPID::createFromSpecF(double damping, double cutoff)
{
    SettingsPID ret;
    double wn = cutoff / damping;
    ret.kp = wn*wn;
    ret.ki = 0;
    ret.kd = 2 * damping * wn;
    ret.gain_antiwidnup = 0;
    ret.weight_reference = 1;
    return ret;
}
