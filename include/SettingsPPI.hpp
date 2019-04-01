#pragma once


#include "SettingsPID.hpp"


namespace controller
{


class SettingsPPI : public SettingsPID
{
private:
    //! Outer loop (P-loop) cutoff frequency
    double wc_outer;

    //! Inner loop (PI-loop) cutoff frequency
    double wc_inner;

    /**
     * @brief Hides the constructor because instances of this class should only be created via
     * #createFromSpecF and #createFromSpecT.
     */
    inline SettingsPPI(double kp, double ki, double kd) :
        SettingsPID(kp, ki, kd)
    {}

public:
    static void ppi2pid(double kp_out, double kp_in, double ki_in, double &kp, double &ki, double &kd);

    double getSuggestedDerivativeCutoff() const;
    double getSuggestedSampling() const;

    static SettingsPPI createFromSpec(double cutoff_outer, double cutoff_inner, double damping = 0.7071);
};


}
