/**Collection of classes related to PID control.
 *
 * Implements classes for
 * 	- PID
 * 	- PID auto-tuning
 * 	- extracting step response properties
 *
 * \author  Ajish Babu (ajish.babu@dfki.de)
 */

#pragma once

#include <vector>

namespace pid_control
{
    //! Structure to hold the PID parameters for 'PARALLEL' type PID.
    struct ParallelPIDSettings
    {
		//! Sampling time in seconds
	    double Ts;

		//! Proportional gain
	    double Kp;

		//! Integral gain
	    double Ki;

		//! Derivative gain
	    double Kd;

		//! Derivative term
		/** Derivative term filtered by a first order system with time constant Td/N
		 * - Typical values of N are between 8 and 20
		 * - No derivative action on frequencies above N/Td
		 */
	    double N;

		//! Setpoint weighing term
		/** Setpoint weighing term, generally between 0 and 1
		 * - B = 0 reference is introduced only through integral term
		 * - B = 1 disables setpoint weighting
		 */
	    double B;

		//! Anti-integrator-windup
		/** Anti-integrator-windup time constant
		 * - < 0 disable
		 * - = 0 and Td = 0  disable
		 * - = 0 and Td > 0  Tt = sqrt(Ti * Td)
		 * */
	    double Tt;


		//! Minimum output value
	    double YMin;
		//! Maximum output value
	    double YMax;

		//! Constructor
        ParallelPIDSettings():Ts(0),Kp(0),Ki(0),Kd(0),N(0),B(1),Tt(-1),YMin(0),YMax(0){}

        bool operator==(const ParallelPIDSettings& other) const{
            return
                Ts == other.Ts &&
                Kp == other.Kp &&
                Ki == other.Ki &&
                Kd == other.Kd &&
                N == other.N &&
                B == other.B &&
                Tt == other.Tt &&
                YMin == other.YMin &&
                YMax == other.YMax;
        }

        bool operator!=(const ParallelPIDSettings& other) const{
            return !(*this == other);
        }

	    void setIdealCoefficients(
		    double _K = 0,
		    double _Ti = 0,
		    double _Td = 0);
    };
}
