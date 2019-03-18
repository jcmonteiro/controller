#pragma once


#include <Eigen/Core>
#include <functional>


namespace controller
{


typedef Eigen::VectorXd Output;
typedef Eigen::VectorXd Input;
typedef int64_t Time;


class Controller
{
public:
    typedef std::function<void (Output &)> CallbackSaturation;
    typedef std::function<void (Output &)> CallbackPostProcessing;

private:
    Output output, output_presat;
    Time time_last;
    bool first_run;

    CallbackPostProcessing cb_postprocessing;
    CallbackSaturation cb_saturation;

protected:
    const unsigned int _N;

    /**
     * @brief Called every time #update is called so that the base class performs the actual control algorithm
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     * @param last_output The last output, defaults to getDefaultOutput during the first run.
     * @return The computed output.
     */
    virtual const Output & updateControl(Time time, const Input & ref, const Input & signal) = 0;

    /**
     * @brief Called during the first #update after a #restart is called
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     */
    virtual void configureFirstRun(Time time, const Input &ref, const Input &signal);

public:
    explicit Controller(unsigned int N_controllers);

    /**
     * @brief Checks if all parameters are set correctly.
     * @return True if everything is ok. False otherwise.
     */
    virtual bool ok() const = 0;

    /**
     * @brief Returns the default output.
     *
     * This method is called internally when update is called for a previous time instant.
     *
     * @return
     */
    virtual const Output & getDefaultOutput() const = 0;

    /**
     * @brief Restarts controller to its initial configuration.
     */
    virtual void restart();

    /**
     * @brief Configures a callback to be called after #update is finished to let the caller
     * modify the final output, possibly performing some post-processing.
     * @param callback The external callback.
     */
    void setCallbackPostProcessing(CallbackPostProcessing callback);

    /**
     * @brief Clears the post-processing callback.
     * @see #setCallbackPostProcessing
     */
    inline void clearCallbackPostProcessing()
    {
        cb_postprocessing = 0;
    }

    /**
     * @brief Clears the saturation callback.
     * @see #setCallbackSaturation
     */
    inline void clearCallbackSaturation()
    {
        cb_saturation = 0;
    }

    /**
     * @brief Configures a callback to be called after #update is finished to let the caller
     * saturate the final output.
     * @param callback The external callback.
     */
    void setCallbackSaturation(CallbackSaturation callback);

    /**
     * @brief Updates the control algorithm up to the specified \p time.
     *
     * To get the control output after the update is performed call #getOutput.
     *
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     * @see getOutput update(Time, const Input &, const Input &)
     */
    void update(Time time, const Input & ref, const Input & signal);

    inline const Output & getOutput() const
    {
        return output;
    }

    inline const Output & getOutputPreSat() const
    {
        return output_presat;
    }

    inline unsigned int size() const
    {
        return _N;
    }

};


}
