#pragma once


#include <Eigen/Core>


namespace controller
{


typedef Eigen::VectorXd Output;
typedef Eigen::VectorXd Input;
typedef int64_t Time;


class Controller
{
private:
    Output output;
    Time time_last;
    bool first_run;

protected:
    const unsigned int _N;

    /**
     * @brief updateControl Called every time #update is called so that the base class performs the actual control algorithm
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     * @param last_output The last output, defaults to getDefaultOutput during the first run.
     * @return The computed output.
     */
    virtual const Output & updateControl(Time time, const Input & ref, const Input & signal, const Output & last_output) = 0;

    /**
     * @brief configureFirstRun Called during the first #update after a #restart is called
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     */
    virtual void configureFirstRun(Time time, const Input &ref, const Input &signal);

public:
    explicit Controller(unsigned int N_controllers);

    /**
     * @brief ok Checks if all parameters are set correctly.
     * @return True if everything is ok. False otherwise.
     */
    virtual bool ok() const = 0;

    /**
     * @brief getDefaultOutput Returns the default output.
     *
     * This method is called internally when update is called for a previous time instant.
     *
     * @return
     */
    virtual const Output & getDefaultOutput() const = 0;

    /**
     * @brief restart Restarts controller to its initial configuration.
     */
    virtual void restart();

    /**
     * @brief update Updates the control algorithm up to the specified \p time.
     *
     * To get the control output after the update is performed call #getOutput.
     *
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     * @param last_output Last output value, possibly different from what is returned by
     * #getOutput if some post-processing is done externally.
     * @see getOutput update(Time, const Input &, const Input &)
     */
    void update(Time time, const Input & ref, const Input & signal, const Output & last_output);

    /**
     * @brief update Calls update sending the last computed output as the last output value.
     * @param time Current time.
     * @param ref Reference.
     * @param signal Signal.
     */
    inline void update(Time time, const Input & ref, const Input & signal)
    {
        update(time, ref, signal, output);
    }

    inline const Output & getOutput() const
    {
        return output;
    }

    inline unsigned int size() const
    {
        return _N;
    }

};


}
