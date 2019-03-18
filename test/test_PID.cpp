#define BOOST_TEST_MODULE LinearSystem test

#include <boost/test/unit_test.hpp>
#include <iostream>
#include "PID.hpp"
#include <fstream>

using namespace controller;

void printProgress(int width, float progress)
{
    std::cout << "[";
    int pos = width * progress;
    for (int i = 0; i < width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
    if (progress == 1.0)
        std::cout << std::endl;
}

bool equal_eigen(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2)
{
    return (m1.array() == m2.array()).all();
}

bool different_eigen(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2)
{
    return (m1.array() != m2.array()).any();
}

void testReset(Controller &controller)
{
    // run twice and see if we get the same results
    // this should verify that restart is working properly
    const unsigned int N = controller.size();
    Input ref(N), sig(N);
    ref << 1, 0.8;
    ref << 0.5, 0.6;
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    std::vector<Output> history(n_turns);
    controller.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig
        );
        history[k] = controller.getOutput();
    }
    controller.restart();
    time = 0;
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig
        );
        if ( ((controller.getOutput() - history[k]).array() != 0).any() )
            BOOST_ERROR("found different output values when running the same controller twice");
    }
}

void testInputChannels(Controller &controller)
{
    const unsigned int N = controller.size();
    Input ref(N), sig(N);
    ref.setConstant(23.43);
    sig.setConstant(11.33);
    double time = 0, dt = 1;
    unsigned int n_turns = 10;
    controller.restart();
    for (unsigned int k = 0; k < n_turns; ++k)
    {
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds( time += dt ),
            ref,
            sig
        );
        Output out = controller.getOutput();
        if ( ((out.array() - out[0]) != 0).any() )
            BOOST_ERROR("found different output values while sending the same inputs to all controller channels");
    }
}

linear_system::LinearSystem getDoubleIntegrator(double sampling)
{
    linear_system::LinearSystem plant;
    Eigen::VectorXd num(3), den(3);
    num << 0, 0, 1;
    den << 1, 0, 0;
    plant.setFilter(num, den);
    plant.setSampling(sampling);
    Eigen::MatrixXd init_out(1, 2);
    init_out << 0, 0;
    plant.setInitialOutputDerivatives(init_out);
    plant.discretizeSystem();
    return plant;
}

void testStepResponse(Controller &controller, const SettingsPID &settings, double sampling, double time_settling, double max_overshoot, double tol_settling_error)
{
    Input ref(1);
    ref.setConstant(1);

    auto plant = getDoubleIntegrator(sampling);

    linear_system::LinearSystem prefilter;
    Eigen::VectorXd num_filter(3), den_filter(3);
    num_filter << 0, pow(settings.getNaturalFrequency(), 2), settings.ki;
    den_filter << settings.kd, settings.kp, settings.ki;
    prefilter.setFilter(num_filter, den_filter);
    prefilter.setSampling(sampling);
    Eigen::MatrixXd init_out_filter(1, 2);
    init_out_filter << plant.getOutput()[0], 0;
    prefilter.setInitialOutputDerivatives(init_out_filter);
    prefilter.discretizeSystem();

    Eigen::MatrixXd init_in(1, 2), init_in_filter(1, 2);
    init_in.setZero();
    init_in_filter << ref[0], ref[0];
    plant.setInitialState(init_in);
    prefilter.setInitialState(init_in_filter);

    double time = 0, dt = sampling;
    double ratio = 0;
    double overshoot = 0;
    double time_overshoot = 0;
    plant.setInitialTime(time);
    prefilter.setInitialTime(time);
    controller.restart();
    while (time < time_settling)
    {
        prefilter.update(
            ref,
            linear_system::LinearSystem::getTimeFromSeconds(time)
        );
        controller.update(
            linear_system::LinearSystem::getTimeFromSeconds(time),
            prefilter.getOutput(),
            plant.getOutput()
        );
        plant.update(controller.getOutput(), linear_system::LinearSystem::getTimeFromSeconds(time));
        ratio = std::abs( plant.getOutput()[0]/ref[0] - 1 );
        if (plant.getOutput()[0] > ref[0] && ratio > overshoot)
        {
            overshoot = ratio;
            time_overshoot = time;
        }
        time += dt;
    }
    if (overshoot > max_overshoot)
    {
        std::stringstream error;
        error << "PID exceeded the maximum overshoot at t = " << time_overshoot/time_settling
              << " ts : (max_overshoot, overshoot) = ("
              << max_overshoot << " , " << overshoot << ")";
        BOOST_ERROR(error.str());
    }
    if (ratio >= tol_settling_error)
    {
        std::stringstream error;
        error << "PID did not meet the required final performance: "
              << "(ref , out) = (" << 1 << " , " << plant.getOutput()[0]/ref[0]
              << ")";
        BOOST_ERROR(error.str());
    }
}

void testFrequencyResponse(Controller &controller, const SettingsPID &settings, double sampling, double damp, double cutoff, double farpole)
{
    double wn = cutoff / damp;
    double dt = sampling;

    auto plant = getDoubleIntegrator(sampling);

    linear_system::LinearSystem prefilter;
    Eigen::VectorXd num_filter(3), den_filter(3);
    num_filter << 0, pow(settings.getNaturalFrequency(), 2), settings.ki;
    den_filter << settings.kd, settings.kp, settings.ki;
    prefilter.setFilter(num_filter, den_filter);
    prefilter.setSampling(sampling);
    Eigen::MatrixXd init_out_filter(1, 2);
    init_out_filter << 0, 0;
    prefilter.setInitialOutputDerivatives(init_out_filter);
    prefilter.discretizeSystem();

    linear_system::LinearSystem model_closedloop;
    Eigen::VectorXd num_model(4), den_model(4);
    num_model << 0, farpole + 2*damp*wn, wn*wn + farpole*2*damp*wn, farpole*wn*wn;
    den_model << 1, farpole + 2*damp*wn, wn*wn + farpole*2*damp*wn, farpole*wn*wn;
    model_closedloop.setFilter(num_model, den_model);
    model_closedloop.setSampling(sampling);
    Eigen::MatrixXd init_out_model(1, 3);
    init_out_model << 0, 0, 0;
    model_closedloop.setInitialOutputDerivatives(init_out_model);
    model_closedloop.discretizeSystem();
    //
    Eigen::MatrixXd init_in_model(1, 3);
    init_in_model.setZero();

    Eigen::MatrixXd init_in(1, 2), init_in_filter(1, 2);
    init_in.setZero();
    init_in_filter.setZero();

    Input ref(1);

    double input_freq_array[] = {cutoff / 10, cutoff, wn};
    for (double input_freq : input_freq_array)
    {
        double T = 2*M_PI / input_freq;
        double t_final = 10 * T;
        double t_transient = t_final - 2*T;
        double time = 0;
        plant.setInitialState(init_in);
        prefilter.setInitialState(init_in_filter);
        model_closedloop.setInitialState(init_in_model);
        plant.setInitialTime(time);
        prefilter.setInitialTime(time);
        model_closedloop.setInitialTime(time);
        controller.restart();
        //
        double amplitude = 0, max_error = 0;
        //
        while (time < t_final)
        {
            auto time_sim = linear_system::LinearSystem::getTimeFromSeconds(time);
            ref[0] = std::sin(input_freq * time);
            prefilter.update(
                ref,
                time_sim
            );
            controller.update(
                time_sim,
                prefilter.getOutput(),
                plant.getOutput()
            );
            plant.update(controller.getOutput(), time_sim);

            model_closedloop.update(prefilter.getOutput(), time_sim);

            if (time > t_transient)
            {
                double amplitude_now = model_closedloop.getOutput()[0];
                double error = std::abs(amplitude_now - plant.getOutput()[0]);
                if (error > max_error)
                    max_error = error;
                if (amplitude_now > amplitude)
                    amplitude = amplitude_now;
            }
            time += dt;
        }
        if (max_error / amplitude > 0.1)
            BOOST_ERROR("PID did not meet the required frequency response");
    }
}

BOOST_AUTO_TEST_CASE(test_channels)
{
    std::cout << "[TEST] same input to multiple channels" << std::endl;
    //
    PID pid(2);

    SettingsPID settings;
    settings.kp = 1;
    settings.kd = 0.2;
    settings.ki = 2;

    pid.configure({settings, settings}, 0.2);
    //
    testReset(pid);
}

BOOST_AUTO_TEST_CASE(test_reset)
{
    std::cout << "[TEST] reset" << std::endl;
    //
    unsigned int N = 20;
    PID pid(N);

    SettingsPID settings;
    settings.kp = 1;
    settings.kd = 0.2;
    settings.ki = 2;

    std::vector<SettingsPID> settings_all(N);
    for (auto &s : settings_all)
        s = settings;

    pid.configure(settings_all, 0.2);
    //
    testInputChannels(pid);
}

BOOST_AUTO_TEST_CASE(test_step_response)
{
    std::cout << "[TEST] step response" << std::endl;
    //
    PID pid(1);
    double ts_array[] = {0.1, 1, 10, 100, 1000, 10000};
    double overshoot_array[] = {0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5};
    double tol_ratio_overshoot = 1.06;
    double tol_settling_error = 0.025;
    for (double over : overshoot_array)
    {
        for (double ts : ts_array)
        {
            auto settings = SettingsPID::createFromSpecT(over, ts);
            double sampling = settings.getSuggestedSampling();
            pid.configure(settings, sampling);
            //
            testStepResponse(pid, settings, sampling, ts, tol_ratio_overshoot * over, tol_settling_error);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_frequency_response)
{
    std::cout << "[TEST] frequency response" << std::endl;
    //
    PID pid(1);
    double cutoff_array[] = {0.01, 0.1, 1, 10, 100, 1000};
    double damp_array[] = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    for (double cutoff : cutoff_array)
    {
        for (double damp : damp_array)
        {
            auto settings = SettingsPID::createFromSpecF(damp, cutoff, 10);
            double wn = cutoff / damp;
            double sampling = (2*M_PI/wn) / 1000;
            auto velocity = SettingsFilter::createSecondOrder(0.7, wn * 200, 1);
            velocity.sampling_period = sampling;
            pid.configure(settings, velocity);
            //
            testFrequencyResponse(pid, settings, sampling, damp, cutoff, settings.getFarPole());
        }
    }
}

BOOST_AUTO_TEST_CASE(test_saturation_cb)
{
    std::cout << "[TEST] saturation callback" << std::endl;
    //
    PID pid(1);
    auto settings = SettingsPID::createFromSpecT(0.05, 1);
    double sampling = settings.getSuggestedSampling();
    pid.configure(settings, sampling);
    //
    Input in(1), ref(1);
    in[0] = 0;
    ref[0] = 0;
    pid.update(0, ref, in);
    ref[0] = 100000000;
    //
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(5*sampling), ref, in);
    BOOST_ASSERT( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    //
    int k = 0;
    pid.setCallbackSaturation(
        [&k] (Output &out) -> void {
            out.setConstant(123);
            k = 321;
        }
    );
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(10*sampling), ref, in);
    BOOST_ASSERT(k == 321);
    BOOST_ASSERT( different_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    BOOST_ASSERT( (pid.getOutput().array() == 123).all() );
}

BOOST_AUTO_TEST_CASE(test_postprocessing_cb)
{
    std::cout << "[TEST] post-processing callback" << std::endl;
    //
    PID pid(1);
    SettingsPID settings;
    settings.kp = 1;
    settings.ki = 0;
    settings.kd = 0;
    double sampling = 1;
    pid.configure(settings, sampling);
    //
    Input in(1), ref(1);
    in[0] = 0;
    ref[0] = 0;
    pid.update(0, ref, in);
    ref[0] = 1;
    //
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(5*sampling), ref, in);
    BOOST_ASSERT( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    //
    int k = 0;
    pid.setCallbackPostProcessing(
        [&k] (Output &out) -> void {
            out.setConstant(123);
            k = 321;
        }
    );
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(10*sampling), ref, in);
    BOOST_ASSERT(k == 321);
    BOOST_ASSERT( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    BOOST_ASSERT( (pid.getOutput().array() == 123).all() );
}
