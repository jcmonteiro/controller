#include "PID.hpp"

#include <linear_system/HelperFunctions.hpp>

#include <gtest/gtest.h>
#include <iostream>

using namespace controller;

void printProgress(float progress)
{
    std::cout << "[";
    int width = 50;
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
        EXPECT_FALSE( ((controller.getOutput() - history[k]).array() != 0).any() ) << "found different output values when running the same controller twice" << std::endl;
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
        EXPECT_FALSE( ((out.array() - out[0]) != 0).any() ) << "found different output values while sending the same inputs to all controller channels" << std::endl;
    }
}

linear_system::LinearSystem getDoubleIntegrator(double sampling)
{
    Eigen::VectorXd num(3), den(3);
    num << 0, 0, 1;
    den << 1, 0, 0;
    linear_system::LinearSystem plant(num, den, sampling);
    plant.setInitialConditions(
        Eigen::MatrixXd::Zero(1, 2),
        Eigen::MatrixXd::Zero(1, 2)
    );
    return plant;
}

void testStepResponse(Controller &controller, const SettingsPIDSecondOrder &settings, double sampling,
    double time_settling, double max_overshoot, double tol_settling_error, double tol_diff_model)
{
    Input ref(1);
    ref.setConstant(1);

    auto plant = getDoubleIntegrator(sampling);

    double wn = settings.getNaturalFrequency();
    double damp = settings.getDamping();
    double far = settings.getFarPole();

    Eigen::VectorXd num_filter(3), den_filter(3);
    num_filter << 0, 0, settings.getKi();
    den_filter << settings.getKd(), settings.getKp(), settings.getKi();
    linear_system::LinearSystem prefilter(num_filter, den_filter, sampling);

    num_filter << settings.getKi(), 0, 0;
    den_filter << settings.getKd(), settings.getKp(), settings.getKi();
    linear_system::LinearSystem feedforward(num_filter, den_filter, sampling);
    controller.setCallbackPostProcessing(
        [&feedforward] (Output &out) -> void {
            out += feedforward.getOutput();
        }
    );

    Eigen::VectorXd num_model(1), den_model(4);
    num_model << wn*wn*far;
    den_model << 1, 2*damp*wn+far, 2*damp*wn*far+wn*wn, far*wn*wn;
    linear_system::LinearSystem model(num_model, den_model, sampling);

    Eigen::MatrixXd init_out_filter(1, 2);
    init_out_filter << plant.getOutput()[0], 0;
    prefilter.setInitialConditions(
        Eigen::MatrixXd::Constant(1,2, ref[0]),
        init_out_filter);
    feedforward.setInitialConditions(
        Eigen::MatrixXd::Constant(1,2, ref[0]),
        Eigen::MatrixXd::Zero(1,2));
    model.setInitialConditions(
        Eigen::MatrixXd::Constant(1,3, ref[0]),
        Eigen::MatrixXd::Zero(1,3)
    );

    linear_system::Time time = 0, dt = linear_system::LinearSystem::getTimeFromSeconds(sampling);
    double ratio = 0;
    double overshoot = 0;
    double time_overshoot = 0;
    plant.setInitialTime(time);
    prefilter.setInitialTime(time);
    feedforward.setInitialTime(time);
    model.setInitialTime(time);
    controller.restart();
    linear_system::Time time_settling_micro = linear_system::LinearSystem::getTimeFromSeconds(time_settling);
    linear_system::Time samples = (time_settling_micro / dt) + 2;
    Eigen::MatrixXd data(samples, 3);
    unsigned int k = 0;
    while (time < time_settling_micro)
    {
        prefilter.update(
            ref,
            time
        );
        feedforward.update(
            ref,
            time
        );
        controller.update(
            time,
            prefilter.getOutput(),
            plant.getOutput()
        );
        plant.update(controller.getOutput(), time);
        ratio = std::abs( plant.getOutput()[0]/ref[0] - 1 );
        if (plant.getOutput()[0] > ref[0] && ratio > overshoot)
        {
            overshoot = ratio;
            time_overshoot = time;
        }
        data(k,0) = time;
        data(k,1) = model.update(ref, time)[0];
        data(k,2) = plant.getOutput()[0];
        ++k;
        time += dt;

    }
    data.conservativeResize(k, Eigen::NoChange);

    EXPECT_FALSE(overshoot > max_overshoot) << "PID exceeded the maximum overshoot at t = " << time_overshoot/time_settling << " ts : (max_overshoot, overshoot) = (" << max_overshoot << " , " << overshoot << ")" << std::endl;
    
    EXPECT_FALSE(ratio >= tol_settling_error) << "PID did not meet the required final performance: " << "(ref , out) = (" << 1 << " , " << plant.getOutput()[0]/ref[0] << ")" << std::endl;

    double diff_model = (data.col(1) - data.col(2)).array().abs().maxCoeff();
    EXPECT_FALSE( diff_model > tol_diff_model ) << "PID closed-loop step-response differs from its nominal value by: " << diff_model << " > " << tol_diff_model << std::endl;
}

void testFrequencyResponse(Controller &controller, const SettingsPIDSecondOrder &settings, double sampling, double damp, double cutoff, double farpole)
{
    double wn = linear_system::cutoff2resonant(cutoff, damp);
    double dt = sampling;

    auto plant = getDoubleIntegrator(sampling);

    Eigen::VectorXd num_filter(3), den_filter(3);
    num_filter << 0, pow(settings.getNaturalFrequency(), 2), settings.getKi();
    den_filter << settings.getKd(), settings.getKp(), settings.getKi();
    linear_system::LinearSystem prefilter(num_filter, den_filter, sampling);

    Eigen::VectorXd num_model(4), den_model(4);
    num_model << 0, farpole + 2*damp*wn, wn*wn + farpole*2*damp*wn, farpole*wn*wn;
    den_model << 1, farpole + 2*damp*wn, wn*wn + farpole*2*damp*wn, farpole*wn*wn;
    linear_system::LinearSystem model_closedloop(num_model, den_model, sampling);

    Eigen::MatrixXd init_out_model(1, 3), init_out_filter(1, 2);
    init_out_model << 0, 0, 0;
    init_out_filter << 0, 0;

    Eigen::MatrixXd init_in_model(1, 3), init_in_filter(1, 2);
    init_in_model.setZero();
    init_in_filter.setZero();

    Input ref(1);

    double input_freq_array[] = {cutoff / 10, cutoff, wn};
    for (double input_freq : input_freq_array)
    {
        double T = 2*M_PI / input_freq;
        double t_final = 10 * T;
        double t_transient = t_final - 2*T;
        double time = 0;
        plant.setInitialConditions(
            Eigen::MatrixXd::Zero(1, 2),
            Eigen::MatrixXd::Zero(1, 2)
        );
        prefilter.setInitialConditions(
            init_in_filter,
            init_out_filter
        );
        model_closedloop.setInitialConditions(
            init_in_model,
            init_out_model
        );
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
        EXPECT_FALSE(max_error / amplitude > 0.1) << "PID did not meet the required frequency response" << std::endl;
    }
}

TEST(TestController,test_channels)
{
    std::cout << "[TEST] same input to multiple channels" << std::endl;
    //
    PID pid(2);

    SettingsPID settings(1, 0.2, 2);

    pid.configure({settings, settings}, 0.2);
    //
    testReset(pid);
}

TEST(TestController, test_reset)
{
    std::cout << "[TEST] reset" << std::endl;
    //
    unsigned int N = 20;
    PID pid(N);

    SettingsPID settings(1, 0.2, 2);

    std::vector<SettingsPID> settings_all(N);
    for (auto &s : settings_all)
        s = settings;

    pid.configure(settings_all, 0.2);
    //
    testInputChannels(pid);
}

TEST(TestController, test_step_response)
{
    std::cout << "[TEST] step response" << std::endl;
    //
    PID pid(1);
    std::vector<double> ts_array        = {0.1, 0.5, 1, 5, 10, 50, 100, 500, 1000, 5000, 10000};
    std::vector<double> overshoot_array = {0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5};
    double tol_ratio_overshoot = 1.1;
    double tol_settling_error = 0.03;
    double tol_diff_model = 0.1;
    float total = ts_array.size() * overshoot_array.size();
    float current = 0;
    printProgress(0);
    for (double over : overshoot_array)
    {
        for (double ts : ts_array)
        {
            auto settings = SettingsPIDSecondOrder::createFromSpecT(over, ts);
            double sampling = settings.getSuggestedSampling();
            pid.configure(settings, sampling);
            //
            testStepResponse(pid, settings, sampling, ts, tol_ratio_overshoot * over, tol_settling_error, tol_diff_model);
            printProgress(++current/total);
        }
    }
}

TEST(TestController, test_frequency_response)
{
    std::cout << "[TEST] frequency response" << std::endl;
    //
    PID pid(1);
    std::vector<double> cutoff_array = {0.01, 0.05, 0.1, 0.5, 1, 5, 10, 50, 100, 500, 1000};
    std::vector<double> damp_array   = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    float total = cutoff_array.size() * damp_array.size();
    float current = 0;
    printProgress(0);
    for (double cutoff : cutoff_array)
    {
        for (double damp : damp_array)
        {
            auto settings = SettingsPIDSecondOrder::createFromSpecF(damp, cutoff, 10);
            double wn = cutoff / damp;
            double sampling = (2*M_PI/wn) / 1000;
            auto velocity = SettingsFilter::createSecondOrder(0.7, wn * 200);
            velocity.sampling_period = sampling;
            pid.configure(settings, velocity);
            //
            testFrequencyResponse(pid, settings, sampling, damp, cutoff, settings.getFarPole());
            printProgress(++current/total);
        }
    }
}

TEST(TestController, test_saturation_cb)
{
    std::cout << "[TEST] saturation callback" << std::endl;
    //
    PID pid(1);
    auto settings = SettingsPIDSecondOrder::createFromSpecT(0.05, 1);
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
    ASSERT_TRUE( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    //
    int k = 0;
    pid.setCallbackSaturation(
        [&k] (Output &out) -> void {
            out.setConstant(123);
            k = 321;
        }
    );
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(10*sampling), ref, in);
    ASSERT_TRUE(k == 321);
    ASSERT_TRUE( different_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    ASSERT_TRUE( (pid.getOutput().array() == 123).all() );
}

TEST(TestController, test_postprocessing_cb)
{
    std::cout << "[TEST] post-processing callback" << std::endl;
    //
    PID pid(1);
    SettingsPID settings(1, 0, 0);
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
    ASSERT_TRUE( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    //
    int k = 0;
    pid.setCallbackPostProcessing(
        [&k] (Output &out) -> void {
            out.setConstant(123);
            k = 321;
        }
    );
    pid.update(linear_system::LinearSystem::getTimeFromSeconds(10*sampling), ref, in);
    ASSERT_TRUE(k == 321);
    ASSERT_TRUE( equal_eigen(pid.getOutput(), pid.getOutputPreSat()) );
    ASSERT_TRUE( (pid.getOutput().array() == 123).all() );
}

TEST(TestController, test_derivative_mode)
{
    std::cout << "[TEST] derivative mode" << std::endl;
    //
    PID pid(1);
    double sampling = 0.01;
    pid.configure( SettingsPIDSecondOrder::createFromSpecT(0.02, 1.0), sampling );
    Input in(1), ref(1);
    in << 0;
    ref << 0;
    for (unsigned int k = 0; k < 10; ++k)
    {
        in[0] = k;
        pid.update(linear_system::LinearSystem::getTimeFromSeconds(k*sampling), in, ref);
    }
    auto deriv1 = pid.getErrorDerivative();
    Eigen::VectorXd deriv2(1);
    deriv2 = deriv1.array() + 1;
    pid.setDerivativeFiltered(false);
    pid.setErrorDerivative(deriv2);
    auto deriv3 = pid.getErrorDerivative();
    pid.setDerivativeFiltered(true);
    auto deriv4 = pid.getErrorDerivative();

    ASSERT_TRUE( equal_eigen(deriv1, deriv4) );
    ASSERT_TRUE( equal_eigen(deriv2, deriv3) );
    ASSERT_TRUE( different_eigen(deriv1, deriv2) );
    ASSERT_TRUE( different_eigen(deriv3, deriv4) );
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}