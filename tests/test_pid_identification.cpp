#include "../AdaptivePID.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <iostream>

int main() {
    constexpr double Ts = 0.05;
    constexpr double K = 1.8;
    constexpr double tau = 1.2;
    constexpr double L = 0.20;

    ctrl::AdaptivePID pid(Ts);
    auto opts = pid.options();
    const std::size_t delay_samples = static_cast<std::size_t>(std::lround(L / Ts));
    opts.delay_min = delay_samples;
    opts.delay_max = delay_samples;
    opts.imc_lambda = 0.5;
    pid.setOptions(opts);

    auto snap = pid.captureCalibration();
    const double a = std::exp(-Ts / tau);
    const double b = K * (1.0 - a);
    snap.theta[0] = a;
    snap.theta[1] = b;
    snap.P[0][0] = snap.P[1][1] = 1.0;
    snap.P[0][1] = snap.P[1][0] = 0.0;
    snap.model = {};
    snap.model.a = a;
    snap.model.b = b;
    snap.model.var_eps = 1e-6;
    snap.model.valid = false;
    snap.delay_samples = delay_samples;
    snap.delay_fraction = 0.0;
    snap.gains = {};
    snap.gains.beta = 1.0;
    snap.gains.N = 10.0;
    assert(pid.applyCalibration(snap));

    // First update to trigger model & tuning refresh
    pid.update(0.0, 0.0);

    auto model = pid.model();
    assert(model.valid);
    assert(std::fabs(model.K - K) < 1e-9);
    assert(std::fabs(model.tau - tau) < 1e-9);
    assert(std::fabs(model.L - L) < 1e-9);
    assert(pid.delaySamples() == delay_samples);

    const double expected_lambda = std::max(opts.imc_lambda,
        std::max(opts.imc_lambda_min, opts.imc_lambda_relL * L));
    const double expected_Kp = (tau + 0.5 * L) / (K * (expected_lambda + 0.5 * L));
    const double expected_Ti = std::min(tau + 0.5 * L, 4.0 * (expected_lambda + 0.5 * L));
    const double expected_Td = (2.0 * tau + L > 1e-9) ? (tau * L) / (2.0 * tau + L) : 0.0;

    auto gains = pid.gains();
    const double tol = 1e-9;
    assert(std::fabs(gains.Kp - expected_Kp) < tol);
    assert(std::fabs(gains.Ti - expected_Ti) < tol);
    assert(std::fabs(gains.Td - expected_Td) < tol);

    const double control = pid.update(1.0, 0.0);
    const double expected_control = gains.Kp * gains.beta + (gains.Kp / gains.Ti) * Ts;
    assert(std::fabs(control - expected_control) < 1e-9);

    // Capture the tuned calibration state and recreate the controller to generate a
    // clean closed-loop simulation trace.
    snap = pid.captureCalibration();
    ctrl::AdaptivePID sim_pid(Ts);
    sim_pid.setOptions(opts);
    assert(sim_pid.applyCalibration(snap));

    constexpr std::size_t num_steps = 200;
    const double setpoint = 1.0;
    double measurement = 0.0;
    double process_state = 0.0;
    std::deque<double> delay(delay_samples + 1, 0.0);

    std::ofstream csv("tests/pid_identification_response.csv");
    csv << "time,setpoint,measurement,control,delayed_control,next_measurement" << '\n';

    for (std::size_t k = 0; k < num_steps; ++k) {
        const double t = static_cast<double>(k) * Ts;
        const double ctrl_signal = sim_pid.update(setpoint, measurement);
        delay.push_back(ctrl_signal);
        const double delayed_ctrl = delay.front();
        delay.pop_front();

        const double next_process_state = a * process_state + b * delayed_ctrl;

        csv << t << ',' << setpoint << ',' << measurement << ',' << ctrl_signal << ','
            << delayed_ctrl << ',' << next_process_state << '\n';

        measurement = next_process_state;
        process_state = next_process_state;
    }

    std::cout << "AdaptivePID parameter mapping test passed" << std::endl;
    return 0;
}
