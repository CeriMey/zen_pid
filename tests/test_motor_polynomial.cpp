#include "../AdaptivePID.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

class InertialMotor {
public:
    InertialMotor(double natural_freq, double damping, double gain, double Ts)
    : wn_(natural_freq),
      zeta_(damping),
      K_(gain),
      Ts_(Ts),
      wn2_(natural_freq * natural_freq)
    {}

    double step(double control, double load_accel) {
        // Second-order inertial system: theta'' + 2*zeta*wn*theta' + wn^2*theta = K*wn^2*u + load
        const double accel = K_ * wn2_ * control - 2.0 * zeta_ * wn_ * omega_ - wn2_ * theta_ + load_accel;
        omega_ += accel * Ts_;
        theta_ += omega_ * Ts_;
        return theta_;
    }

    double velocity() const { return omega_; }
    double position() const { return theta_; }

private:
    double wn_;
    double zeta_;
    double K_;
    double Ts_;
    double wn2_;
    double omega_{0.0};
    double theta_{0.0};
};

struct Sample {
    double t;
    double r;
    double y;
    double y_meas;
    double u;
    double excitation;
    double load;
    ctrl::AdaptivePID::FOPDT model;
    ctrl::AdaptivePID::PIDGains gains;
};

} // namespace

int main() {
    using ctrl::AdaptivePID;

    const double Ts = 0.02;         // 50 Hz sampling
    const double duration = 40.0;   // seconds
    const std::size_t steps = static_cast<std::size_t>(duration / Ts);

    InertialMotor motor(/*wn=*/3.5, /*zeta=*/0.6, /*gain=*/1.1, Ts);

    AdaptivePID pid(Ts);
    auto opts = pid.options();
    opts.delay_min = 0;
    opts.delay_max = 3;
    opts.delay_smooth = 3;
    opts.corr_window = 120;
    opts.imc_lambda = 0.35;
    opts.imc_lambda_min = 0.08;
    opts.imc_lambda_relL = 0.6;
    opts.out_min = -1.5;
    opts.out_max = 1.5;
    opts.fast_adapt_gain = 0.08;
    pid.setOptions(opts);
    pid.setSetpointWeight(0.7);
    pid.setDerivativeFilterN(12.0);

    std::default_random_engine rng(1337);
    std::normal_distribution<double> noise(0.0, 0.0015);

    std::vector<Sample> log;
    log.reserve(steps);

    std::ofstream csv("tests/data/motor_polynomial_run.csv");
    csv << "t,setpoint,y_true,y_meas,u,excitation,load,K_est,tau_est,L_est,model_valid,fit_percent,fit_rmse,Kp,Ti,Td" << '\n';
    csv << std::fixed << std::setprecision(6);

    double measured_y = 0.0;
    double true_y = 0.0;
    double load_accel = 0.0;
    double sq_err = 0.0;
    double abs_err_tail = 0.0;
    double max_abs_err = 0.0;
    const std::size_t tail_samples = static_cast<std::size_t>(5.0 / Ts);

    for (std::size_t k = 0; k < steps; ++k) {
        const double t = k * Ts;
        // Polynomial setpoint (clamped to a realistic angle)
        double r = 0.015 * t + 0.0006 * t * t - 0.000008 * t * t * t;
        r = std::clamp(r, 0.0, 0.6);

        // Identification burst during the first 3 seconds
        double excitation = 0.0;
        if (t < 3.0) {
            excitation = (((k / 10) % 2) == 0) ? 0.35 : -0.35;
        }

        // Introduce a small load torque step after 18 s
        if (t >= 18.0 && t < 24.0) {
            load_accel = 0.1;
        } else if (t >= 24.0) {
            load_accel = 0.0;
        }

        const double u = pid.update(r, measured_y, excitation);
        true_y = motor.step(u, load_accel);
        const double y_meas = true_y + noise(rng);
        measured_y = y_meas;

        const double err = r - true_y;
        sq_err += err * err;
        if (steps - k <= tail_samples) {
            abs_err_tail += std::fabs(err);
        }
        max_abs_err = std::max(max_abs_err, std::fabs(err));

        const auto model = pid.model();
        const auto gains = pid.gains();

        log.push_back(Sample{t, r, true_y, y_meas, u, excitation, load_accel, model, gains});

        csv << t << ',' << r << ',' << true_y << ',' << y_meas << ',' << u << ','
            << excitation << ',' << load_accel << ',' << model.K << ',' << model.tau << ','
            << model.L << ',' << (model.valid ? 1 : 0) << ',' << pid.fitPercent() << ','
            << pid.fitRMSE() << ',' << gains.Kp << ',' << gains.Ti << ',' << gains.Td << '\n';
    }

    csv.close();

    const double rmse = std::sqrt(sq_err / static_cast<double>(steps));
    const double steady_err = abs_err_tail / static_cast<double>(tail_samples);
    const auto final_model = log.back().model;
    const auto final_gains = log.back().gains;

    std::cout << "Motor polynomial tracking RMSE: " << rmse << '\n';
    std::cout << "Steady-state |e| avg (last 5 s): " << steady_err << '\n';
    std::cout << "Peak |e|: " << max_abs_err << '\n';
    std::cout << "Final model: K=" << final_model.K << ", tau=" << final_model.tau
              << ", L=" << final_model.L << ", valid=" << (final_model.valid ? "yes" : "no") << '\n';
    std::cout << "Final gains: Kp=" << final_gains.Kp << ", Ti=" << final_gains.Ti
              << ", Td=" << final_gains.Td << '\n';

    if (!final_model.valid) {
        std::cerr << "Model never validated" << std::endl;
        return 1;
    }
    if (!(final_model.K > 0.05 && final_model.tau > 0.1)) {
        std::cerr << "Identified model parameters unrealistic" << std::endl;
        return 2;
    }
    if (rmse > 0.08) {
        std::cerr << "RMSE too high" << std::endl;
        return 3;
    }
    if (steady_err > 0.05) {
        std::cerr << "Steady-state error too high" << std::endl;
        return 4;
    }
    if (max_abs_err > 0.25) {
        std::cerr << "Transient error unacceptable" << std::endl;
        return 5;
    }
    if (pid.fitPercent() < 55.0) {
        std::cerr << "Model fit quality insufficient" << std::endl;
        return 6;
    }

    std::cout << "Motor polynomial trajectory test passed" << std::endl;
    return 0;
}
