#include "../AdaptivePID.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

int main() {
    using ctrl::AdaptivePID;
    AdaptivePID pid(0.1);
    auto opts = pid.options();
    opts.delay_max = 8;
    opts.delay_smooth = 3;
    opts.corr_window = 50;
    opts.imc_lambda = 0.8;
    pid.setOptions(opts);

    // excite controller to populate internal state
    double u = 0.0;
    for (int k = 0; k < 200; ++k) {
        double r = (k % 50 < 25) ? 1.0 : 0.2;
        double y = 0.5 * std::sin(0.05 * k) + 0.2 * u;
        u = pid.update(r, y);
    }

    auto snapshot = pid.captureCalibration();
    const char* path = "tests/data/tmp_calibration.tbl";
    if (!pid.saveCalibration(path)) {
        std::cerr << "Unable to save calibration" << std::endl;
        return 1;
    }

    AdaptivePID pid_loaded(0.1);
    pid_loaded.setOptions(opts);
    if (!pid_loaded.loadCalibration(path)) {
        std::cerr << "Unable to load calibration" << std::endl;
        return 2;
    }

    auto g_ref = pid.gains();
    auto g_loaded = pid_loaded.gains();
    assert(std::fabs(g_ref.Kp - g_loaded.Kp) < 1e-9);
    assert(std::fabs(g_ref.Ti - g_loaded.Ti) < 1e-9);
    assert(std::fabs(g_ref.Td - g_loaded.Td) < 1e-9);

    auto m_ref = pid.model();
    auto m_loaded = pid_loaded.model();
    assert(std::fabs(m_ref.K - m_loaded.K) < 1e-9);
    assert(std::fabs(m_ref.tau - m_loaded.tau) < 1e-9);
    assert(std::fabs(m_ref.L - m_loaded.L) < 1e-9);
    assert(m_ref.valid == m_loaded.valid);

    assert(pid.delaySamples() == pid_loaded.delaySamples());

    // ensure dynamic state is identical when applying the snapshot
    pid.applyCalibration(snapshot);
    double test_r = 0.6;
    double test_y = 0.45;
    double u_ref = pid.update(test_r, test_y);
    double u_loaded = pid_loaded.update(test_r, test_y);
    assert(std::fabs(u_ref - u_loaded) < 1e-9);

    std::remove(path);
    std::cout << "Calibration persistence test passed" << std::endl;
    return 0;
}
