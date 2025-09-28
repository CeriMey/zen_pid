#include "../AdaptivePID.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

struct FirstOrderPlant {
    double a{0.0};
    double b{0.0};
    double state{0.0};

    double update(double u) {
        state = a * state + b * u;
        return state;
    }
};

struct LowPassFilter {
    double alpha{1.0};
    double state{0.0};
    bool initialised{false};

    explicit LowPassFilter(double time_constant, double Ts) {
        const double tau = std::max(time_constant, 1e-6);
        alpha = Ts / (tau + Ts);
    }

    double filter(double input) {
        if (!initialised) {
            state = input;
            initialised = true;
            return state;
        }
        state += alpha * (input - state);
        return state;
    }
};

bool loadCalibrationIfPresent(ctrl::AdaptivePID& pid, const std::filesystem::path& file) {
    if (std::filesystem::exists(file)) {
        if (pid.loadCalibration(file.string())) {
            std::cout << "Calibration chargée depuis " << file << '\n';
            return true;
        }
        std::cout << "Impossible de charger la calibration " << file
                  << ", utilisation des paramètres par défaut.\n";
        return false;
    }
    std::cout << "Aucune calibration précédente pour " << file.filename() << ".\n";
    return false;
}

void saveCalibration(const ctrl::AdaptivePID& pid, const std::filesystem::path& file) {
    if (pid.saveCalibration(file.string())) {
        std::cout << "Calibration enregistrée dans " << file << '\n';
    } else {
        std::cout << "Impossible d'enregistrer la calibration dans " << file << '\n';
    }
}

void ensureInitialGains(ctrl::AdaptivePID& pid, bool loaded, double Kp, double Ti, double Td) {
    if (!loaded) {
        pid.setManualGains(Kp, Ti, Td);
        pid.enableAdaptation(true);
        return;
    }
    auto snap = pid.captureCalibration();
    if (snap.gains.Kp < 1e-6) {
        std::cout << "Calibration chargée mais gains nuls, utilisation d'un préréglage manuel.\n";
        pid.setManualGains(Kp, Ti, Td);
        pid.enableAdaptation(true);
    }
}

int main() {
    constexpr double Ts = 0.01;  // 10 ms
    ctrl::AdaptivePID pid_x(Ts);
    ctrl::AdaptivePID pid_y(Ts);

    // Limites de sortie réalistes (par ex. effort moteur)
    pid_x.setOutputLimits(-3.0, 3.0);
    pid_y.setOutputLimits(-3.5, 3.5);

    // Lissage de la consigne via un filtre passe-bas premier ordre
    LowPassFilter sp_filter_x(0.3, Ts); // tau = 0.3 s
    LowPassFilter sp_filter_y(0.4, Ts);

    // Fichiers de sauvegarde des calibrations
    std::filesystem::path data_dir = std::filesystem::path("tests") / "data";
    std::filesystem::create_directories(data_dir);
    const auto cal_x = data_dir / "pid_x.cal";
    const auto cal_y = data_dir / "pid_y.cal";

    // Recharger les paramètres apprises lors des précédents lancements
    const bool loaded_x = loadCalibrationIfPresent(pid_x, cal_x);
    const bool loaded_y = loadCalibrationIfPresent(pid_y, cal_y);

    ensureInitialGains(pid_x, loaded_x, 1.2, 0.8, 0.02);
    ensureInitialGains(pid_y, loaded_y, 1.0, 1.0, 0.05);

    // Deux procédés décorrélés à identifier
    FirstOrderPlant plant_x;
    FirstOrderPlant plant_y;

    const double tau_x = 0.8;
    const double tau_y = 1.2;
    const double gain_x = 1.3;
    const double gain_y = 0.9;

    plant_x.a = std::exp(-Ts / tau_x);
    plant_y.a = std::exp(-Ts / tau_y);
    plant_x.b = gain_x * (1.0 - plant_x.a);
    plant_y.b = gain_y * (1.0 - plant_y.a);

    constexpr int total_samples = 10000;
    const double total_duration = total_samples * Ts;
    const double pi = std::acos(-1.0);

    struct Phase {
        std::string label;
        int samples;
    };

    const std::vector<Phase> phases = {
        {"idle_start", 700},
        {"slow_ramp", 1300},
        {"stop_1", 600},
        {"ultra_fast_square", 1200},
        {"stop_2", 600},
        {"slow_sine", 1500},
        {"stop_3", 600},
        {"chirp", 1600},
        {"stop_4", 700},
        {"random_bursts", 1200},
    };

    int total_phase_samples = 0;
    for (const auto& phase : phases) {
        total_phase_samples += phase.samples;
    }
    if (total_phase_samples != total_samples) {
        std::cerr << "La configuration de phases ne totalise pas " << total_samples
                  << " échantillons (" << total_phase_samples << " calculés).\n";
        return 1;
    }

    std::vector<double> sp_x_raw(total_samples, 0.0);
    std::vector<double> sp_y_raw(total_samples, 0.0);
    std::vector<int> phase_index_lookup(total_samples, 0);

    std::mt19937 setpoint_rng_x(2024);
    std::mt19937 setpoint_rng_y(2025);
    std::uniform_real_distribution<double> random_step_x(-1.2, 1.2);
    std::normal_distribution<double> random_step_y(0.0, 0.9);

    int cursor = 0;
    for (std::size_t phase_idx = 0; phase_idx < phases.size(); ++phase_idx) {
        const auto& phase = phases[phase_idx];
        for (int n = 0; n < phase.samples; ++n, ++cursor) {
            const double t_phase = n * Ts;
            const double progress = static_cast<double>(n) / std::max(phase.samples - 1, 1);
            double value_x = 0.0;
            double value_y = 0.0;

            switch (phase_idx) {
                case 0: // idle_start
                case 2: // stop_1
                case 4: // stop_2
                case 6: // stop_3
                case 8: // stop_4
                    value_x = 0.0;
                    value_y = 0.0;
                    break;
                case 1: // slow_ramp
                    value_x = 1.2 * progress;
                    value_y = -0.8 * progress;
                    break;
                case 3: { // ultra_fast_square
                    const bool high = ((n / 3) % 2) == 0;
                    value_x = high ? 0.9 : -0.9;
                    const bool high_y = ((n / 2) % 2) == 0;
                    value_y = high_y ? 1.1 : -1.1;
                    break;
                }
                case 5: // slow_sine
                    value_x = 0.7 * std::sin(2.0 * pi * 0.15 * t_phase);
                    value_y = 0.9 * std::sin(2.0 * pi * 0.18 * t_phase + pi / 4.0);
                    break;
                case 7: { // chirp
                    const double f0 = 0.2;   // Hz
                    const double f1 = 2.0;   // Hz
                    const double duration = phase.samples * Ts;
                    const double k = (f1 - f0) / std::max(duration, Ts);
                    const double time = n * Ts;
                    const double phase_angle = 2.0 * pi * (f0 * time + 0.5 * k * time * time);
                    value_x = 0.8 * std::sin(phase_angle);
                    value_y = 0.6 * std::sin(phase_angle + pi / 3.0);
                    break;
                }
                case 9: { // random_bursts
                    const int hold_x = 40;
                    const int hold_y = 80;
                    static double target_x = 0.0;
                    static double target_y = 0.0;
                    if (n % hold_x == 0) {
                        target_x = random_step_x(setpoint_rng_x);
                    }
                    if (n % hold_y == 0) {
                        target_y = std::clamp(random_step_y(setpoint_rng_y), -1.5, 1.5);
                    }
                    value_x = target_x;
                    value_y = target_y;
                    break;
                }
                default:
                    value_x = 0.0;
                    value_y = 0.0;
                    break;
            }

            sp_x_raw[cursor] = value_x;
            sp_y_raw[cursor] = value_y;
            phase_index_lookup[cursor] = static_cast<int>(phase_idx);
        }
    }

    std::ofstream csv(data_dir / "dual_axis_log.csv");
    if (!csv) {
        std::cerr << "Impossible d'ouvrir le fichier CSV de sortie." << std::endl;
        return 1;
    }
    csv << "time_s,phase,sp_x_raw,sp_x_filtered,y_x,u_x,sp_y_raw,sp_y_filtered,y_y,u_y\n";

    std::default_random_engine rng_noise{42};
    std::normal_distribution<double> noise(0.0, 0.01);

    double y_x = 0.0;
    double y_y = 0.0;

    int current_phase = -1;
    int phase_start_sample = 0;

    for (int k = 0; k < total_samples; ++k) {
        const double t = k * Ts;
        const int phase_idx = phase_index_lookup[k];

        if (phase_idx != current_phase) {
            phase_start_sample = k;
            current_phase = phase_idx;
            std::cout << "Phase " << phases[phase_idx].label << " démarrée à t=" << t
                      << " s (échantillon " << k << ")" << std::endl;
        }

        const double sp_x = sp_filter_x.filter(sp_x_raw[k]);
        const double sp_y = sp_filter_y.filter(sp_y_raw[k]);

        const double u_x = pid_x.update(sp_x, y_x);
        const double u_y = pid_y.update(sp_y, y_y);

        y_x = plant_x.update(u_x) + noise(rng_noise);
        y_y = plant_y.update(u_y) + noise(rng_noise);

        csv << t << ',' << phases[phase_idx].label << ',' << sp_x_raw[k] << ',' << sp_x << ','
            << y_x << ',' << u_x << ',' << sp_y_raw[k] << ',' << sp_y << ',' << y_y << ','
            << u_y << '\n';

        if (((k - phase_start_sample) % 200) == 0) {
            std::cout << "  t=" << t << " s\tsp_x_f=" << sp_x << "\ty_x=" << y_x
                      << "\tu_x=" << u_x << "\tsp_y_f=" << sp_y << "\ty_y=" << y_y
                      << "\tu_y=" << u_y << '\n';
        }
    }

    saveCalibration(pid_x, cal_x);
    saveCalibration(pid_y, cal_y);

    std::cout << "Simulation terminée après " << total_duration
              << " s. Résultats enregistrés dans " << (data_dir / "dual_axis_log.csv")
              << '\n';

    return 0;
}
