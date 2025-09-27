#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "../AdaptivePID.h"

namespace {

struct TimeValue {
    double time;
    double value;
};

struct SimConfig {
    double K{1.8};
    double tau{1.2};
    double L{0.35};
    double Ts{0.05};
    double duration{40.0};
    size_t steps{0};
    double imc_lambda{0.0}; // 0 -> auto based on defaults
    double noise_std{0.01};
    unsigned int seed{42u};
    double init_Kp{0.5};
    double init_Ti{1.0};
    double init_Td{0.0};
    double u_min{-5.0};
    double u_max{5.0};
    bool excitation_enabled{true};
    double excitation_duration{4.0};
    double excitation_amplitude{1.5};
    double excitation_period{0.5};
    std::vector<TimeValue> setpoints{{0.0, 0.0}, {1.0, 1.0}, {15.0, 0.5}, {25.0, 1.2}};
    std::vector<TimeValue> disturbances{{0.0, 0.0}, {18.0, 0.2}}; // additive output load
    std::string csv_path{};
    size_t delay_max_override{0};
};

struct DelayLine {
    DelayLine(size_t integer_delay, double fractional, double fill_value = 0.0)
    : buffer_(std::max<size_t>(integer_delay + 3, 4), fill_value),
      head_(0),
      d_int_(integer_delay),
      frac_(fractional)
    {}

    double push(double u) {
        head_ = (head_ + 1) % buffer_.size();
        buffer_[head_] = u;
        size_t idx_int = dec(head_, d_int_);
        size_t idx_prev = dec(idx_int, 1);
        double delayed = buffer_[idx_int];
        if (frac_ > 1e-9) {
            delayed = (1.0 - frac_) * buffer_[idx_int] + frac_ * buffer_[idx_prev];
        }
        return delayed;
    }

private:
    size_t dec(size_t idx, size_t delta) const {
        size_t N = buffer_.size();
        return (idx + N - (delta % N)) % N;
    }

    std::vector<double> buffer_;
    size_t head_;
    size_t d_int_;
    double frac_;
};

class FOPDTPlant {
public:
    FOPDTPlant(double K, double tau, double L, double Ts)
    : K_(K),
      tau_(tau),
      L_(L),
      Ts_(Ts),
      delay_(makeDelayLine(L, Ts))
    {
        if (tau_ <= 0.0) throw std::invalid_argument("tau must be positive");
        a_ = std::exp(-Ts_ / tau_);
        b_ = K_ * (1.0 - a_);
        state_ = 0.0;
    }

    double step(double u, double disturbance) {
        const double u_delayed = delay_.push(u);
        state_ = a_ * state_ + b_ * u_delayed;
        return state_ + disturbance;
    }

    double gain() const { return K_; }
    double timeConstant() const { return tau_; }
    double deadTime() const { return L_; }

private:
    static DelayLine makeDelayLine(double L, double Ts) {
        if (Ts <= 0.0) throw std::invalid_argument("Ts must be positive");
        const double d_samples = (Ts > 0.0 ? L / Ts : 0.0);
        const size_t d_int = static_cast<size_t>(std::max(0.0, std::floor(d_samples)));
        const double frac = std::clamp(d_samples - static_cast<double>(d_int), 0.0, 0.999);
        return DelayLine(d_int, frac, 0.0);
    }

    double K_;
    double tau_;
    double L_;
    double Ts_;
    DelayLine delay_;
    double a_{0.0};
    double b_{0.0};
    double state_{0.0};
};

struct SampleRow {
    size_t iteration;
    double t;
    double r;
    double y_true;
    double y_meas;
    double u;
    double u_ff;
    double disturbance;
    ctrl::AdaptivePID::FOPDT model;
    ctrl::AdaptivePID::PIDGains gains;
    size_t delay_samples;
    double fit_percent;
    double fit_rmse;
};

bool parseTimeValue(const std::string& token, TimeValue& tv) {
    const auto pos = token.find(':');
    if (pos == std::string::npos) return false;
    try {
        tv.time = std::stod(token.substr(0, pos));
        tv.value = std::stod(token.substr(pos + 1));
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

void printUsage(const char* exe) {
    std::cout << "Usage: " << exe << " [options]\n"
              << "Options:\n"
              << "  --K <value>              Process gain (default 1.8)\n"
              << "  --tau <value>            Time constant in seconds (default 1.2)\n"
              << "  --delay <value>          Dead time in seconds (default 0.35)\n"
              << "  --Ts <value>             Sample time in seconds (default 0.05)\n"
              << "  --duration <value>       Simulation length in seconds (default 40)\n"
              << "  --steps <value>          Override simulation length with a fixed iteration count\n"
              << "  --imc-lambda <value>     Target closed-loop constant (optional)\n"
              << "  --noise-std <value>      Measurement noise std dev (default 0.01)\n"
              << "  --seed <value>           RNG seed (default 42)\n"
              << "  --init-Kp <value>        Gain proportionnel initial (défaut 0.5)\n"
              << "  --init-Ti <value>        Temps intégral initial en s (défaut 1.0)\n"
              << "  --init-Td <value>        Temps dérivatif initial en s (défaut 0.0)\n"
              << "  --umin <value>           Saturation sortie min (défaut -5)\n"
              << "  --umax <value>           Saturation sortie max (défaut +5)\n"
              << "  --excitation-duration <value>  Durée excitation PRBS initiale (s, défaut 4)\n"
              << "  --excitation-amplitude <value> Amplitude excitation (défaut 1.5)\n"
              << "  --excitation-period <value>    Période de mise à jour PRBS (s, défaut 0.5)\n"
              << "  --no-excitation         Désactive le signal d'identification initial\n"
              << "  --sp <t:value>           Setpoint change (can repeat, t in seconds)\n"
              << "  --dist <t:value>         Output disturbance step (can repeat)\n"
              << "  --csv <file>             Save detailed log to CSV\n"
              << "  --delay-max <samples>    Override controller delay_max option\n"
              << "  --help                   Show this message\n";
}

bool parseArgs(int argc, char** argv, SimConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto requireValue = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + name);
            }
            return argv[++i];
        };

        try {
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return false;
            } else if (arg == "--K") {
                cfg.K = std::stod(requireValue(arg));
            } else if (arg == "--tau") {
                cfg.tau = std::stod(requireValue(arg));
            } else if (arg == "--delay") {
                cfg.L = std::stod(requireValue(arg));
            } else if (arg == "--Ts") {
                cfg.Ts = std::stod(requireValue(arg));
            } else if (arg == "--duration") {
                cfg.duration = std::stod(requireValue(arg));
            } else if (arg == "--steps") {
                cfg.steps = static_cast<size_t>(std::stoull(requireValue(arg)));
            } else if (arg == "--imc-lambda") {
                cfg.imc_lambda = std::stod(requireValue(arg));
            } else if (arg == "--noise-std") {
                cfg.noise_std = std::stod(requireValue(arg));
            } else if (arg == "--seed") {
                cfg.seed = static_cast<unsigned int>(std::stoul(requireValue(arg)));
            } else if (arg == "--init-Kp") {
                cfg.init_Kp = std::stod(requireValue(arg));
            } else if (arg == "--init-Ti") {
                cfg.init_Ti = std::stod(requireValue(arg));
            } else if (arg == "--init-Td") {
                cfg.init_Td = std::stod(requireValue(arg));
            } else if (arg == "--umin") {
                cfg.u_min = std::stod(requireValue(arg));
            } else if (arg == "--umax") {
                cfg.u_max = std::stod(requireValue(arg));
            } else if (arg == "--excitation-duration") {
                cfg.excitation_duration = std::stod(requireValue(arg));
            } else if (arg == "--excitation-amplitude") {
                cfg.excitation_amplitude = std::stod(requireValue(arg));
            } else if (arg == "--excitation-period") {
                cfg.excitation_period = std::stod(requireValue(arg));
            } else if (arg == "--no-excitation") {
                cfg.excitation_enabled = false;
            } else if (arg == "--sp") {
                TimeValue tv{};
                if (!parseTimeValue(requireValue(arg), tv)) {
                    throw std::runtime_error("Invalid --sp argument; expected t:value");
                }
                cfg.setpoints.push_back(tv);
            } else if (arg == "--dist") {
                TimeValue tv{};
                if (!parseTimeValue(requireValue(arg), tv)) {
                    throw std::runtime_error("Invalid --dist argument; expected t:value");
                }
                cfg.disturbances.push_back(tv);
            } else if (arg == "--csv") {
                cfg.csv_path = requireValue(arg);
            } else if (arg == "--delay-max") {
                cfg.delay_max_override = static_cast<size_t>(std::stoul(requireValue(arg)));
            } else {
                throw std::runtime_error("Unknown option: " + arg);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    auto sortByTime = [](std::vector<TimeValue>& vec) {
        std::sort(vec.begin(), vec.end(), [](const TimeValue& a, const TimeValue& b) {
            return a.time < b.time;
        });
    };
    sortByTime(cfg.setpoints);
    sortByTime(cfg.disturbances);

    if (cfg.setpoints.empty() || cfg.setpoints.front().time > 0.0) {
        cfg.setpoints.insert(cfg.setpoints.begin(), TimeValue{0.0, cfg.setpoints.empty() ? 0.0 : cfg.setpoints.front().value});
    }
    if (cfg.disturbances.empty() || cfg.disturbances.front().time > 0.0) {
        cfg.disturbances.insert(cfg.disturbances.begin(), TimeValue{0.0, 0.0});
    }
    if (cfg.steps > 0) {
        cfg.duration = static_cast<double>(cfg.steps) * cfg.Ts;
    }
    return true;
}

struct Metrics {
    double rmse{0.0};
    double mae{0.0};
    double iae{0.0};
    double max_overshoot{0.0};
};

Metrics computeMetrics(const std::vector<SampleRow>& rows, double Ts) {
    Metrics m{};
    if (rows.empty()) return m;
    double sum_sq = 0.0;
    double sum_abs = 0.0;
    double sum_int_abs = 0.0;
    double max_overshoot = 0.0;
    for (const auto& row : rows) {
        const double err = row.r - row.y_true;
        sum_sq += err * err;
        sum_abs += std::fabs(err);
        sum_int_abs += std::fabs(err) * Ts;
        const double overshoot = row.y_true - row.r;
        if (overshoot > max_overshoot) {
            max_overshoot = overshoot;
        }
    }
    m.rmse = std::sqrt(sum_sq / rows.size());
    m.mae = sum_abs / rows.size();
    m.iae = sum_int_abs;
    m.max_overshoot = max_overshoot;
    return m;
}

void saveCsv(const std::vector<SampleRow>& rows, const std::string& path) {
    std::ofstream ofs(path);
    if (!ofs) {
        throw std::runtime_error("Unable to open CSV file: " + path);
    }
    ofs << "iteration,time,setpoint,y_true,y_meas,u,u_ff,disturbance,model_valid,est_K,est_tau,est_L,delay_samples,fit_percent,fit_rmse,Kp,Ti,Td";
    ofs << "\n";
    ofs << std::setprecision(9);
    for (const auto& row : rows) {
        ofs << row.iteration << ',' << row.t << ',' << row.r << ',' << row.y_true << ',' << row.y_meas << ','
            << row.u << ',' << row.u_ff << ',' << row.disturbance << ',' << (row.model.valid ? 1 : 0) << ','
            << row.model.K << ',' << row.model.tau << ',' << row.model.L << ','
            << row.delay_samples << ',' << row.fit_percent << ',' << row.fit_rmse << ','
            << row.gains.Kp << ',' << row.gains.Ti << ',' << row.gains.Td << "\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    SimConfig cfg;
    if (!parseArgs(argc, argv, cfg)) {
        return 0;
    }

    size_t steps = cfg.steps;
    if (steps == 0) {
        steps = static_cast<size_t>(std::ceil(cfg.duration / cfg.Ts));
    }
    FOPDTPlant plant(cfg.K, cfg.tau, cfg.L, cfg.Ts);

    ctrl::AdaptivePID pid(cfg.Ts);
    auto opts = pid.options();
    if (cfg.imc_lambda > 0.0) {
        opts.imc_lambda = cfg.imc_lambda;
    }
    const double delay_margin = std::max(0.5, 5.0 * cfg.Ts);
    const size_t delay_max_auto = static_cast<size_t>(std::ceil((cfg.L + delay_margin) / cfg.Ts));
    if (cfg.delay_max_override > 0) {
        opts.delay_max = cfg.delay_max_override;
    } else {
        opts.delay_max = std::max(delay_max_auto, opts.delay_min + size_t(1));
    }
    pid.setOptions(opts);
    pid.setManualGains(cfg.init_Kp, cfg.init_Ti, cfg.init_Td);
    pid.setOutputLimits(cfg.u_min, cfg.u_max);

    std::mt19937 rng(cfg.seed);
    std::normal_distribution<double> noise(0.0, cfg.noise_std);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    std::vector<SampleRow> log;
    log.reserve(steps + 1);

    double measured_y = 0.0;
    double true_y = 0.0;
    double disturbance = cfg.disturbances.front().value;
    size_t sp_idx = 0;
    size_t dist_idx = 0;
    double setpoint = cfg.setpoints.front().value;

    std::cout << std::fixed << std::setprecision(4);

    double excitation_signal = 0.0;
    double next_excitation_update = 0.0;

    for (size_t k = 0; k < steps; ++k) {
        const double t = k * cfg.Ts;
        while (sp_idx + 1 < cfg.setpoints.size() && t >= cfg.setpoints[sp_idx + 1].time - 1e-12) {
            ++sp_idx;
            setpoint = cfg.setpoints[sp_idx].value;
        }
        while (dist_idx + 1 < cfg.disturbances.size() && t >= cfg.disturbances[dist_idx + 1].time - 1e-12) {
            ++dist_idx;
            disturbance = cfg.disturbances[dist_idx].value;
        }

        double u_ff = 0.0;
        const bool excitation_active = cfg.excitation_enabled && t < cfg.excitation_duration;
        if (excitation_active) {
            if (cfg.excitation_period <= 1e-6) {
                excitation_signal = cfg.excitation_amplitude;
            } else if (t >= next_excitation_update) {
                const double sign = (uni01(rng) < 0.5 ? -1.0 : 1.0);
                excitation_signal = sign * cfg.excitation_amplitude;
                next_excitation_update = t + cfg.excitation_period;
            }
            u_ff = excitation_signal;
        } else {
            excitation_signal = 0.0;
        }

        const double u = pid.update(setpoint, measured_y, u_ff);
        true_y = plant.step(u, disturbance);
        const double noise_sample = (cfg.noise_std > 0.0 ? noise(rng) : 0.0);
        measured_y = true_y + noise_sample;

        SampleRow row{};
        row.iteration = k;
        row.t = t;
        row.r = setpoint;
        row.y_true = true_y;
        row.y_meas = measured_y;
        row.u = u;
        row.u_ff = u_ff;
        row.disturbance = disturbance;
        row.model = pid.model();
        row.gains = pid.gains();
        row.delay_samples = pid.delaySamples();
        row.fit_percent = pid.fitPercent();
        row.fit_rmse = pid.fitRMSE();
        log.push_back(row);
    }

    const auto metrics = computeMetrics(log, cfg.Ts);
    const auto final_model = log.back().model;
    const auto final_gains = log.back().gains;
    const double estimated_delay = final_model.L;
    const double discrete_delay = static_cast<double>(log.back().delay_samples) * cfg.Ts;

    std::cout << "\n=== Résumé simulation ===\n";
    std::cout << "Procédé réel: K=" << plant.gain() << ", tau=" << plant.timeConstant()
              << " s, L=" << plant.deadTime() << " s\n";
    std::cout << "Estimation finale: K=" << final_model.K << ", tau=" << final_model.tau
              << " s, L=" << estimated_delay << " s (valid=" << (final_model.valid ? "oui" : "non") << ")\n";
    std::cout << "Délai discret estimé: " << log.back().delay_samples << " échantillons -> "
              << discrete_delay << " s\n";
    std::cout << "Erreur de latence: " << (estimated_delay - plant.deadTime()) << " s\n";
    std::cout << "Gains PID: Kp=" << final_gains.Kp << ", Ti=" << final_gains.Ti
              << " s, Td=" << final_gains.Td << " s\n";
    std::cout << "Qualité (controller): FIT=" << log.back().fit_percent
              << " %, RMSE=" << log.back().fit_rmse << "\n";
    std::cout << "Erreurs boucle fermée (vrai procédé): RMSE=" << metrics.rmse
              << ", MAE=" << metrics.mae << ", IAE=" << metrics.iae
              << " s, overshoot max=" << metrics.max_overshoot << "\n";

    std::cout << "\nRapport JSON:\n" << pid.reportJSON() << "\n";

    if (!cfg.csv_path.empty()) {
        try {
            saveCsv(log, cfg.csv_path);
            std::cout << "\nLog détaillé sauvegardé dans " << cfg.csv_path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Impossible d'écrire le CSV: " << e.what() << "\n";
        }
    }

    return 0;
}

