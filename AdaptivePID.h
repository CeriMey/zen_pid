#pragma once
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <fstream>
#include <limits>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>
#include <algorithm>

/**
 * AdaptivePID — PID à coefficients adaptatifs (C++11, header-only)
 * ---------------------------------------------------------------
 * - Identification en ligne d’un modèle FOPDT discret: y_k = a*y_{k-1} + b*u_{k-d}
 * - Estimation de délai (latence) par corrélation croisée u↔y (fenêtre glissante)
 * - RLS à facteur d’oubli adaptatif (apprentissage rapide quand l’erreur augmente)
 * - Recalage PID (IMC/SIMC) à partir de (K, tau, L)
 * - Anti-windup (rétro-calcul), dérivée filtrée (N), pondération de consigne (beta)
 * - Rapport JSON des paramètres, incertitudes, métriques et gains
 *
 * Sécurité / Robustesse:
 * - Limitation sortie, clamp des paramètres, monotonie des filtres
 * - Détection NaN/Inf, bornes de tau et K, gel des gains si modèle invalide
 *
 * Auteur: (vous)
 * Licence: à définir par le propriétaire du code
 */
namespace ctrl {

class AdaptivePID {
public:
    struct PIDGains {
        double Kp{0.0};
        double Ti{1.0};     // s
        double Td{0.0};     // s
        double beta{1.0};   // setpoint weight for P
        double N{10.0};     // derivative filter factor (Td/N time constant)
    };
    struct FOPDT {
        double K{1.0};         // process gain
        double tau{1.0};       // time constant (s)
        double L{0.0};         // dead time (s)
        double a{0.99};        // discrete pole
        double b{0.0};         // discrete input coefficient
        double var_eps{1e-6};  // EWMA of residual variance
        bool   valid{false};
    };
    struct CalibrationSnapshot {
        double Ts{0.0};
        PIDGains gains{};
        FOPDT model{};
        double theta[2]{0.99, 0.0};
        double P[2][2]{{1e3, 0.0}, {0.0, 1e3}};
        double eps_var{1e-6};
        std::size_t delay_samples{0};
        double delay_fraction{0.0};
        double I_state{0.0};
        double y_prev{0.0};
        double y_dot{0.0};
        double last_u{0.0};
        double fit_rmse{0.0};
        double fit_percent{0.0};
        bool adapt_on{true};
    };
    struct Options {
        // Identification
        size_t delay_min{0};
        size_t delay_max{50};        // in samples (L_max = delay_max*Ts)
        size_t corr_window{200};     // samples used for cross-correlation
        size_t delay_smooth{5};      // majority filter on delay estimate
        double lambda_min{0.90};     // RLS forgetting factor bounds
        double lambda_max{0.995};
        double rls_P0{1e3};          // initial covariance magnitude
        double eps_var_alpha{0.01};  // EWMA for residual variance
        double fast_adapt_gain{0.08}; // how strongly residual increases adaptation speed
        // Tuning (IMC/SIMC)
        double imc_lambda{0.5};      // desired closed-loop time constant (s)
        double imc_lambda_min{0.05}; // floor to avoid over-aggressiveness
        double imc_lambda_relL{0.8}; // lambda >= 0.8*L (SIMC guidance)
        // PID execution
        double out_min{-1e9};
        double out_max{+1e9};
        double tracking_Tt_factor{1.0}; // Tt = tracking_Tt_factor * Ti
        double dt_min{1e-6};            // guard
        // Reporting
        size_t fit_window{400}; // for FIT metric
    };

    explicit AdaptivePID(double Ts)
    : Ts_(Ts)
    {
        reset();
        initBuffers();
    }

    // ---- API de configuration ----
    void setOptions(const Options& o) {
        opts_ = o;
        if (opts_.delay_smooth == 0) opts_.delay_smooth = 1;
        if (opts_.delay_min > opts_.delay_max) std::swap(opts_.delay_min, opts_.delay_max);
        initBuffers();
    }
    const Options& options() const { return opts_; }

    void setSampleTime(double Ts) {
        Ts_ = std::max(Ts, 1e-6);
        initBuffers();
    }
    double getSampleTime() const { return Ts_; }

    void setSetpointWeight(double beta) { gains_.beta = clamp(beta, 0.0, 1.0); }
    void setDerivativeFilterN(double N) { gains_.N = std::max(1.0, N); }
    void setOutputLimits(double umin, double umax) {
        opts_.out_min = std::min(umin, umax);
        opts_.out_max = std::max(umin, umax);
    }
    void enableAdaptation(bool on) { adapt_on_ = on; }

    // Calibration persistence helpers
    CalibrationSnapshot captureCalibration() const {
        CalibrationSnapshot snap;
        snap.Ts = Ts_;
        snap.gains = gains_;
        snap.model = model_;
        snap.theta[0] = theta_[0];
        snap.theta[1] = theta_[1];
        snap.P[0][0] = P_[0][0];
        snap.P[0][1] = P_[0][1];
        snap.P[1][0] = P_[1][0];
        snap.P[1][1] = P_[1][1];
        snap.eps_var = eps_var_;
        snap.delay_samples = d_hat_;
        snap.delay_fraction = L_frac_;
        snap.I_state = I_state_;
        snap.y_prev = y_prev_;
        snap.y_dot = y_dot_;
        snap.last_u = last_u_;
        snap.fit_rmse = fit_rmse_;
        snap.fit_percent = fit_percent_;
        snap.adapt_on = adapt_on_;
        return snap;
    }

    bool applyCalibration(const CalibrationSnapshot& snap) {
        if (!(std::isfinite(snap.Ts) && snap.Ts > 0.0)) return false;
        if (!std::isfinite(snap.gains.Kp) || !std::isfinite(snap.gains.Ti) ||
            !std::isfinite(snap.gains.Td) || !std::isfinite(snap.gains.beta) ||
            !std::isfinite(snap.gains.N)) return false;
        if (!std::isfinite(snap.theta[0]) || !std::isfinite(snap.theta[1])) return false;
        if (!std::isfinite(snap.P[0][0]) || !std::isfinite(snap.P[0][1]) ||
            !std::isfinite(snap.P[1][0]) || !std::isfinite(snap.P[1][1])) return false;
        if (!std::isfinite(snap.model.K) || !std::isfinite(snap.model.tau) ||
            !std::isfinite(snap.model.L) || !std::isfinite(snap.model.a) ||
            !std::isfinite(snap.model.b) || !std::isfinite(snap.model.var_eps)) return false;
        Ts_ = snap.Ts;
        initBuffers();

        gains_ = snap.gains;
        model_ = snap.model;
        theta_[0] = snap.theta[0];
        theta_[1] = snap.theta[1];
        P_[0][0] = snap.P[0][0];
        P_[0][1] = snap.P[0][1];
        P_[1][0] = snap.P[1][0];
        P_[1][1] = snap.P[1][1];
        eps_var_ = std::max(0.0, snap.eps_var);

        d_hat_ = clampDelaySamples(snap.delay_samples);
        L_frac_ = snap.delay_fraction;
        delay_hist_.assign(delayHistSize(), d_hat_);
        delay_idx_ = 0;

        I_state_ = snap.I_state;
        y_prev_ = snap.y_prev;
        y_dot_ = snap.y_dot;
        last_u_ = clamp(snap.last_u, opts_.out_min, opts_.out_max);
        fit_rmse_ = snap.fit_rmse;
        fit_percent_ = snap.fit_percent;
        fit_buffer_.clear();
        adapt_on_ = snap.adapt_on;
        window_count_ = 0;
        return true;
    }

    bool saveCalibration(const std::string& filepath) const {
        std::ofstream ofs(filepath, std::ios::trunc);
        if (!ofs.is_open()) return false;
        ofs << "#AdaptivePID-calibration-v1\n";
        ofs << std::setprecision(17);
        CalibrationSnapshot snap = captureCalibration();
        ofs << snap.Ts << "\n";
        ofs << snap.gains.Kp << ' ' << snap.gains.Ti << ' ' << snap.gains.Td << ' '
            << snap.gains.beta << ' ' << snap.gains.N << "\n";
        ofs << snap.model.K << ' ' << snap.model.tau << ' ' << snap.model.L << ' '
            << snap.model.a << ' ' << snap.model.b << ' ' << snap.model.var_eps << ' '
            << (snap.model.valid ? 1 : 0) << "\n";
        ofs << snap.theta[0] << ' ' << snap.theta[1] << "\n";
        ofs << snap.P[0][0] << ' ' << snap.P[0][1] << ' ' << snap.P[1][0] << ' '
            << snap.P[1][1] << "\n";
        ofs << snap.eps_var << "\n";
        ofs << static_cast<unsigned long long>(snap.delay_samples) << ' ' << snap.delay_fraction << "\n";
        ofs << snap.I_state << ' ' << snap.y_prev << ' ' << snap.y_dot << ' ' << snap.last_u << "\n";
        ofs << snap.fit_rmse << ' ' << snap.fit_percent << "\n";
        ofs << (snap.adapt_on ? 1 : 0) << "\n";
        return ofs.good();
    }

    bool loadCalibration(const std::string& filepath) {
        std::ifstream ifs(filepath);
        if (!ifs.is_open()) return false;
        auto start = ifs.tellg();
        std::string header;
        std::getline(ifs, header);
        if (header.rfind("#AdaptivePID-calibration-v1", 0) != 0) {
            ifs.clear();
            ifs.seekg(start);
        }
        CalibrationSnapshot snap;
        int model_valid = 0;
        unsigned long long delay_samples = 0;
        int adapt_flag = 1;
        if (!(ifs >> snap.Ts)) return false;
        if (!(ifs >> snap.gains.Kp >> snap.gains.Ti >> snap.gains.Td >> snap.gains.beta >> snap.gains.N)) return false;
        if (!(ifs >> snap.model.K >> snap.model.tau >> snap.model.L >> snap.model.a >> snap.model.b >> snap.model.var_eps >> model_valid)) return false;
        if (!(ifs >> snap.theta[0] >> snap.theta[1])) return false;
        if (!(ifs >> snap.P[0][0] >> snap.P[0][1] >> snap.P[1][0] >> snap.P[1][1])) return false;
        if (!(ifs >> snap.eps_var)) return false;
        if (!(ifs >> delay_samples >> snap.delay_fraction)) return false;
        snap.delay_samples = static_cast<std::size_t>(delay_samples);
        if (!(ifs >> snap.I_state >> snap.y_prev >> snap.y_dot >> snap.last_u)) return false;
        if (!(ifs >> snap.fit_rmse >> snap.fit_percent)) return false;
        if (!(ifs >> adapt_flag)) return false;
        snap.adapt_on = (adapt_flag != 0);
        snap.model.valid = (model_valid != 0);
        return applyCalibration(snap);
    }

    // Force gains (bumpless: ajuste l’intégrateur)
    void setManualGains(double Kp, double Ti, double Td) {
        gains_.Kp = safePos(Kp);
        gains_.Ti = std::max(Ts_, safePos(Ti));
        gains_.Td = std::max(0.0, Td);
        // Ajuster l'état anti-windup pour éviter le saut
        // (optionnel: rien à faire si on garde l’intégrale)
    }

    // ---- Boucle principale (appel à chaque échantillon) ----
    // r: consigne; y: mesure; retourne u: commande
    double update(double r, double y, double u_ff = 0.0) {
        if (!std::isfinite(r) || !std::isfinite(y)) return last_u_;
        // Mise à jour des buffers pour estimation
        pushSample(r, y, last_u_); // on corrèle avec la commande réellement appliquée

        // 1) Estimation du délai (latence)
        if (adapt_on_) estimateDelay();

        // 2) RLS sur [a, b] avec u_{k-d}
        if (adapt_on_) rlsUpdate(y);

        // 3) De [a,b,d] -> [K, tau, L], validation, puis tuning IMC/SIMC
        if (adapt_on_) modelFromParamsAndTune();

        // 4) Exécution PID (formulation parallèle avec pondération de consigne)
        double e = r - y;

        // P avec pondération de consigne (dérivée sur la mesure)
        double up = gains_.Kp * (gains_.beta * r - y);

        // I (anti-windup back-calculation)
        // I_state correspond directement à la contribution intégrale (en unité de sortie)
        double Ki = (gains_.Ti > 0.0 ? gains_.Kp / gains_.Ti : 0.0);
        I_state_ += Ki * e * Ts_;
        // Dérivée filtrée (sur la mesure)
        double Td = gains_.Td;
        double uf = 0.0;
        if (Td > 0.0) {
            double Tf = Td / gains_.N;
            double alpha = Tf / (Tf + Ts_);
            y_dot_ = alpha * y_dot_ + (1.0 - alpha) * ((y - y_prev_) / Ts_);
            uf = -gains_.Kp * Td * y_dot_;
        }

        // Sortie non-saturée
        double u = up + I_state_ + uf + u_ff;

        // Saturation & anti-windup tracking
        double u_sat = clamp(u, opts_.out_min, opts_.out_max);
        if (gains_.Ti > 0.0) {
            double Tt = opts_.tracking_Tt_factor * gains_.Ti;
            if (Tt < Ts_) Tt = Ts_;
            I_state_ += (u_sat - u) * (Ts_ / Tt); // rétro-calcul
        }

        // Mises à jour mémoires
        y_prev_ = y;
        last_u_ = u_sat;

        // Statistiques pour FIT
        updateFit(y);

        return u_sat;
    }

    // Réinitialisation complète
    void reset() {
        // Gains par défaut
        gains_.Kp = 0.0; gains_.Ti = 1.0; gains_.Td = 0.0; gains_.beta = 1.0; gains_.N = 10.0;
        // États PID
        I_state_ = 0.0;
        y_prev_ = 0.0;
        y_dot_ = 0.0;
        last_u_ = 0.0;
        // RLS
        theta_[0] = 0.99; // a
        theta_[1] = 0.0;  // b
        P_[0][0] = P_[1][1] = opts_.rls_P0; P_[0][1] = P_[1][0] = 0.0;
        eps_var_ = 1e-6;
        // Modèle
        model_ = FOPDT{};
        model_.a = theta_[0];
        model_.b = theta_[1];
        model_.var_eps = eps_var_;
        model_.valid = false;
        // Délai
        delay_hist_.assign(delayHistSize(), 0);
        delay_idx_ = 0;
        d_hat_ = 0;
        // FIT
        fit_buffer_.clear(); fit_buffer_.reserve(opts_.fit_window);
    }

    // Rapport JSON des paramètres actuels et de la qualité
    std::string reportJSON() const {
        std::ostringstream os;
        os << std::setprecision(6) << std::fixed;
        os << "{";
        os << "\"Ts\":" << Ts_ << ",";
        os << "\"model\":{"
           << "\"K\":" << model_.K << ","
           << "\"tau\":" << model_.tau << ","
           << "\"L\":" << model_.L << ","
           << "\"a\":" << model_.a << ","
           << "\"b\":" << model_.b << ","
           << "\"var_eps\":" << model_.var_eps << ","
           << "\"valid\":" << (model_.valid ? "true":"false") << "},";
        os << "\"delay_samples\":" << d_hat_ << ",";
        os << "\"pid\":{"
           << "\"Kp\":" << gains_.Kp << ","
           << "\"Ti\":" << gains_.Ti << ","
           << "\"Td\":" << gains_.Td << ","
           << "\"beta\":" << gains_.beta << ","
           << "\"N\":" << gains_.N << "},";
        os << "\"fit\":{"
           << "\"rmse\":" << fit_rmse_ << ","
           << "\"fit_percent\":" << fit_percent_ << "},";
        os << "\"options\":{"
           << "\"imc_lambda\":" << opts_.imc_lambda << ","
           << "\"delay_max\":" << opts_.delay_max << ","
           << "\"corr_window\":" << opts_.corr_window << "}";
        os << "}";
        return os.str();
    }

    // Accès/lecture
    PIDGains gains() const { return gains_; }
    FOPDT    model() const { return model_; }
    size_t   delaySamples() const { return d_hat_; }
    double   lastOutput() const { return last_u_; }
    double   fitPercent() const { return fit_percent_; }
    double   fitRMSE() const { return fit_rmse_; }

private:
    // ---- Utilitaires numériques ----
    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(hi, x));
    }
    static double safePos(double x) {
        if (!std::isfinite(x)) return 0.0;
        return (x <= 0.0 ? std::numeric_limits<double>::min() : x);
    }

    // ---- Buffers & Initialisation ----
    void initBuffers() {
        size_t N = std::max(opts_.corr_window + opts_.delay_max + 2, size_t(16));
        u_buf_.assign(N, 0.0);
        y_buf_.assign(N, 0.0);
        r_buf_.assign(N, 0.0);
        buf_head_ = 0;
        // Pre-calc window means quickly via running sums
        sum_u_ = sum_y_ = 0.0;
        window_count_ = 0;
        if (opts_.delay_smooth == 0) opts_.delay_smooth = 1;
        if (delay_hist_.size() != delayHistSize()) {
            delay_hist_.assign(delayHistSize(), d_hat_);
            delay_idx_ = 0;
        }
    }

    void pushSample(double r, double y, double u_applied) {
        // ring buffer
        size_t N = u_buf_.size();
        buf_head_ = (buf_head_ + 1) % N;

        // for running sums over corr_window use a second pointer
        // but for simplicity compute means in computeCorrelation()
        u_buf_[buf_head_] = u_applied;
        y_buf_[buf_head_] = y;
        r_buf_[buf_head_] = r;

        if (window_count_ < opts_.corr_window) window_count_++;
    }

    // ---- Estimation du délai par corrélation ----
    void estimateDelay() {
        if (window_count_ < 8) return; // pas assez d'info
        // brute-force correlation over [delay_min, delay_max]
        size_t best_d = d_hat_;
        double best_score = -1.0;

        for (size_t d = opts_.delay_min; d <= opts_.delay_max; ++d) {
            double c = crossCorrelation(d);
            double score = std::fabs(c);
            if (score > best_score) {
                best_score = score;
                best_d = d;
            }
        }

        // raffinement sous-échantillon (quadratique) autour du max si possible
        double frac = 0.0;
        if (best_d > opts_.delay_min && best_d < opts_.delay_max) {
            double Cdm = std::fabs(crossCorrelation(best_d - 1));
            double Cd  = std::fabs(crossCorrelation(best_d));
            double Cdp = std::fabs(crossCorrelation(best_d + 1));
            double denom = (Cdm - 2.0*Cd + Cdp);
            if (std::fabs(denom) > 1e-12) {
                frac = 0.5 * (Cdm - Cdp) / denom; // in [-0.5, 0.5] ideally
                frac = clamp(frac, -0.49, 0.49);
            }
        }
        // Lissage du délai entier via un petit historique (majorité)
        if (!delay_hist_.empty()) {
            delay_hist_[delay_idx_] = best_d;
            delay_idx_ = (delay_idx_ + 1) % delay_hist_.size();
        }
        size_t smoothed = majority(delay_hist_);
        d_hat_ = smoothed;
        L_frac_ = frac; // pour information; L ≈ (d + frac)*Ts
    }

    // Corrélation croisée non normalisée sur la dernière fenêtre
    double crossCorrelation(size_t delay) const {
        // sum_{i=0}^{W-1} [ (u[k-delay-i]-mean_u) * (y[k-i]-mean_y) ]
        size_t W = std::min(window_count_, opts_.corr_window);
        if (W < 4 || delay >= u_buf_.size()-2) return 0.0;

        // compute means quickly over window (not normalized by variance for robustness to amplitude)
        double mean_u = 0.0, mean_y = 0.0;
        for (size_t i = 0; i < W; ++i) {
            size_t idx_y = dec(buf_head_, i);
            size_t idx_u = dec(buf_head_, i + delay);
            mean_y += y_buf_[idx_y];
            mean_u += u_buf_[idx_u];
        }
        mean_u /= double(W);
        mean_y /= double(W);

        double c = 0.0;
        for (size_t i = 0; i < W; ++i) {
            size_t idx_y = dec(buf_head_, i);
            size_t idx_u = dec(buf_head_, i + delay);
            double uy = (u_buf_[idx_u] - mean_u) * (y_buf_[idx_y] - mean_y);
            c += uy;
        }
        return c;
    }

    size_t dec(size_t head, size_t i) const {
        size_t N = u_buf_.size();
        return (head + N - (i % N)) % N;
    }

    static size_t majority(const std::vector<size_t>& v) {
        // majority vote on integers
        if (v.empty()) return 0;
        size_t best = 0, best_count = 0;
        for (size_t x : v) {
            size_t c = 0;
            for (size_t y : v) if (y == x) ++c;
            if (c > best_count) { best_count = c; best = x; }
        }
        return best;
    }

    size_t delayHistSize() const {
        return std::max<std::size_t>(1, opts_.delay_smooth);
    }

    size_t clampDelaySamples(size_t d) const {
        if (opts_.delay_max < opts_.delay_min) return opts_.delay_min;
        if (d < opts_.delay_min) return opts_.delay_min;
        if (d > opts_.delay_max) return opts_.delay_max;
        return d;
    }

    // ---- RLS ----
    void rlsUpdate(double yk) {
        // phi = [y_{k-1}, u_{k-d}]
        size_t idx_ud = dec(buf_head_, d_hat_);
        double ykm1 = y_prev_;
        double ukd  = u_buf_[idx_ud];

        double phi0 = ykm1;
        double phi1 = ukd;

        // y_hat
        double yhat = theta_[0]*phi0 + theta_[1]*phi1;
        double eps  = yk - yhat;

        // EWMA variance of residual
        eps_var_ = (1.0 - opts_.eps_var_alpha)*eps_var_ + opts_.eps_var_alpha*(eps*eps);
        double sigma = std::sqrt(std::max(eps_var_, 1e-12));

        // adaptive forgetting: reduce lambda when residual is large
        double rho = std::fabs(eps) / (sigma + 1e-12);
        double lam = opts_.lambda_max - opts_.fast_adapt_gain * std::min(1.0, rho);
        lam = clamp(lam, opts_.lambda_min, opts_.lambda_max);

        // K = P*phi / (lambda + phi^T*P*phi)
        double Pphi0 = P_[0][0]*phi0 + P_[0][1]*phi1;
        double Pphi1 = P_[1][0]*phi0 + P_[1][1]*phi1;
        double den   = lam + phi0*Pphi0 + phi1*Pphi1;
        if (den < 1e-12) den = 1e-12;
        double K0 = Pphi0 / den;
        double K1 = Pphi1 / den;

        // theta update
        theta_[0] += K0 * eps;
        theta_[1] += K1 * eps;

        // P update: P = (1/lam)*(P - [K]*[phi]^T*P)
        double P00 = P_[0][0];
        double P01 = P_[0][1];
        double P10 = P_[1][0];
        double P11 = P_[1][1];

        P_[0][0] = (P00 - K0*(phi0*P00 + phi1*P10)) / lam;
        P_[0][1] = (P01 - K0*(phi0*P01 + phi1*P11)) / lam;
        P_[1][0] = (P10 - K1*(phi0*P00 + phi1*P10)) / lam;
        P_[1][1] = (P11 - K1*(phi0*P01 + phi1*P11)) / lam;

        // clamp for symmetry & positiveness (simple guard)
        P_[0][1] = P_[1][0] = 0.5*(P_[0][1] + P_[1][0]);
        double maxP = 1e12;
        P_[0][0] = clamp(P_[0][0], 1e-18, maxP);
        P_[1][1] = clamp(P_[1][1], 1e-18, maxP);

        // expose to model
        model_.a = theta_[0];
        model_.b = theta_[1];
        model_.var_eps = eps_var_;
    }

    // ---- Modèle & tuning ----
    void modelFromParamsAndTune() {
        // Convertir a,b,d -> K, tau, L
        double a = model_.a;
        double b = model_.b;

        bool ok = std::isfinite(a) && std::isfinite(b) && (a > 0.0) && (a < 1.0 - 1e-6);
        if (!ok) { model_.valid = false; return; }

        double tau = -Ts_ / std::log(a);
        double Kp  = b / (1.0 - a);
        if (!std::isfinite(tau) || tau < Ts_*0.5 || tau > 1e4) { model_.valid = false; return; }
        if (!std::isfinite(Kp)  || std::fabs(Kp) < 1e-12 || std::fabs(Kp) > 1e6) { model_.valid = false; return; }

        // Dead-time (latence)
        double L = (double(d_hat_) + L_frac_) * Ts_;
        L = std::max(0.0, L);

        model_.tau = tau;
        model_.K   = Kp;
        model_.L   = L;
        model_.valid = true;

        // Tuning SIMC/IMC
        double lam = std::max(opts_.imc_lambda, std::max(opts_.imc_lambda_min, opts_.imc_lambda_relL * L));
        // SIMC PID rules for FOPDT:
        // Kc = (tau + 0.5 L) / (K (lam + 0.5 L)); Ti = min(tau + 0.5 L, 4 (lam + 0.5 L)); Td = (tau * L) / (2 tau + L)
        double Kc = (tau + 0.5*L) / (std::fabs(Kp) * (lam + 0.5*L));
        double Ti = std::min(tau + 0.5*L, 4.0*(lam + 0.5*L));
        double Td = (2.0*tau + L > 1e-9) ? (tau * L) / (2.0*tau + L) : 0.0;

        // Conserver le signe du gain procédé dans le signe de Kc
        if (Kp < 0.0) Kc = -Kc;

        // Clamp soft
        Kc = clamp(Kc, -1e6, 1e6);
        Ti = clamp(Ti, Ts_, 1e6);
        Td = clamp(Td, 0.0, 1e6);

        // Appliquer en douceur ? Ici on met à jour directement (simple, bumpless via anti-windup déjà en place)
        gains_.Kp = Kc;
        gains_.Ti = Ti;
        gains_.Td = Td;
    }

    // ---- FIT metrics ----
    void updateFit(double y) {
        // One-step-ahead prediction with current a,b,d
        size_t idx_ud = dec(buf_head_, d_hat_);
        double yhat = model_.a * y_prev_ + model_.b * u_buf_[idx_ud];
        if (!std::isfinite(yhat)) return;

        if (fit_buffer_.size() >= opts_.fit_window) fit_buffer_.erase(fit_buffer_.begin());
        fit_buffer_.push_back({y, yhat});

        // compute RMSE and FIT%
        double se = 0.0, dy = 0.0, mean_y = 0.0;
        for (auto &p : fit_buffer_) mean_y += p.obs;
        mean_y /= double(fit_buffer_.size());
        for (auto &p : fit_buffer_) {
            double e = p.obs - p.pred;
            se += e*e;
            double c = p.obs - mean_y;
            dy += c*c;
        }
        fit_rmse_ = std::sqrt(se / std::max<size_t>(1, fit_buffer_.size()));
        fit_percent_ = (dy > 1e-12) ? (100.0 * (1.0 - std::sqrt(se/dy))) : 0.0;
        fit_percent_ = clamp(fit_percent_, -100.0, 100.0);
    }

private:
    // Sample time
    double Ts_{0.01};

    // PID
    PIDGains gains_;
    double I_state_{0.0};
    double y_prev_{0.0};
    double y_dot_{0.0};
    double last_u_{0.0};

    // RLS (theta=[a,b], P 2x2)
    double theta_[2]{0.99, 0.0};
    double P_[2][2]{{1e3,0},{0,1e3}};
    double eps_var_{1e-6};

    // Buffers
    std::vector<double> u_buf_, y_buf_, r_buf_;
    size_t buf_head_{0};
    size_t window_count_{0};
    double sum_u_{0.0}, sum_y_{0.0}; // (non utilisés finalement, conservés pour extensions)

    // Delay estimation
    size_t d_hat_{0};
    double L_frac_{0.0};
    std::vector<size_t> delay_hist_;
    size_t delay_idx_{0};

    // Fit metrics
    struct ObsPred { double obs; double pred; };
    std::vector<ObsPred> fit_buffer_;
    double fit_rmse_{0.0};
    double fit_percent_{0.0};

    // Options
    Options opts_;
    bool adapt_on_{true};

    // Model
    FOPDT model_;
};

} // namespace ctrl
