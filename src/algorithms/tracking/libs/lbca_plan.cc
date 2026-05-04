/*!
 * \file lbca_plan.cc
 * \brief Implements LBCA + PLAN normalization and PLL bandwidth stepping.
 *
 * Discriminator statistics (Ewma §3.2 FL preprocessing): mean μ̂ via IIR low-pass
 * and σ̂² as EWMA of squared deviations. |μᵤₑ| is |E[eᵤ]| (= |IIR(eᵤ)|) so D̃ → 0
 * under zero-mean noise — NOT E[|eᵤ|] which is ≈√(2/π)·σ even in static cases.
 * Normalization Ñ,D̃ (25–26), control c = g_max·D̃ − g, B̂ = B + c (38–39) with
 * g_max = S from (59–60). PLAN piecewise approximation (45); paper-literal dead-
 * zone Schmitt per Eq.(15); output clipped to FAB-style [B_min, B_max] (58);
 * B_N = B·τ_int (7).
 *
 * -----------------------------------------------------------------------------
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "lbca_plan.h"
#include "dll_pll_conf.h"
#include <algorithm>
#include <cmath>

void LbcaPlan::reload_from_conf(const Dll_Pll_Conf &conf)
{
    enable_ = conf.lbca_pll_enable;
    stats_alpha_ = conf.lbca_stats_alpha;
    T_LBCA_ = conf.lbca_T_LBCA;
    S_ = conf.lbca_S;
    use_plan_ = conf.lbca_use_plan;
    delta_b_hz_ = conf.lbca_pll_delta_bw_hz;
    pll_b_min_hz_ = conf.lbca_pll_bw_min_hz;
    pll_b_max_hz_ = conf.lbca_pll_bw_max_hz;
    warmup_steps_ = conf.lbca_warmup_steps;
}


void LbcaPlan::reset(double initial_pll_bw_hz)
{
    step_counter_ = 0;
    ewma_mean_eu_ = 0.0;
    ewma_var_ = 0.0;
    B_schmitt_latched_ = initial_pll_bw_hz;
}


void LbcaPlan::clear_statistics(double pll_bw_reference_hz)
{
    step_counter_ = 0;
    ewma_mean_eu_ = 0.0;
    ewma_var_ = 0.0;
    B_schmitt_latched_ = pll_bw_reference_hz;
}


double LbcaPlan::sigmoid_plan(double Sk_arg)
{
    if (Sk_arg >= 5.0)
        return 1.0;
    if (Sk_arg >= 2.375)
        return 0.03125 * Sk_arg + 0.84375;
    if (Sk_arg >= 1.0)
        return 0.125 * Sk_arg + 0.625;
    if (Sk_arg >= 0.0)
        return 0.25 * Sk_arg + 0.5;
    return 1.0 - sigmoid_plan(-Sk_arg);
}


double LbcaPlan::sigmoid_eval(double linear_arg) const
{
    if (use_plan_)
        return sigmoid_plan(linear_arg);
    if (linear_arg >= 34.0)
        return 1.0;
    if (linear_arg <= -34.0)
        return 0.0;
    return 1.0 / (1.0 + std::exp(-linear_arg));
}


double LbcaPlan::weighting_g(double BN) const
{
    const double s1 = sigmoid_eval(50.0 * (BN - 0.06));
    const double s2 = sigmoid_eval(250.0 * (BN - 0.36));
    return S_ * (T_LBCA_ * s1 + (1.0 - T_LBCA_) * s2);
}


double LbcaPlan::schmitt_update(double B_latched_hz, double B_hat_hz, double delta_b_hz)
{
    // Paper Eq.(15) dead-zone Schmitt: B[n+1] = B[n] ± ΔB only when |B̂−B|>ΔB,
    // B[n+1] = B[n] otherwise. Combined with the FAB-style clip applied to B̂
    // outside (Eq.58) and the [B_min,B_max] re-clip on the return, the actual
    // bandwidth ratchets toward the clamp in ΔB-sized steps and then sits there.
    const double diff = B_hat_hz - B_latched_hz;
    if (diff > delta_b_hz)
        return B_latched_hz + delta_b_hz;
    if (diff < -delta_b_hz)
        return B_latched_hz - delta_b_hz;
    return B_latched_hz;
}


double LbcaPlan::step_bandwidth_hz(double pll_disc_hz, double pll_bw_current_hz,
    double tau_int_s)
{
    if (!enable_)
        {
            return pll_bw_current_hz;
        }

    const double alpha = stats_alpha_;

    const double mean_prev = ewma_mean_eu_;
    ewma_mean_eu_ = alpha * pll_disc_hz + (1.0 - alpha) * ewma_mean_eu_;
    ewma_var_ = alpha * (pll_disc_hz - mean_prev) * (pll_disc_hz - mean_prev) +
                (1.0 - alpha) * ewma_var_;

    ++step_counter_;
    if (warmup_steps_ > 0 &&
        step_counter_ <= static_cast<uint32_t>(warmup_steps_))
        {
            B_schmitt_latched_ = pll_bw_current_hz;
            return pll_bw_current_hz;
        }

    double sigma_eu = std::sqrt(std::max(ewma_var_, 0.0));
    sigma_eu = std::max(sigma_eu, 1e-18);

    // Paper Eq.(25)-(26) take the absolute value of the *mean* estimate, not the
    // mean of |eᵤ|. With E[|x|]=√(2/π)·σ for zero-mean Gaussian, the latter would
    // pin D̃≈0.44 even in pure-noise static cases and erroneously raise the BW.
    const double abs_mean_eu = std::fabs(ewma_mean_eu_);
    const double denom = sigma_eu + std::max(abs_mean_eu, 1e-18);

    const double Dtilde = abs_mean_eu / denom;
    const double BN = std::max(0.0, pll_bw_current_hz) * tau_int_s;
    const double g_bn = weighting_g(BN);
    const double c = S_ * Dtilde - g_bn;

    double B_hat = pll_bw_current_hz + c;

    B_hat = std::max(pll_b_min_hz_, std::min(pll_b_max_hz_, B_hat));

    double B_next = schmitt_update(B_schmitt_latched_, B_hat, delta_b_hz_);

    B_next = std::max(pll_b_min_hz_, std::min(pll_b_max_hz_, B_next));

    B_schmitt_latched_ = B_next;

    return B_next;
}
