/*!
 * \file lbca_plan.h
 * \brief LBCA + PLAN bandwidth control for Cortés-style PLL adaptation (Sensors 2021).
 *
 * -----------------------------------------------------------------------------
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_LBCA_PLAN_H
#define GNSS_SDR_LBCA_PLAN_H

#include <cstdint>

class Dll_Pll_Conf;


class LbcaPlan
{
public:
    void reload_from_conf(const Dll_Pll_Conf &conf);

    void reset(double initial_pll_bw_hz);

    void clear_statistics(double pll_bw_reference_hz);

    double step_bandwidth_hz(double pll_discriminator_hz, double pll_bw_current_hz,
        double tau_int_s);

    bool enabled() const { return enable_; }

private:
    static double sigmoid_plan(double Sk_arg);

    double sigmoid_eval(double linear_arg) const;
    double weighting_g(double BN) const;
    double schmitt_update(double B_latched_hz, double B_hat_hz, double delta_b_hz);

    bool enable_{false};
    double stats_alpha_{0.05};
    double T_LBCA_{0.2};
    double S_{0.1};
    bool use_plan_{true};
    double delta_b_hz_{0.5};
    double pll_b_min_hz_{4.0};
    double pll_b_max_hz_{18.0};
    int32_t warmup_steps_{10};

    uint32_t step_counter_{0};
    double ewma_mean_eu_{0.0};
    double ewma_var_{0.0};
    double B_schmitt_latched_{35.0};
};


#endif /* GNSS_SDR_LBCA_PLAN_H */
