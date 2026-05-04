/*!
 * \file neural_residual_tracking_cell.h
 * \brief Tiny LibTorch MLP: online-learned **code-rate (DLL) residual** on classical tracking.
 */

#ifndef GNSS_SDR_NEURAL_RESIDUAL_TRACKING_CELL_H
#define GNSS_SDR_NEURAL_RESIDUAL_TRACKING_CELL_H

#include "dll_pll_conf.h"
#include <cstdint>
#include <memory>

class NeuralResidualTrackingCell
{
public:
    explicit NeuralResidualTrackingCell(const Dll_Pll_Conf &conf);
    NeuralResidualTrackingCell(const NeuralResidualTrackingCell &) = delete;
    NeuralResidualTrackingCell &operator=(const NeuralResidualTrackingCell &) = delete;
    ~NeuralResidualTrackingCell();

    /*!
     * Builds features from correlator outputs + discriminators, runs Adam aligning the bounded
     * **code-chip-rate delta** with the normalized DLL discriminator; applies only to **code NCO rate**.
     */
    void step_apply_and_learn(
        float Ip,
        float Qp,
        float abs_early,
        float abs_prompt,
        float abs_late,
        float pll_folded_rad,
        double dll_err_chips_ti,
        double fll_err_hz,
        double coh_integration_s,
        double cn0_db_hz,
        double carrier_lock_test,
        bool carrier_aiding,
        double code_chip_rate,
        double signal_carrier_freq_hz,
        float &rem_carr_phase_rad_io,
        double &carrier_doppler_hz_io,
        double &code_freq_chips_io);

    /*! Call on loss-of-lock / clear_tracking_vars — restarts warmup ramp counters. */
    void on_tracking_reset();

    /*! Write neural_ckpt_save_path if nonempty. */
    void save_checkpoint_best_effort(bool quiet);

private:
    class Impl;
    std::unique_ptr<Impl> d_i;
};

#endif  // GNSS_SDR_NEURAL_RESIDUAL_TRACKING_CELL_H
