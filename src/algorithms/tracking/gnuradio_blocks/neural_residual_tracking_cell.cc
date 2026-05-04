/*!
 * \file neural_residual_tracking_cell.cc
 */

#include "neural_residual_tracking_cell.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#include <torch/torch.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace
{
struct TinyResidualNetImpl : torch::nn::Module
{
    torch::nn::Linear fc1{nullptr};
    torch::nn::Linear fc2{nullptr};

    TinyResidualNetImpl(int64_t in_feats, int64_t hidden, int64_t out_dims)
    {
        fc1 = register_module("fc1", torch::nn::Linear(in_feats, hidden));
        fc2 = register_module("fc2", torch::nn::Linear(hidden, out_dims));
    }

    torch::Tensor forward(torch::Tensor x)
    {
        x = torch::relu(fc1->forward(x));
        return fc2->forward(x);
    }
};

TORCH_MODULE(TinyResidualNet);

constexpr int kInFeats = 11;
constexpr int kOutDims = 1;  // DLL / code-rate residual only
}  // namespace

class NeuralResidualTrackingCell::Impl
{
public:
    explicit Impl(const Dll_Pll_Conf &c)
        : conf(c),
          net(kInFeats, std::max<int32_t>(4, c.neural_hidden), kOutDims)
    {
        if (!conf.neural_ckpt_load_path.empty())
            {
                try
                    {
                        std::ifstream test(conf.neural_ckpt_load_path);
                        if (test.good())
                            {
                                torch::load(net, conf.neural_ckpt_load_path);
                                LOG(INFO) << "Neural residual checkpoint loaded: " << conf.neural_ckpt_load_path;
                            }
                    }
                catch (const std::exception &ex)
                    {
                        LOG(WARNING) << "Neural residual checkpoint load failed (starting fresh weights): "
                                     << ex.what();
                    }
            }

        torch::optim::AdamOptions adam_opt(static_cast<double>(conf.neural_lr));
        adam_opt.betas(std::tuple<double, double>{0.9, 0.999});
        opt = std::make_unique<torch::optim::Adam>(net->parameters(), adam_opt);
    }

    void reset_counters() { warmup_step = 0; }

    void save_checkpoint_best_effort(bool quiet)
    {
        if (conf.neural_ckpt_save_path.empty())
            {
                return;
            }
        try
            {
                torch::save(net, conf.neural_ckpt_save_path);
                if (!quiet)
                    {
                        DLOG(INFO) << "Saved neural residual checkpoint: " << conf.neural_ckpt_save_path;
                    }
            }
        catch (const std::exception &ex)
            {
                LOG(WARNING) << "Neural residual checkpoint save failed: " << ex.what();
            }
    }

    void tick_save()
    {
        if (conf.neural_ckpt_every_n == 0U)
            {
                return;
            }
        save_counter++;
        if ((save_counter % conf.neural_ckpt_every_n) == 0U)
            {
                save_checkpoint_best_effort(true);
            }
    }

    void step(float Ip,
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
        double &code_freq_chips_io)
    {
        (void)abs_prompt;
        (void)carrier_aiding;
        (void)code_chip_rate;
        (void)signal_carrier_freq_hz;

        if (conf.neural_cn0_learn_gate_db >= 0.0 &&
            cn0_db_hz > 1.0e-6 &&
            cn0_db_hz < conf.neural_cn0_learn_gate_db)
            {
                return;
            }
        if (conf.neural_lock_learn_gate >= -0.999 &&
            carrier_lock_test < conf.neural_lock_learn_gate)
            {
                return;
            }

        warmup_step++;

        float mag_p = std::sqrt(Ip * Ip + Qp * Qp) + 1e-6F;
        const float denom = mag_p;

        float zvals[kInFeats] = {
            Ip / denom,
            Qp / denom,
            abs_early / denom,
            abs_late / denom,
            std::tanh(static_cast<float>(pll_folded_rad)),
            std::tanh(static_cast<float>(dll_err_chips_ti / 0.25)),
            std::tanh(static_cast<float>(fll_err_hz / 200.0)),
            static_cast<float>(cn0_db_hz / 50.0),
            static_cast<float>(carrier_lock_test),
            std::clamp(static_cast<float>(coh_integration_s / 0.020), 0.05F, 5.0F),
            1.0F,
        };

        torch::Tensor z_in = torch::from_blob(zvals, {1, kInFeats}, torch::kFloat32).clone();

        const double denom_g = std::sqrt(pll_folded_rad * pll_folded_rad +
                                    (dll_err_chips_ti / 0.25) * (dll_err_chips_ti / 0.25) +
                                    (fll_err_hz / 200.0) * (fll_err_hz / 200.0)) +
            1.0e-6;
        const float g_dll = static_cast<float>((dll_err_chips_ti / 0.25) / denom_g);

        opt->zero_grad();

        torch::Tensor raw = net->forward(z_in).squeeze();

        const float dc_max = static_cast<float>(conf.neural_max_dcode_chips_s);

        const float warmup_ramp =
            conf.neural_warmup_steps > 0
                ? std::min(1.0F, static_cast<float>(warmup_step) / static_cast<float>(conf.neural_warmup_steps))
                : 1.0F;

        torch::Tensor cand_dc = dc_max * torch::tanh(raw);
        torch::Tensor pred = torch::tensor(warmup_ramp, torch::kFloat32) * cand_dc;
        torch::Tensor g_dll_t = torch::tensor(g_dll, torch::kFloat32);

        const double lam = conf.neural_lambda_delta;
        torch::Tensor alignment_term = -(pred * g_dll_t);
        torch::Tensor delta_reg = lam * cand_dc * cand_dc;
        torch::Tensor loss = alignment_term + delta_reg;

        loss.backward();

        const double clip_nm = conf.neural_grad_clip_norm;
        if (clip_nm > 0.0)
            {
                torch::nn::utils::clip_grad_norm_(net->parameters(), clip_nm);
            }

        const double apply_dc = pred.detach().item<double>();

        opt->step();

        tick_save();

        code_freq_chips_io += apply_dc;
        (void)rem_carr_phase_rad_io;
        (void)carrier_doppler_hz_io;
    }

private:
    Dll_Pll_Conf conf;
    TinyResidualNet net;
    std::unique_ptr<torch::optim::Adam> opt;
    uint64_t warmup_step{0};
    uint64_t save_counter{0};
};


NeuralResidualTrackingCell::NeuralResidualTrackingCell(const Dll_Pll_Conf &conf)
    : d_i(std::make_unique<Impl>(conf))
{
}


NeuralResidualTrackingCell::~NeuralResidualTrackingCell()
{
    if (d_i)
        {
            d_i->save_checkpoint_best_effort(true);
        }
}


void NeuralResidualTrackingCell::step_apply_and_learn(
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
    double &code_freq_chips_io)
{
    if (d_i)
        {
            d_i->step(Ip, Qp, abs_early, abs_prompt, abs_late, pll_folded_rad, dll_err_chips_ti,
                fll_err_hz, coh_integration_s, cn0_db_hz, carrier_lock_test, carrier_aiding,
                code_chip_rate, signal_carrier_freq_hz,
                rem_carr_phase_rad_io, carrier_doppler_hz_io, code_freq_chips_io);
        }
}


void NeuralResidualTrackingCell::on_tracking_reset()
{
    if (d_i)
        {
            d_i->reset_counters();
        }
}


void NeuralResidualTrackingCell::save_checkpoint_best_effort(bool quiet)
{
    if (d_i)
        {
            d_i->save_checkpoint_best_effort(quiet);
        }
}
