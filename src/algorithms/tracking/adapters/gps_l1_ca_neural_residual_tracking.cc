/*!
 * \file gps_l1_ca_neural_residual_tracking.cc
 * \brief GPS L1 C/A classical tracking + neural residual (see neural_residual_tracking_cell).
 */
#include "gps_l1_ca_neural_residual_tracking.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaNeuralResidualTracking::GpsL1CaNeuralResidualTracking(
    const ConfigurationInterface *configuration,
    const std::string &role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GpsL1CaNeuralResidualTracking::configure_tracking_parameters(
    const ConfigurationInterface *configuration __attribute__((unused)))
{
    config_params().system = 'G';
    const std::array<char, 3> sig{'1', 'C', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);

    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    config_params().vector_length = vector_length;

    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A: extend_correlation_symbols must be > 0. "
                      << "Coherent integration set to 1 ms."
                      << TEXT_RESET << std::endl;
        }
    else if (config_params().extend_correlation_symbols > 20)
        {
            config_params().extend_correlation_symbols = 20;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A: extend_correlation_symbols limited to 20 (20 ms)."
                      << TEXT_RESET << std::endl;
        }

    config_params().track_pilot = configuration->property(this->role() + ".track_pilot", false);
    if (config_params().track_pilot)
        {
            config_params().track_pilot = false;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A does not have pilot signal. "
                      << "Data tracking enabled instead."
                      << TEXT_RESET << std::endl;
        }

    config_params().neural_residual_tracking = true;
}


void GpsL1CaNeuralResidualTracking::create_tracking_block()
{
    if (config_params().item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(config_params());
            DLOG(INFO) << "Neural residual tracking block (" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            set_item_size(0);
            tracking_sptr_ = nullptr;
            LOG(WARNING) << config_params().item_type << " unknown tracking item type.";
        }
}
