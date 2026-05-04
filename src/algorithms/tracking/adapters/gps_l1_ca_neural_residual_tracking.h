/*!
 * \file gps_l1_ca_neural_residual_tracking.h
 * \brief GPS L1 C/A tracking with classical DLL/PLL plus online LibTorch residual MLP.
 */
#ifndef GNSS_SDR_GPS_L1_CA_NEURAL_RESIDUAL_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_NEURAL_RESIDUAL_TRACKING_H

#include "base_dll_pll_tracking.h"

class GpsL1CaNeuralResidualTracking : public BaseDllPllTracking
{
public:
    GpsL1CaNeuralResidualTracking(const ConfigurationInterface *configuration,
        const std::string &role,
        unsigned int in_streams,
        unsigned int out_streams);

    inline std::string implementation() override
    {
        return "GPS_L1_CA_Neural_Residual_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface *configuration) override;
    void create_tracking_block() override;
};

#endif  // GNSS_SDR_GPS_L1_CA_NEURAL_RESIDUAL_TRACKING_H
