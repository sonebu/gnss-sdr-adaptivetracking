/*!
 * \file gps_l1_ca_gradient_pi_tracking.h
 * \brief Adapter for GPS L1 C/A PI-shaped gradient tracking
 *        (atan2 / E-L / fll_diff_atan as discriminator-proxies for joint gradient updates).
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef GNSS_SDR_GPS_L1_CA_GRADIENT_PI_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_GRADIENT_PI_TRACKING_H

#include "base_dll_pll_tracking.h"

class GpsL1CaGradientPiTracking : public BaseDllPllTracking
{
public:
    GpsL1CaGradientPiTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    inline std::string implementation() override
    {
        return "GPS_L1_CA_Gradient_PI_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};

#endif  // GNSS_SDR_GPS_L1_CA_GRADIENT_PI_TRACKING_H
