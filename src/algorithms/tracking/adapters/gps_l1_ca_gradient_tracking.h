/*!
 * \file gps_l1_ca_gradient_tracking.h
 * \brief Adapter for GPS L1 C/A gradient-based joint tracking (correlator power loss).
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef GNSS_SDR_GPS_L1_CA_GRADIENT_TRACKING_H
#define GNSS_SDR_GPS_L1_CA_GRADIENT_TRACKING_H

#include "base_dll_pll_tracking.h"

class GpsL1CaGradientTracking : public BaseDllPllTracking
{
public:
    GpsL1CaGradientTracking(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    inline std::string implementation() override
    {
        return "GPS_L1_CA_Gradient_Tracking";
    }

private:
    void configure_tracking_parameters(const ConfigurationInterface* configuration) override;
    void create_tracking_block() override;
};

#endif  // GNSS_SDR_GPS_L1_CA_GRADIENT_TRACKING_H
