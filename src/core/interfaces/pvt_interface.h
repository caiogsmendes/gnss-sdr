/*!
 * \file pvt_interface.h
 * \brief This class represents an interface to a PVT block.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Abstract class for PVT solvers. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_PVT_INTERFACE_H
#define GNSS_SDR_PVT_INTERFACE_H

#include "galileo_almanac.h"
#include "galileo_ephemeris.h"
#include "gnss_block_interface.h"
#include "gps_almanac.h"
#include "gps_ephemeris.h"
#include "gnss_synchro.h"
#include <map>

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */


/*!
 * \brief This class represents an interface to a PVT block.
 *
 * Abstract class for PVT interfaces, derived from GNSSBlockInterface.
 * Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class PvtInterface : public GNSSBlockInterface
{
public:
    virtual void reset() = 0;
    virtual void clear_ephemeris() = 0;
    virtual std::map<int, Gps_Ephemeris> get_gps_ephemeris() const = 0;
    virtual std::map<int, Galileo_Ephemeris> get_galileo_ephemeris() const = 0;
    virtual std::map<int, Gps_Almanac> get_gps_almanac() const = 0;
    virtual std::map<int, Galileo_Almanac> get_galileo_almanac() const = 0;
    // Caio:
    virtual std::map<int, Gnss_Synchro> get_gnss_observables() const = 0;
    virtual int get_num_sat_observ() = 0;
    virtual bool got_first_fix() = 0;
    // virtual std::map<int, Gnss_Ephemeris> get_gnss_ephemeris() const = 0;

    virtual bool get_latest_PVT(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        double* ground_speed_kmh,
        double* course_over_ground_deg,
        time_t* UTC_time) = 0;

    virtual bool get_latest_PVT_(double* longitude_deg,
        double* latitude_deg,
        double* height_m,
        time_t* UTC_time,
        double* gps_time_offset,
        double* rx_posX,
        double* rx_posY,
        double* rx_posZ,
        double* rx_velX,
        double* rx_velY,
        double* rx_velZ) = 0;

    virtual bool get_latest_PVT_2(double* rx_posX,
        double* rx_posY,
        double* rx_posZ,
        double* rx_velX,
        double* rx_velY,
        double* rx_velZ,
        time_t* UTC_time,
        double* gdop,
        double* hdop,
        double* vdop,
        double* pdop) = 0;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_INTERFACE_H
