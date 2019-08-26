/*!
 * \file gnss_block_interface.h
 * \brief This interface represents a GNSS block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef OPEN_LOOP_PROCESSING_INTERFACE_H
#define OPEN_LOOP_PROCESSING_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"

/*!
 * \brief This abstract class represents an interface to GNSS blocks.
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class OpenLoopProcessingInterface : public GNSSBlockInterface
{
public:
    virtual void set_gnss_synchro(Gnss_Synchro* gnss_synchro) = 0;
    virtual void set_channel(unsigned int channel) = 0;
};
#endif /*OPEN_LOOP_PROCESSING_INTERFACE_H*/
