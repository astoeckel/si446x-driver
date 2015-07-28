/*
 *  si443x-driver -- Driver for the Silabs Si443x wireless transceiver IC
 *  Copyright (C) 2015 Andreas Stöckel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file spi_driver_generic.hpp
 *
 * Contains the generic interface for the GPIO driver that is used for resetting
 * the device and (optinally) switching the RX/TX antenna.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_GPIO_DRIVER_GENERIC_HPP_
#define _SI443X_GPIO_DRIVER_GENERIC_HPP_

#include <stdint.h>

#include <si443x_types.hpp>

namespace si443x {
/**
 * Generic interface used by the Si443x driver to access additional pins of the
 * hardware that are not reachable via SPI. Uses the curiously recurring
 * template pattern to implement inheritance without virtual functions.
 */
template<typename GpioDriverImpl>
class GpioDriverGeneric {
public:
	/**
	 * Should set the reset pin to high/low.
	 */
	void reset(bool high) {
		static_cast<GpioDriverImpl*>(this)->resetImpl(high);
	}

	/**
	 * Should be used to select the tx antenna.
	 */
	void txAnt(bool high) {
		static_cast<GpioDriverImpl*>(this)->txAntImpl(high);
	}

	/**
	 * Should be used to select the rx antenna.
	 */
	void rxAnt(bool high) {
		static_cast<GpioDriverImpl*>(this)->rxAntImpl(high);
	}

	/**
	 * Switches between RX/TX and NONE.
	 */
	void antennaState(AntennaState state) {
		switch (state) {
			case AntennaState::TX:
				rxAnt(false);
				txAnt(true);
				break;
			case AntennaState::RX:
				txAnt(false);
				rxAnt(true);
				break;
			case AntennaState::OFF:
				txAnt(false);
				rxAnt(false);
				break;
		}
	}
};

}

#endif /* _SI443X_GPIO_DRIVER_GENERIC_HPP_ */

