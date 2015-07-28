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
 * @file spi_driver_rpi.hpp
 *
 * Contains the generic interface for the GPIO driver that is used for resetting
 * the device and (optinally) switching the RX/TX antenna.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_GPIO_DRIVER_RPI_HPP_
#define _SI443X_GPIO_DRIVER_RPI_HPP_

#include "gpio_driver_generic.hpp"

namespace si443x {
/**
 * GpioDriverRpi is uses the wiringPi library to access the Raspberry Pi GPIO
 * pins.
 */
class GpioDriverRpi : public GpioDriverGeneric<GpioDriverRpi> {
	friend GpioDriverGeneric<GpioDriverRpi>;

private:
	int pinReset, pinTxAnt, pinRxAnt;

	void resetImpl(bool high);
	void txAntImpl(bool high);
	void rxAntImpl(bool high);

public:
	GpioDriverRpi(int pinReset = -1, int pinTxAnt = -1, int pinRxAnt = -1);
};
}

#endif /* _SI443X_GPIO_DRIVER_GENERIC_HPP_ */

