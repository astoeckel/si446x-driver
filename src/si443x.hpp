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
 * @file si443x.hpp
 *
 * Contains actual driver for the Si443x series.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_HPP_
#define _SI443X_HPP_

#include "si443x_types.hpp"

namespace si443x {

template <typename SpiDriver, typename GpioDriver, typename DelayDriver>
class Si443x {
private:
	SpiDriver &spi;
	GpioDriver &gpio;
	DelayDriver &delay;

public:
	Si443x(SpiDriver &spi, GpioDriver &gpio, DelayDriver &delay)
	    : spi(spi), gpio(gpio), delay(delay)
	{
	}

	Status reset()
	{
		// Disable the antenna
		gpio.antennaState(AntennaState::OFF);

		// Perform a reset cycle
		gpio.reset(true);
		delay.ms(20);
		gpio.reset(false);

		// Wait until the device has booted up
		return spi.waitNotBusy();
	}

	Status powerup()
	{
		PowerUp args;
		args.boot_options = 0x01;             // No patch, boot EZRadio PRO FW
		args.xtal_options = 0x00;             // Internal oscillator
		args.xo_freq = endian32(0x01C9C380);  // 30 MHz (default value)
		return spi.exec(Command::POWER_UP, &args, nullptr);
	}

	Status partInfo(PartInfo &info)
	{
		Status res = spi.exec(Command::PART_INFO, nullptr, &info);
		info.part = endian(info.part);
		info.id = endian(info.id);
		return res;
	}

	Status funcInfo(FuncInfo &info)
	{
		return spi.exec(Command::FUNC_INFO, nullptr, &info);
	}

	Status gpioConfig(GpioConfig &config)
	{
		// Convert the gpio configuration to the RAW format and execute the
		// spio command.
		GpioConfigRaw raw = config.toRaw();
		Status res = spi.exec(Command::GPIO_PIN_CFG, &raw, &raw);

		// Convert the result back to the given configuration
		config = GpioConfig(raw);
		return res;
	}
};
}

#endif /* _SI443X_HPP_ */

