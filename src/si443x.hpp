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
#define HANDLE_ERR(CMD)        \
	{                          \
		Status s = CMD;        \
		if (s != Status::OK) { \
			return s;          \
		}                      \
	}
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
		Status status = spi.exec(Command::PART_INFO, nullptr, &info);
		info.part = endian(info.part);
		info.id = endian(info.id);
		return status;
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
		Status status = spi.exec(Command::GPIO_PIN_CFG, &raw, &raw);

		// Convert the result back to the given configuration
		config = GpioConfig(raw);
		return status;
	}

	Status setFrequency(float center)
	{
		uint8_t outdiv, band;

		// see:
		// https://github.com/Yveaux/RadioHead/blob/master/RadioHead/RH_RF24.cpp
		// Non-continuous frequency bands
		if (center <= 1050.0 && center >= 850.0) {
			outdiv = 4;
			band = 0;
		} else if (center <= 525.0 && center >= 425.0) {
			outdiv = 8;
			band = 2;
		} else if (center <= 350.0 && center >= 284.0) {
			outdiv = 12;
			band = 3;
		} else if (center <= 175.0 && center >= 142.0) {
			outdiv = 24;
			band = 5;
		} else {
			return Status::ERR_OUT_OF_RANGE;
		}

		// Calculate the RF frequencies
		center *= 1000000.0;  // Convert to Hz
		uint32_t xtal_freq = 30000000L;
		unsigned long f_pfd = 2 * xtal_freq / outdiv;
		unsigned int n = ((unsigned int)(center / f_pfd)) - 1;
		float ratio = center / (float)f_pfd;
		float rest = ratio - (float)n;
		unsigned long m = (unsigned long)(rest * 524288UL);

		// Set band and frequency
		HANDLE_ERR(set(Property::MODEM_CLKGEN_BAND, band | 8));
		HANDLE_ERR(set(Property::FREQ_CONTROL_INTE, n));
		HANDLE_ERR(set(Property::FREQ_CONTROL_FRAC, uint24(m)));
		return Status::OK;
	}

	Status setModulation(ModType type, ModSource source = ModSource::PACKET,
	                     TxDirectModeGpio directGpio = TxDirectModeGpio::GPIO0,
	                     TxDirectModeType directType = TxDirectModeType::SYNC)
	{
		return set(Property::MODEM_MOD_TYPE,
		           static_cast<uint8_t>(type) | static_cast<uint8_t>(source) |
		               static_cast<uint8_t>(directGpio) |
		               static_cast<uint8_t>(directType));
	}

	Status changeState(State state) {
		return spi.exec(Command::CHANGE_STATE, &state, nullptr);
	}

	Status set(const Property8 &prop, uint8_t value)
	{
		SetPropertyRaw<uint8_t> raw(prop.group, prop.size, prop.id, value);
		return spi.exec(Command::SET_PROPERTY, &raw, nullptr);
	}

	Status set(const Property16 &prop, uint16_t value)
	{
		SetPropertyRaw<uint16_t> raw(prop.group, prop.size, prop.id,
		                             endian16(value));
		return spi.exec(Command::SET_PROPERTY, &raw, nullptr);
	}

	Status set(const Property24 &prop, uint24 value)
	{
		SetPropertyRaw<uint24> raw(prop.group, prop.size, prop.id, value);
		return spi.exec(Command::SET_PROPERTY, &raw, nullptr);
	}

	Status set(const Property32 &prop, uint32_t value)
	{
		SetPropertyRaw<uint32_t> raw(prop.group, prop.size, prop.id,
		                             endian32(value));
		return spi.exec(Command::SET_PROPERTY, &raw, nullptr);
	}

	Status get(const Property8 &prop, uint8_t &value)
	{
		GetPropertyRaw raw(prop.group, prop.size, prop.id);
		return spi.exec(Command::GET_PROPERTY, &raw, &value);
	}

	Status get(const Property16 &prop, uint16_t &value)
	{
		GetPropertyRaw raw(prop.group, prop.size, prop.id);
		Status status = spi.exec(Command::GET_PROPERTY, &raw, &value);
		value = endian16(value);
		return status;
	}

	Status get(const Property24 &prop, uint24 &value)
	{
		GetPropertyRaw raw(prop.group, prop.size, prop.id);
		return spi.exec(Command::GET_PROPERTY, &raw, &value);
	}

	Status get(const Property32 &prop, uint32_t &value)
	{
		GetPropertyRaw raw(prop.group, prop.size, prop.id);
		Status status = spi.exec(Command::GET_PROPERTY, &raw, &value);
		value = endian32(value);
		return status;
	}
#undef HANDLE_ERR
};
}

#endif /* _SI443X_HPP_ */

