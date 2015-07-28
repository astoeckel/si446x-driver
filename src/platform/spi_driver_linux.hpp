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
 * @file spi_driver_linux.hpp
 *
 * Implementation of SpiDriverGeneric to use the "spidev" driver under linux
 * to access the SPI bus. The driver uses the spidev and has been tested on a
 * Raspberry Pi Model B.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_SPI_DRIVER_LINUX_HPP_
#define _SI443X_SPI_DRIVER_LINUX_HPP_

#include "spi_driver_generic.hpp"

namespace si443x {
/**
 * Uses clock_gettime to provide a timeout after 100ms have passed since a
 * command was issued.
 */
class ClockTimeoutProvider {
private:
	static constexpr uint64_t MaxTime = 1000;  // 100ms

	uint64_t startTs;

public:
	/**
	 * Constructor of the ClockTimeoutProvider class, resets the structure to
	 * its initial state.
	 */
	ClockTimeoutProvider();

	/**
	 * Resets the ClockTimeoutProvider to its initial state by setting the start
	 * timestamp to the current timestamp.
	 */
	void reset();

	/**
	 * Returns true if the time passed since the start timestamp is larger than
	 * MaxTime.
	 */
	bool triggered();
};

/**
 * The SpiDriverLinux implements the SpiDriverGenericTransceiver interface to
 * provide a communication channel with the si443x device.
 */
class SpiDriverLinux
    : public SpiDriverGenericTransceiver<SpiDriverLinux, ClockTimeoutProvider> {
	// Be friends with the parent class to allow the curiously recurring
	// template pattern to work.
	friend SpiDriverGenericTransceiver<SpiDriverLinux, ClockTimeoutProvider>;

private:
	/**
	 * File descriptor of the SPI device or a negative number if the device
	 * could not be opened. In this case all functions will return ERR_IO.
	 */
	int fd;

	/**
	 * Function implementing a raw SPI transmission.
	 */
	Status transceiveImpl(uint8_t nBytes, uint8_t *buf);

public:
	/**
	 * Creates a new SpiDriverLinux instance and opens the corresponding spi
	 * device.
	 *
	 * @param deviceName is the name of the associated SPI device file (usually
	 * /dev/spidevB.C where B is the bus-id and C is the channel-id).
	 * @param speed is the bus-speed in Hz. Default value is 1 MHz.
	 */
	SpiDriverLinux(const char *deviceName, int speed = 500000);

	/**
	 * Destroys the SpiDriverLinux instance and closes the corresponding spi
	 * device if it had been opened.
	 */
	~SpiDriverLinux();

	/**
	 * Returns true if the Spi device has been successfully opened.
	 */
	bool isOpen() {
		return (fd >= 0);
	}
};
}

#endif /* _SI443X_SPI_DRIVER_LINUX_HPP_ */

