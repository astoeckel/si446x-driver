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

#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "spi_driver_linux.hpp"

namespace si443x {

/* Class ClockTimeoutProvider */

static uint64_t time_milliseconds()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return ts.tv_sec * 1000 + (ts.tv_nsec / (1000 * 1000));
}

ClockTimeoutProvider::ClockTimeoutProvider() { reset(); }

void ClockTimeoutProvider::reset() { startTs = time_milliseconds(); }

bool ClockTimeoutProvider::triggered()
{
	return (time_milliseconds() - startTs) > MaxTime;
}

/* Class SpiDriverLinux */

Status SpiDriverLinux::transceiveImpl(uint8_t nBytes, uint8_t *buf)
{
	// Make sure the file descriptor is valid
	if (fd < 0) {
		return Status::ERR_IO;
	}

	// Perform a receive and transmit at the same time, use the same buffer
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	xfer.tx_buf = reinterpret_cast<intptr_t>(buf);
	xfer.rx_buf = reinterpret_cast<intptr_t>(buf);
	xfer.len = nBytes;

	// Perform the request
	int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		return Status::ERR_IO;
	}
	return Status::OK;
}

#define ERR(CMD)              \
	if (!ok || ((CMD) < 0)) { \
		if (ok) {             \
			ok = false;       \
			perror(#CMD);     \
		}                     \
	}

SpiDriverLinux::SpiDriverLinux(const char *deviceName, int speed)
{
	// SPI Properties
	uint32_t mode = 0;
	uint8_t lsb_first = 0;
	uint8_t bits = 8;
	uint32_t freq = speed;

	// Try to open the SPI device and set its parameters
	bool ok = true;
	ERR(fd = open(deviceName, O_RDWR));
	ERR(ioctl(fd, SPI_IOC_WR_MODE32, &mode));
	ERR(ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first));
	ERR(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits));
	ERR(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq));

	// Error handling
	if (!ok && fd >= 0) {
		close(fd);
		fd = -1;
	}
}

SpiDriverLinux::~SpiDriverLinux()
{
	if (fd >= 0) {
		close(fd);
	}
}
}

