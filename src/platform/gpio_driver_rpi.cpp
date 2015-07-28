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

#include "gpio_driver_rpi.hpp"

#include <wiringPi.h>

namespace si443x {

void GpioDriverRpi::resetImpl(bool high)
{
	if (pinReset >= 0) {
		digitalWrite(pinReset, high);
	}
}

void GpioDriverRpi::txAntImpl(bool high)
{
	if (pinTxAnt >= 0) {
		digitalWrite(pinTxAnt, !high);
	}
}

void GpioDriverRpi::rxAntImpl(bool high)
{
	if (pinRxAnt >= 0) {
		digitalWrite(pinRxAnt, !high);
	}
}

GpioDriverRpi::GpioDriverRpi(int pinReset, int pinTxAnt, int pinRxAnt)
    : pinReset(pinReset), pinTxAnt(pinTxAnt), pinRxAnt(pinRxAnt)
{
	// Initialize WiringPi
	wiringPiSetupGpio();

	// Mark all pins as output and drive them LOW
	if (pinReset >= 0) {
		pinMode(pinReset, OUTPUT);
		reset(false);
	}

	if (pinTxAnt >= 0) {
		pinMode(pinTxAnt, OUTPUT);
		txAnt(false);
	}

	if (pinRxAnt >= 0) {
		pinMode(pinRxAnt, OUTPUT);
		rxAnt(false);
	}
}
}

