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
 * @file delay_driver_linux.hpp
 *
 * Contains the linux implementation of the delay interface.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_DELAY_DRIVER_LINUX_HPP_
#define _SI443X_DELAY_DRIVER_LINUX_HPP_

#include "delay_driver_generic.hpp"

namespace si443x {
class DelayDriverLinux : public DelayDriverGeneric<DelayDriverLinux> {
	friend DelayDriverGeneric<DelayDriverLinux>;

private:
	void delayMsImpl(uint16_t ms);
	void delayUsImpl(uint16_t us);
};
}

#endif /* _SI443X_DELAY_DRIVER_LINUX_HPP_ */

