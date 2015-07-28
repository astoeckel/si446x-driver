#  si443x-driver -- Driver for the Silabs Si443x wireless transceiver IC
#  Copyright (C) 2015 Andreas Stöckel
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.0)
project(si443x-driver)

add_definitions(-std=c++11)
add_definitions(-Wall)

include_directories(src/)

add_executable(main
	src/platform/gpio_driver_generic
	src/platform/gpio_driver_rpi
	src/platform/delay_driver_generic
	src/platform/delay_driver_linux
	src/platform/spi_driver_generic
	src/platform/spi_driver_linux
	src/main
	src/si443x
	src/si443x_types
)

target_link_libraries(main
	wiringPi
)

