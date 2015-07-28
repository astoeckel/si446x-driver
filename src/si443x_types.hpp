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
 * @file si443x_types.hpp
 *
 * Contains types, enums and constants used throughout the driver.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_TYPES_HPP_
#define _SI443X_TYPES_HPP_

#include <stdint.h>
#include <stddef.h>

namespace si443x {

/**
 * The SpiDriverResult enum specifies the result of a SpiDriver operation.
 */
enum class Status : int {
	/**
     * The "OK" value indicates that the action has been executed correctly.
     */
	OK = 0,

	/**
     * The "BUSY" value is a possible result of the "readResponse()" function.
     * It indicates that the device is still busy and the previously queued
     * command was not yet executed.
     */
	BUSY = -1,

	/**
     * The "ERR_IO" value indicates that a SPI function could not be executed
     * because of a general I/O error (e.g. a SPI bus handle could not be
     * acquired. Should only occur on linux hosts, on a µC there should be no
     * way a SPI transfer can fail). In this case the program should probably
     * abort.
     */
	ERR_IO = -2,

	/**
     * Returned if a synchronous command fails to finish within a certain time.
     */
	ERR_TIMEOUT = -3
};

/**
 * Enum used for switching the antenna mode (only needed if an external hf
 * antenna switch is used)
 */
enum class AntennaState : uint8_t {
	/**
     * Switches the antenna circuit to RX mode.
     */
	RX,

	/**
     * Switches the antenna circuit to TX mode.
     */
	TX,

	/**
     * Switches the antenna circuit OFF (neither RX or TX are enabled).
     */
	OFF
};

/**
 * Possible firmware commands understood be the SI4436. See corresponding
 * documentation for more information.
 */
enum class Command : uint8_t {
	NOP = 0x00,
	PART_INFO = 0x01,
	POWER_UP = 0x02,
	FUNC_INFO = 0x10,
	SET_PROPERTY = 0x11,
	GET_PROPERTY = 0x12,
	GPIO_PIN_CFG = 0x13,
	FIFO_INFO = 0x15,
	GET_INT_STATUS = 0x20,
	REQUEST_DEVICE_STATE = 0x33,
	CHANGE_STATE = 0x34,
	READ_CMD_BUFF = 0x44,
	FRR_A_READ = 0x50,
	FRR_B_READ = 0x51,
	FRR_C_READ = 0x53,
	FRR_D_READ = 0x57,
	IRCAL = 0x17,
	IRCAL_MANUAL = 0x1A,
	START_TX = 0x31,
	WRITE_TX_FIFO = 0x66,
	PACKET_INFO = 0x16,
	GET_MODEM_STATUS = 0x22,
	START_RX = 0x32,
	RX_HOP = 0x36,
	READ_RX_FIFO = 0x77,
	GET_ADC_READING = 0x14,
	GET_PH_STATUS = 0x21,
	GET_CHIP_STATUS = 0x23
};

enum class GpioPullCtl : uint8_t { PULL_DIS = 0x00, GPIO_PULL_EN = 0x40 };

enum class GpioMode : uint8_t {
	DONOTHING = 0x00,
	TRISTATE = 0x01,
	DRIVE0 = 0x02,
	DRIVE1 = 0x03,
	INPUT = 0x04,
	_32K_CLK = 0x05,
	BOOT_CLK = 0x06,
	DIV_CLK = 0x07,
	CTS = 0x08,
	INV_CTS = 0x09,
	CMD_OVERLAP = 0x0A,
	SDO = 0x0B,
	POR = 0x0C,
	CAL_WUT = 0x0D,
	WUT = 0x0E,
	EN_PA = 0x0F,
	TX_DATA_CLK = 0x10,
	RX_DATA_CLK = 0x11,
	EN_LNA = 0x12,
	TX_DATA = 0x13,
	RX_DATA = 0x14,
	RX_RAW_DATA = 0x15,
	ANTENNA_1_SW = 0x16,
	ANTENNA_2_SW = 0x17,
	VALID_PREAMBLE = 0x18,
	INVALID_PREAMBLE = 0x19,
	SYNC_WORD_DETECT = 0x1A,
	CCA = 0x1B,
	IN_SLEEP = 0x1C,
	TX_STATE = 0x20,
	RX_STATE = 0x21,
	RX_FIFO_FULL = 0x22,
	TX_FIFO_EMPTY = 0x23,
	LOW_BATT = 0x24,
	CCA_LATCH = 0x25,
	HOPPED = 0x26,
	HOP_TABLE_WRAP = 0x27
};

enum class NirqMode {
	DONOTHING = 0x00,
	TRISTATE = 0x01,
	DRIVE0 = 0x02,
	DRIVE1 = 0x03,
	INPUT = 0x04,
	DIV_CLK = 0x07,
	CTS = 0x08,
	SDO = 0x0B,
	POR = 0x0C,
	EN_PA = 0x0F,
	TX_DATA_CLK = 0x10,
	RX_DATA_CLK = 0x11,
	EN_LNA = 0x12,
	TX_DATA = 0x13,
	RX_DATA = 0x14,
	RX_RAW_DATA = 0x15,
	ANTENNA_1_SW = 0x16,
	ANTENNA_2_SW = 0x17,
	VALID_PREAMBLE = 0x18,
	INVALID_PREAMBLE = 0x19,
	SYNC_WORD_DETECT = 0x1A,
	CCA = 0x1B,
	NIRQ = 0x27
};

enum class SdoMode {
	DONOTHING = 0x00,
	TRISTATE = 0x01,
	DRIVE0 = 0x02,
	DRIVE1 = 0x03,
	INPUT = 0x04,
	DIV_CLK = 0x07,
	CTS = 0x08,
	SDO = 0x0B,
	POR = 0x0C,
	EN_PA = 0x0F,
	TX_DATA_CLK = 0x10,
	RX_DATA_CLK = 0x11,
	EN_LNA = 0x12,
	TX_DATA = 0x13,
	RX_DATA = 0x14,
	RX_RAW_DATA = 0x15,
	ANTENNA_1_SW = 0x16,
	ANTENNA_2_SW = 0x17,
	VALID_PREAMBLE = 0x18,
	INVALID_PREAMBLE = 0x19,
	SYNC_WORD_DETECT = 0x1A,
	CCA = 0x1B,
};

enum class GpioDriveStrength {
	HIGH = 0x00,
	MED_HIGH = 0x20,
	MED_LOW = 0x40,
	LOW = 0x60
};

#pragma pack(push, 1)
struct PartInfo {
	uint8_t chiprev;
	uint16_t part;
	uint8_t pbuild;
	uint16_t id;
	uint8_t customer;
	uint8_t romid;
};

struct FuncInfo {
	uint8_t revext;
	uint8_t revbranch;
	uint8_t revint;
	uint16_t reserved0;
	uint8_t func;
};

struct PowerUp {
	uint8_t boot_options;
	uint8_t xtal_options;
	uint32_t xo_freq;
};

struct GpioConfigRaw {
	uint8_t gpio[4];
	uint8_t nirq;
	uint8_t sdo;
	uint8_t gen_config;
};

class GpioConfig {
private:
	GpioMode gpio_[4];
	NirqMode nirq_;
	SdoMode sdo_;
	GpioDriveStrength strength_;
	uint8_t pullups_;
	uint8_t states_;

public:
	GpioConfig()
	{
		for (int i = 0; i < 4; i++) {
			gpio_[i] = GpioMode::DONOTHING;
		}
		nirq_ = NirqMode::DONOTHING;
		sdo_ = SdoMode::DONOTHING;
		strength_ = GpioDriveStrength::LOW;
		pullups_ = 0x00;
		states_ = 0x00;
	}

	GpioConfig(const GpioConfigRaw &raw)
	{
		states_ = 0x00;
		pullups_ = 0x00;
		for (int i = 0; i < 4; i++) {
			gpio_[i] = static_cast<GpioMode>(raw.gpio[i] & 0x1F);
			states_ |= (raw.gpio[i] & 0x80) >> (7 - i);
		}

		nirq_ = static_cast<NirqMode>(raw.nirq & 0x1F);
		states_ |= (raw.nirq & 0x80) >> 3;

		sdo_ = static_cast<SdoMode>(raw.sdo & 0x1F);
		states_ |= (raw.sdo & 0x80) >> 2;

		strength_ = static_cast<GpioDriveStrength>(raw.gen_config & 0x60);
	}

	GpioConfigRaw toRaw() const
	{
		GpioConfigRaw raw;
		for (int i = 0; i < 4; i++) {
			raw.gpio[i] = static_cast<uint8_t>(gpio_[i]) |
			              ((pullups_ & (1 << i)) << (6 - i));
		}
		raw.nirq = static_cast<uint8_t>(nirq_) | ((pullups_ & 0x10) << 2);
		raw.sdo = static_cast<uint8_t>(sdo_) | ((pullups_ & 0x20) << 1);
		raw.gen_config = static_cast<uint8_t>(strength_);
		return raw;
	}

	template <size_t idx>
	GpioConfig &gpio(GpioMode mode, bool pullup = false)
	{
		gpio_[idx] = mode;
		pullups_ |= (pullup ? (1 << idx) : 0x00);
		return *this;
	}

	GpioConfig &nirq(NirqMode mode, bool pullup = false)
	{
		nirq_ = mode;
		pullups_ |= (pullup ? 0x10 : 0x00);
		return *this;
	}

	GpioConfig &sdo(SdoMode mode, bool pullup = false)
	{
		sdo_ = mode;
		pullups_ |= (pullup ? 0x20 : 0x00);
		return *this;
	}

	GpioConfig &strength(GpioDriveStrength s)
	{
		strength_ = s;
		return *this;
	}

	GpioDriveStrength strength() const { return strength_; }

	template <size_t idx>
	GpioMode gpio() const
	{
		return gpio_[idx];
	}

	NirqMode nirq() const { return nirq_; }

	SdoMode sdo() const { return sdo_; }

	template <size_t idx>
	bool gpioVal() const
	{
		return states_ & (1 << idx);
	}

	bool nirqVal() const { return states_ & (1 << 4); }

	bool sdoVal() const { return states_ & (1 << 5); }
};

#pragma pack(pop)


static inline constexpr uint16_t endian16(uint16_t v)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	return (v << 8) | (v >> 8);
#else
	return v;
#endif
}

static inline constexpr uint16_t endian32(uint32_t v)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	return (v >> 24) | ((v << 8) & 0x00FF0000) | ((v >> 8) & 0x0000FF00) |
	       (v << 24);
#else
	return v;
#endif
}

static inline constexpr uint16_t endian(uint16_t v)
{
	return endian16(v);
}

static inline constexpr uint16_t endian(uint32_t v)
{
	return endian32(v);
}

}

#endif /* _SI443X_TYPES_HPP_ */

