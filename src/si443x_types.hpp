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
	ERR_TIMEOUT = -3,

	/**
     * Returned if a value is out of range.
     */
	ERR_OUT_OF_RANGE = -4
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

enum class NirqMode : uint8_t {
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

enum class SdoMode : uint8_t {
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

enum class GpioDriveStrength : uint8_t {
	HIGH = 0x00,
	MED_HIGH = 0x20,
	MED_LOW = 0x40,
	LOW = 0x60
};

enum class TxDirectModeType : uint8_t {
	SYNC = 0 << 7,
	ASYNC = 1 << 7
};

enum class TxDirectModeGpio : uint8_t {
	GPIO0 = 0 << 5,
	GPIO1 = 1 << 5,
	GPIO2 = 2 << 5,
	GPIO3 = 3 << 5
};

enum class ModSource : uint8_t {
	PACKET = 0 << 3,
	DIRECT = 1 << 3,
	PSEUDO = 2 << 3
};

enum class ModType : uint8_t {
	CW = 0 << 0,
	OOK = 1 << 0,
	_2FSK = 2 << 0,
	_2GFSK = 3 << 0,
	_4FSK = 4 << 0,
	_4GFSK = 5 << 0
};

enum class State : uint8_t {
	NOCHANGE = 0,
	SLEEP = 1,
	SPI_ACTIVE = 2,
	READY = 3,
	TX_TUNE = 5,
	RX_TUNE = 6,
	TX = 7,
	RX = 8
};

#pragma pack(push, 1)

struct uint24 {
	uint8_t a1;
	uint8_t a2;
	uint8_t a3;

	uint24() {}
	constexpr uint24(uint32_t v)
	    : a1((v & 0xFF0000) >> 16),
	      a2((v & 0x00FF00) >> 8),
	      a3((v & 0x0000FF) >> 0)
	{
	}
	constexpr uint24(uint8_t a1, uint8_t a2, uint8_t a3)
	    : a1(a1), a2(a2), a3(a3)
	{
	}

	uint32_t asInt() { return (a1 << 16) | (a2 << 8) | a3; }
};

template <size_t Size>
struct PropertyDescr {
	static constexpr size_t size = Size;
	uint8_t group;
	uint8_t id;
	constexpr PropertyDescr(uint8_t group, uint8_t id) : group(group), id(id) {}
	constexpr PropertyDescr(uint16_t v)
	    : group((v & 0xFF00) >> 8), id((v & 0x00FF) >> 0)
	{
	}
};

struct Property8 : public PropertyDescr<1> {
	using PropertyDescr<1>::PropertyDescr;
};
struct Property16 : public PropertyDescr<2> {
	using PropertyDescr<2>::PropertyDescr;
};
struct Property24 : public PropertyDescr<3> {
	using PropertyDescr<3>::PropertyDescr;
};
struct Property32 : public PropertyDescr<4> {
	using PropertyDescr<4>::PropertyDescr;
};

namespace Property {
constexpr Property8 GLOBAL_XO_TUNE = 0x0000;
constexpr Property8 GLOBAL_CLK_CFG = 0x0001;
constexpr Property8 GLOBAL_LOW_BATT_THRESH = 0x0002;
constexpr Property8 GLOBAL_CONFIG = 0x0003;
constexpr Property8 GLOBAL_WUT_CONFIG = 0x0004;
constexpr Property16 GLOBAL_WUT_M = 0x0005;
constexpr Property8 GLOBAL_WUT_R = 0x0007;
constexpr Property8 GLOBAL_WUT_LDC = 0x0008;
constexpr Property8 GLOBAL_WUT_CAL = 0x0009;

constexpr Property8 INT_CTL_ENABLE = 0x0100;
constexpr Property8 INT_CTL_PH_ENABLE = 0x0101;
constexpr Property8 INT_CTL_MODEM_ENABLE = 0x0102;
constexpr Property8 INT_CTL_CHIP_ENABLE = 0x0103;

constexpr Property8 FRR_CTL_A_MODE = 0x02000;
constexpr Property8 FRR_CTL_B_MODE = 0x02001;
constexpr Property8 FRR_CTL_C_MODE = 0x02002;
constexpr Property8 FRR_CTL_D_MODE = 0x02003;

constexpr Property8 PREAMBLE_TX_LENGTH = 0x1000;
constexpr Property8 PREAMBLE_CONFIG_STD_1 = 0x1001;
constexpr Property8 PREAMBLE_CONFIG_NSTD = 0x1002;
constexpr Property8 PREAMBLE_CONFIG_STD_2 = 0x1003;
constexpr Property8 PREAMBLE_CONFIG = 0x1004;
constexpr Property32 PREAMBLE_PATTERN = 0x1005;
constexpr Property8 PREAMBLE_POSTAMBLE_CONFIG = 0x1009;
constexpr Property32 PREAMBLE_POSTAMBLE_PATTERN = 0x100a;

constexpr Property8 SYNC_CONFIG = 0x1100;
constexpr Property32 SYNC_BITS = 0x1101;
constexpr Property8 SYNC_CONFIG2 = 0x1105;

constexpr Property8 PKT_CRC_CONFIG = 0x1200;
constexpr Property16 PKT_WHT_POLY = 0x1201;
constexpr Property16 PKT_WHT_SEED = 0x1203;
constexpr Property8 PKT_WHT_BIT_NUM = 0x1205;
constexpr Property8 PKT_CONFIG1 = 0x1206;
constexpr Property8 PKT_CONFIG2 = 0x1207;
constexpr Property8 PKT_LEN = 0x1208;
constexpr Property8 PKT_LEN_FIELD_SOURCE = 0x1209;
constexpr Property8 PKT_LEN_ADJUST = 0x120A;
constexpr Property8 PKT_TX_THRESHOLD = 0x120B;
constexpr Property8 PKT_RX_THRESHOLD = 0x120C;
// TODO Other PKT Stuff

constexpr Property8 MODEM_MOD_TYPE = 0x2000;
constexpr Property8 MODEM_CLKGEN_BAND = 0x2051;

constexpr Property8 FREQ_CONTROL_INTE = 0x4000;
constexpr Property24 FREQ_CONTROL_FRAC = 0x4001;
};

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

template <typename Payload>
struct SetPropertyRaw {
	uint8_t group;
	uint8_t num_props;
	uint8_t start_prop;
	Payload payload;

	SetPropertyRaw() {}
	SetPropertyRaw(uint8_t group, uint8_t num_props, uint8_t start_prop,
	               Payload payload)
	    : group(group),
	      num_props(num_props),
	      start_prop(start_prop),
	      payload(payload)
	{
	}
};

struct GetPropertyRaw {
	uint8_t group;
	uint8_t num_props;
	uint8_t start_prop;

	GetPropertyRaw() {}
	GetPropertyRaw(uint8_t group, uint8_t num_props, uint8_t start_prop)
	    : group(group), num_props(num_props), start_prop(start_prop)
	{
	}
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

static inline constexpr uint32_t endian32(uint32_t v)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	return (v >> 24) | ((v << 8) & 0x00FF0000) | ((v >> 8) & 0x0000FF00) |
	       (v << 24);
#else
	return v;
#endif
}

static inline constexpr uint16_t endian(uint16_t v) { return endian16(v); }

static inline constexpr uint32_t endian(uint32_t v) { return endian32(v); }
}

#endif /* _SI443X_TYPES_HPP_ */

