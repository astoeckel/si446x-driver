#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <platform/delay_driver_linux.hpp>
#include <platform/gpio_driver_rpi.hpp>
#include <platform/spi_driver_linux.hpp>

#include <si443x.hpp>
#include <si443x_types.hpp>

using namespace si443x;

bool handleErr(Status res)
{
	switch (res) {
		case Status::ERR_IO:
			printf("I/O Error");
			exit(1);
			break;
		case Status::ERR_TIMEOUT:
			printf("Timeout");
			exit(1);
			break;
		case Status::ERR_OUT_OF_RANGE:
			printf("Out of range!");
			exit(1);
			break;
		case Status::BUSY:
			return false;
		case Status::OK:
			return true;
	}
	return false;
}

int main(void)
{
	// Setup GPIO
	printf("Init GPIO...\n");
	GpioDriverRpi gpio(25, 24, 23);

	// Setup WiringPi
	printf("Opening SPI...\n");
	SpiDriverLinux spi("/dev/spidev0.0");
	if (!spi.isOpen()) {
		printf("Error opening /dev/spidev0.0");
		return 1;
	}

	// Init the driver
	DelayDriverLinux delay;
	Si443x<SpiDriverLinux, GpioDriverRpi, DelayDriverLinux> radio(spi, gpio,
	                                                              delay);

	// Reset the device
	printf("Reset cycle...\n");
	handleErr(radio.reset());

	// Boot the device
	printf("Booting...\n");
	handleErr(radio.powerup());
	printf("Done!\n");

	printf("Read Part Info...\n");
	PartInfo info;
	handleErr(radio.partInfo(info));
	printf("chiprev %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
	       info.chiprev, info.part, info.pbuild, info.id, info.customer,
	       info.romid);

	// Configure GPIO
	handleErr(
	    radio.gpioConfig(GpioConfig()
	                         .gpio<0>(GpioMode::DRIVE0, true)  // TX-EN
	                         .gpio<1>(GpioMode::DRIVE1, true)  // RX-EN
	                         .gpio<2>(GpioMode::INPUT, false)  // NC
	                         .gpio<3>(GpioMode::INPUT, false)  // Input data
	                         .strength(GpioDriveStrength::LOW)));

	GpioConfig gconf;
	handleErr(radio.gpioConfig(gconf));
	printf("gpio0: %d, gpio1: %d, strength: %d\n",
	       static_cast<uint8_t>(gconf.gpio<0>()),
	       static_cast<uint8_t>(gconf.gpio<1>()),
	       static_cast<uint8_t>(gconf.strength()) >> 5);

	printf("gpio0: %d, gpio1: %d\n", static_cast<uint8_t>(gconf.gpioVal<0>()),
	       static_cast<uint8_t>(gconf.gpioVal<1>()));

	// Configure the RF frequency to 433.92 MHz
	handleErr(radio.setFrequency(433.92));

	// Set the modulation type
	handleErr(radio.setModulation(ModType::OOK, ModSource::DIRECT,
	                              TxDirectModeGpio::GPIO3,
	                              TxDirectModeType::ASYNC));

	// Start TX
	handleErr(radio.changeState(State::TX));

	return 0;
}
