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
 * @file spi_driver_generic.hpp
 *
 * Contains the generic interface for the SPI driver that has to be implemented
 * by the actual platform. The SPI driver is one of the platform-dependent parts
 * of the si443x driver. Makes use of the curiously recurring template pattern
 * to implement function overriding without the overhead of virtual functions.
 *
 * @author Andreas Stöckel
 */

#ifndef _SI443X_SPI_DRIVER_GENERIC_HPP_
#define _SI443X_SPI_DRIVER_GENERIC_HPP_

#include <alloca.h>
#include <stdint.h>

#include <si443x_types.hpp>

namespace si443x {
/**
 * The DefaultTimeoutProvider class triggers a timeout after a certain count of
 * calls to the "triggered()" method. Other implementations of the same
 * interface may use a realtime clock in order to abort after a certain number
 * of iterations.
 */
class DefaultTimeoutProvider {
private:
	// Do not trigger the timeout more than 10000
	static constexpr uint16_t MaxNCalls = 10000;

	uint16_t nCalls = 0;

public:
	/**
	 * Resets the timeout provider.
	 */
	void reset() { nCalls = 0; }

	/**
	 * Returns true if the timeout has triggered.
	 */
	bool triggered() { return nCalls++ >= MaxNCalls; }
};

/**
 * The NullTimeoutProvider class never triggers a timeout.
 */
struct NullTimeoutProvider {
	/**
	 * The NullTimeoutProvider never triggers.
	 */
	bool triggered() { return false; }
};

/**
 * The SpiDriverGeneric class provides a common interface that has to be
 * implemented by the actual platform drivers. Despite its name SpiDriverGeneric
 * does not represent a generic Spi transmitter interface (the
 * SpiDriverGenericTransceiver class provides such an interface). It provides a
 * generic interface specifically for the communication with the si443x ic. This
 * allows the actual implementation to be more memory-efficient.
 *
 * Communication with the si443x device is a two step process: First a
 *
 * Implementers have to implement two functions: "sendImpl()" and
 * "receiveImpl()". See their description for more information.
 *
 * @tparam SpiDriverImpl is the type of the deriving class. SpiDriverGeneric
 * uses the curiously recurring template pattern to allow function overriding
 * without the overhead of virtual function calls and tables.
 * @tparam TimeoutProvider is the type that should be used for tracking
 * timeouts when waiting for the response from the device.
 */
template <typename SpiDriverImpl,
          typename TimeoutProvider = DefaultTimeoutProvider>
class SpiDriverGeneric {
private:
	/**
	 * Sends a new command to the Si443x. The actual method has to be
	 * implemented by a deriving class.
	 *
	 * @param cmd is the command word the should be executed.
	 * @param argsSize is the number of bytes that should be sent as argument.
	 * May be zero in which case "argsBuf" is ignored.
	 * @param argsBuf is a pointer at the buffer containing the arguments. May
	 * be nullptr if argsSize is zero.
	 * @return Status::OK if the command has been executed
	 * successfully and Status::ERR_IO if there has been a general I/O
	 * error.
	 */
	Status sendImpl(uint8_t cmd, uint8_t argsSize,
	                         const uint8_t *argsBuf)
	{
		return static_cast<SpiDriverImpl *>(this)
		    ->sendImpl(cmd, argsSize, argsBuf);
	}

	/**
	 * Tries to read the current response from the Si443x.
	 *
	 * @param resSize is the size of the result buffer. May be zero. In this
	 * case the command may be used to check whether the device is no longer
	 * busy.
	 * @param resBuf is the pointer to which the result should be written.
	 * @return Status::OK if the response has been read,
	 * Status::BUSY if the device is currently busy and
	 * Status::ERR_IO if there has been a general I/O error.
	 */
	Status receiveImpl(uint8_t resSize, uint8_t *resBuf)
	{
		return static_cast<SpiDriverImpl *>(this)->receiveImpl(resSize, resBuf);
	}

public:
	/**
	 * Sends a new command to the Si443x.
	 *
	 * @tparam Args is the type of the arguments passed to the function.
	 * @param cmd is the command word the should be executed.
	 * @param args is a pointer at the structure containing the arguments. May
	 * be nullptr.
	 * @return Status::OK if the command has been executed
	 * successfully and Status::ERR_IO if there has been a general I/O
	 * error.
	 */
	template <typename Args>
	Status send(Command cmd, const Args args = nullptr)
	{
		return sendImpl(static_cast<uint8_t>(cmd),
		                args == nullptr ? 0 : sizeof(Args),
		                (const uint8_t *)(args));
	}

	/**
	 * Tries to read the current response from the Si443x.
	 *
	 * @tparam Result is the type of the structure in which the result should be
	 * stored.
	 * @param cmd is the command word the should be executed.
	 * @param res is a pointer at the structure to which the results should be
	 * written.
	 * @return Status::OK if the response has been read.
	 * Status::BUSY if the device is currently busy and
	 * Status::ERR_IO if there has been a general I/O error.
	 */
	template <typename Result>
	Status receive(Result res = nullptr)
	{
		return receiveImpl(res == nullptr ? 0 : sizeof(Result),
		                   (uint8_t *)(res));
	}

	/**
	 * Tries to read the current response from the Si443x. Uses the given
	 * timeoutProvider in order to abort the operation after a certain time.
	 *
	 * @tparam Result is the type of the structure in which the result should be
	 * stored.
	 * @tparam TimoutProvider is the type of the class causing the receive
	 * function to fail after a certain number of trials/after a certain time
	 * has passed.
	 * @param res is a pointer at the structure to which the results should be
	 * written.
	 * @param timeoutProvider is a reference at the object controlling when
	 * @return Status::OK if the response has been read,
	 * Status::BUSY if the device is currently busy,
	 * Status::ERR_IO if there has been a general I/O error and
	 * Status::ERR_TIMEOUT if the timeout provideder was triggered.
	 */
	template <typename Result, typename LocalTimeoutProvider>
	Status receive(Result res, LocalTimeoutProvider &timeout)
	{
		Status state = receive(res);
		if (state == Status::BUSY && timeout.triggered()) {
			return Status::ERR_TIMEOUT;
		}
		return state;
	}

	/**
	 * Synchronously executes the given command, passes the given arguments and
	 * waits until the result has been written into the result buffer. If
	 * possible this function should not be used, as it unecessarily uses
	 * compute resources while waiting for the device response. Use "send" and
	 * "receive" in a asynchronous manner instead using a state-machine.
	 *
	 * @param cmd is the command that should be executed.
	 * @param args is a pointer at the buffer containing the arguments.
	 * @param res is a pointer at the buffer containing the result.
	 */
	template <typename Args, typename Result>
	Status exec(Command cmd, const Args args = nullptr,
	                     Result res = nullptr)
	{
		// Send the actual command
		Status status = send(cmd, args);
		if (status != Status::OK) {
			return status;
		}

		// Receive the response, abort after a certain time.
		TimeoutProvider timeout;
		while ((status = receive(res, timeout)) == Status::BUSY) {
		}

		return status;
	}

	/**
	 * Waits untils the device is no longer busy or the timout is triggered.
	 * According to the datasheet this function has to be called after a reset.
	 */
	template <typename LocalTimeoutProvider>
	Status waitNotBusy(LocalTimeoutProvider &timeout)
	{
		Status status;
		while ((status = receive(nullptr, timeout)) == Status::BUSY) {
		}
		return status;
	}

	/**
	 * Waits untils the device is no longer busy. According to the datasheet
	 * this function has to be called after a reset.
	 */
	Status waitNotBusy()
	{
		TimeoutProvider timeout;
		return waitNotBusy(timeout);
	}
};

/**
 * The SpiDriverGenericTransceiver class is a convenience class implementing
 * the SpiDriverGeneric interface and leaving a generic spi "transceive" method
 * to be implementeted. However, this function may require an astounding 18 byte
 * of additional stack space and an overhead of up to 16 unnecessary SPI
 * transmissions. Therefore µC drivers should directly implement the
 * SpiDriverGeneric interface if possible.
 *
 * @tparam SpiDriverImpl is the type of the deriving class.
 * SpiDriverGenericTransceiver uses the curiously recurring template pattern to
 * allow function overriding without the overhead of virtual function calls and
 * tables.
 * @tparam TimeoutProvider is the type that should be used for tracking
 * timeouts when waiting for the response from the device, it is passed to the
 * SpiDriverGeneric parent class.
 */
template <typename SpiDriverImpl,
          typename TimeoutProvider = DefaultTimeoutProvider>
class SpiDriverGenericTransceiver
    : public SpiDriverGeneric<
          SpiDriverGenericTransceiver<SpiDriverImpl, TimeoutProvider>,
          TimeoutProvider> {
	// Be firends with the parent class to allow the curiously recurring
	// template
	// pattern to work.
	friend SpiDriverGeneric<
	    SpiDriverGenericTransceiver<SpiDriverImpl, TimeoutProvider>,
	    TimeoutProvider>;

private:
	/**
	 * Generic SPI transceive method to be implemented by the child class.
	 *
	 * @param bufSize is the size of the buffer to be transceived.
	 * @param buf is the buffer from which the data should be read and that will
	 * be overriden by the result.
	 * @return Status::OK if the command has been executed
	 * successfully and Status::ERR_IO if there has been a general I/O
	 * error.
	 */
	Status transceiveImpl(uint8_t bufSize, uint8_t *buf)
	{
		return static_cast<SpiDriverImpl *>(this)->transceiveImpl(bufSize, buf);
	}

	/**
	 * Implementation of the sendImpl method as defined by SpiDriverGeneric.
	 */
	Status sendImpl(uint8_t cmd, uint8_t argsSize,
	                         const uint8_t *argsBuf)
	{
		uint8_t *buf = static_cast<uint8_t *>(alloca(argsSize + 1));
		buf[0] = cmd;
		for (uint8_t i = 0; i < argsSize; i++) {
			buf[i + 1] = argsBuf[i];
		}
		return transceiveImpl(argsSize + 1, buf);
	}

	/**
	 * Implementation of the receiveImpl method as defined by SpiDriverGeneric.
	 */
	Status receiveImpl(uint8_t resSize, uint8_t *resBuf)
	{
		// Assemble the read/write buffer
		uint8_t *buf = static_cast<uint8_t *>(alloca(resSize + 2));
		buf[0] = static_cast<uint8_t>(Command::READ_CMD_BUFF);
		for (int i = 0; i < resSize; i++) {
			buf[i + 2] = 0;
		}

		// Perform the actual SPI communication.
		Status status = transceiveImpl(resSize + 2, buf);

		// If the device wrote 0xFF into the second byte, the response is
		// available. Otherwise, the device is still busy.
		if (status == Status::OK) {
			if (buf[1] == 0xFF) {
				for (uint8_t i = 0; i < resSize; i++) {
					resBuf[i] = buf[i + 2];
				}
				return Status::OK;
			}
			return Status::BUSY;
		}
		return status;
	}
};
}

#endif /* _SI443X_SPI_DRIVER_GENERIC_HPP_ */

