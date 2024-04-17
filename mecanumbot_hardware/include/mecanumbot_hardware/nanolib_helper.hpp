#pragma once

#include <nanotec/accessor_factory.hpp>

class nanolib_exception : public std::exception {
public:
	nanolib_exception(const std::string& msg) : message(msg) {
	}

	virtual char const *what() const noexcept {
		return message.c_str();
	}

private:
	std::string message;
};

class NanoLibHelper {
public:
	NanoLibHelper();
	virtual ~NanoLibHelper();

	/**
	 * @brief Checks the result and throws an exception if unsuccessful.
	 * 
	 * @param fault  - context description 
	 * @param result - the result from the operation
	 */
	static void checkResult(const char* fault, const nlc::Result &result);

	/**
	 * @brief Checks the result and throws an exception if unsuccessful.
	 *
	 * @param fault  - context description
	 * @param result - the result from the operation
	 * 
	 * @return The result of the operation 
	 */
	template <class ResultClass>
	static ResultClass checkedResult(const char *fault, const ResultClass &result) {
		checkResult(fault, result);
		return result;
	}

	nlc::NanoLibAccessor *operator->() const noexcept {
		return nanolibAccessor;
	}

	/**
	 * @brief Get a list of available bus hardware
	 *
	 * @return std::vector<nlc::BusHardwareId>
	 */
	std::vector<nlc::BusHardwareId> getBusHardware() const;

	/**
	 * @brief Create bus hardware options object
	 *
	 * @return nlc::BusHardwareOptions A set of options for opening the bus hardware
	 */
	nlc::BusHardwareOptions createBusHardwareOptions(const nlc::BusHardwareId &busHardwareId) const;

	/**
	 * @brief Opens the bus hardware with given id and options
	 *
	 * @param busHwId The id of the bus hardware taken from NanoLibHelper.getBusHardware()
	 * @param busHwOptions The hardware options taken from
	 * NanoLibHelper.createBusHardwareOptions()
	 */
	void openBusHardware(const nlc::BusHardwareId &busHwId,
						 const nlc::BusHardwareOptions &busHwOptions) const;

	/**
	 * @brief Closes the bus hardware (access no longer possible after that)
	 *
	 * Note: the call of the function is optional because the nanolib will cleanup the
	 * bus hardware itself on closing.
	 *
	 * @param busHwId The bus hardware id to close
	 */
	void closeBusHardware(const nlc::BusHardwareId &busHwId) const;

	/**
	 * @brief Scans bus and returns all found device ids.
	 *
	 * CAUTION: open bus hardware first with NanoLibHelper.openBusHardware()
	 *
	 * Note: this functionality is not available on all bus hardwares. It is assumed that
	 * this example runs with CANopen where the scan is possible.
	 *
	 * @param busHwId The bus hardware to scan
	 * @return std::vector<nlc::DeviceId> Vector with found devices
	 */
	std::vector<nlc::DeviceId> scanBus(const nlc::BusHardwareId &busHwId) const;

	/**
	 * @brief Registers the device id into NanoLib internal list
	 *
	 * @param deviceId The DeviceId of the device
	 * @return Handle to registered device.
	 */
	nlc::DeviceHandle addDevice(const nlc::DeviceId &deviceId) const;

	/**
	 * @brief Connects to given device id
	 *
	 * @param deviceId The device id to connect to
	 */
	void connectDevice(const nlc::DeviceHandle &deviceId) const;

	/**
	 * @brief Disconnects given device
	 *
	 * Note: the call of the function is optional because the nanolib will cleanup the
	 * devices on bus itself on closing.
	 *
	 * @param deviceId
	 */
	void disconnectDevice(const nlc::DeviceHandle &deviceId) const;

	/**
	 * @brief Reads out an integer of given device
	 *
	 * Note: the interpretation of the data type is up to the user. Signed integer
	 * are interpreted as unsigned integer.
	 *
	 * @param deviceId The id of the device to read from
	 * @param odIndex The index and sub-index of the object dictionary to read from
	 *
	 * @return int
	 */
	int64_t readInteger(const nlc::DeviceHandle &deviceId, const nlc::OdIndex &odIndex) const;

	/**
	 * @brief Writes given value to the device
	 *
	 * @param deviceId The id of the device to write to
	 * @param value The value to write to the device
	 * @param odIndex The index and sub-index of the object dictionary to write to
	 * @param bitLength The bit length of the object to write to, either 8, 16 to 32
	 * 		(see manual for all the bit lengths of all objects)
	 */
	void writeInteger(const nlc::DeviceHandle &deviceId, int64_t value, const nlc::OdIndex &odIndex,
					  unsigned int bitLength) const;

	/**
	 * @brief Reads out a od object array
	 *
	 * Note: the interpretation of the data type is up to the user. Signed integer
	 * are interpreted as unsigned integer.
	 *
	 * @param deviceId The id of the device to read from
	 * @param odIndex The index and sub-index of the object dictionary to read from
	 *
	 * @return std::vector<std::uint32_t>
	 */
	std::vector<std::int64_t> readArray(const nlc::DeviceHandle &deviceId,
										const uint16_t odIndex) const;

	/**
	 * @brief Reads out string from device
	 *
	 * @param deviceId The id of the device to read from
	 * @param odIndex The index and sub-index of the object dictionary to read from
	 *
	 * @return std::string
	 */
	std::string readString(const nlc::DeviceHandle &deviceId, const nlc::OdIndex &odIndex) const;

	/**
	 * @brief Set the logging level
	 *
	 * @param logLevel
	 */
	void setLoggingLevel(nlc::LogLevel logLevel);

	/**
	 * @brief Configure a sampler
	 */
	void configureSampler(const nlc::DeviceHandle deviceHandle,
						  const nlc::SamplerConfiguration &samplerConfiguration);

	/**
	 * @brief Starts a sampler
	 */
	void startSampler(const nlc::DeviceHandle deviceHandle, nlc::SamplerNotify *samplerNotify,
					  int64_t applicationData);

	/**
	 * @brief Stops a sampler
	 */
	void stopSampler(const nlc::DeviceHandle deviceHandle);

	/**
	 * @brief Getting the status of a sampler
	 */
	nlc::SamplerState getSamplerState(const nlc::DeviceHandle deviceHandle);

	/**
	 * @brief Getting sampled data
	 */
	std::vector<nlc::SampleData> getSamplerData(const nlc::DeviceHandle deviceHandle);

	/** 
	* @brief Getting sampler last error
	*/
	nlc::ResultVoid getSamplerLastError(const nlc::DeviceHandle deviceHandle);
	
private:
	nlc::NanoLibAccessor *nanolibAccessor;
};