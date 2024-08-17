#include "mecanumbot_hardware/nanolib_helper.hpp"
#include "nanotec/nano_lib_hw_strings.hpp"

#include <iostream>


NanoLibHelper::NanoLibHelper()
: nanolibAccessor(getNanoLibAccessor())
{
}

NanoLibHelper::~NanoLibHelper() {
	// Do not try to delete pointer to the accessor
}

void NanoLibHelper::checkResult(const char *fault, const nlc::Result &result) {
	if (result.hasError()) {
		std::string errorDesc(fault);

		errorDesc += " failed with error: ";
		errorDesc += result.getError();
		throw nanolib_exception(errorDesc);
	}
}

std::vector<nlc::BusHardwareId> NanoLibHelper::getBusHardware() const {
	return checkedResult("listAvailableBusHardware", nanolibAccessor->listAvailableBusHardware()).getResult();
}

nlc::BusHardwareOptions
NanoLibHelper::createBusHardwareOptions(const nlc::BusHardwareId &busHardwareId) const {
	// create new bus hardware options
	nlc::BusHardwareOptions busHwOptions;

	// now add all options necessary for opening the bus hardware
	if (busHardwareId.getProtocol() == nlc::BUS_HARDWARE_ID_PROTOCOL_CANOPEN) {
		// in case of CAN bus it is the baud rate
		busHwOptions.addOption(nlc::busHwOptionsDefaults.canBus.BAUD_RATE_OPTIONS_NAME,
							   nlc::busHwOptionsDefaults.canBus.baudRate.BAUD_RATE_1000K);

		if (busHardwareId.getBusHardware() == nlc::BUS_HARDWARE_ID_IXXAT) {
			// in case of HMS IXXAT we need also bus number
			busHwOptions.addOption(
				nlc::busHwOptionsDefaults.canBus.ixxat.ADAPTER_BUS_NUMBER_OPTIONS_NAME,
				nlc::busHwOptionsDefaults.canBus.ixxat.adapterBusNumber.BUS_NUMBER_0_DEFAULT);
		}
	} else if (busHardwareId.getProtocol() == nlc::BUS_HARDWARE_ID_PROTOCOL_MODBUS_RTU) {
		// in case of Modbus RTU it is the serial baud rate
		busHwOptions.addOption(nlc::busHwOptionsDefaults.serial.BAUD_RATE_OPTIONS_NAME,
							   nlc::busHwOptionsDefaults.serial.baudRate.BAUD_RATE_19200);
		// and serial parity
		busHwOptions.addOption(nlc::busHwOptionsDefaults.serial.PARITY_OPTIONS_NAME,
							   nlc::busHwOptionsDefaults.serial.parity.EVEN);
	} else {
	}

	return busHwOptions;
}

void NanoLibHelper::openBusHardware(const nlc::BusHardwareId &busHwId,
									const nlc::BusHardwareOptions &busHwOptions) const {
	checkResult("openBusHardwareWithProtocol",
				nanolibAccessor->openBusHardwareWithProtocol(busHwId, busHwOptions));
}

void NanoLibHelper::closeBusHardware(const nlc::BusHardwareId &busHwId) const {
	checkResult("closeBusHardware", nanolibAccessor->closeBusHardware(busHwId));
}

class ScanBusCallback : public nlc::NlcScanBusCallback {
public:
	nlc::ResultVoid callback(nlc::BusScanInfo info, std::vector<nlc::DeviceId> const &devicesFound,
							 int32_t data) {
		(void)devicesFound;
		switch (info) {
		case nlc::BusScanInfo::Start:
			std::cout << "Scan started." << std::endl;
			break;

		case nlc::BusScanInfo::Progress:
			if ((data & 1) == 0) // data holds scan progress
			{
				std::cout << ".";
			}
			break;

		case nlc::BusScanInfo::Finished:
			std::cout << std::endl << "Scan finished." << std::endl;
			break;

		default:
			break;
		}

		return nlc::ResultVoid();
	}
};

std::vector<nlc::DeviceId> NanoLibHelper::scanBus(const nlc::BusHardwareId &busHwId) const {
	ScanBusCallback scanBusCallback;
	return checkedResult("scanDevices", nanolibAccessor->scanDevices(busHwId, &scanBusCallback)).getResult();
}

nlc::DeviceHandle NanoLibHelper::addDevice(const nlc::DeviceId &deviceId) const {
	return checkedResult("addDevice", nanolibAccessor->addDevice(deviceId)).getResult();
}

void NanoLibHelper::connectDevice(const nlc::DeviceHandle &deviceId) const {
	checkResult("connectDevice", nanolibAccessor->connectDevice(deviceId));
}

void NanoLibHelper::disconnectDevice(const nlc::DeviceHandle &deviceId) const {
	checkResult("disconnectDevice", nanolibAccessor->disconnectDevice(deviceId));
}

int64_t NanoLibHelper::readInteger(const nlc::DeviceHandle &deviceId,
								   const nlc::OdIndex &odIndex) const {
	return checkedResult("readNumber", nanolibAccessor->readNumber(deviceId, odIndex)).getResult();
}

void NanoLibHelper::writeInteger(const nlc::DeviceHandle &deviceId, int64_t value,
								 const nlc::OdIndex &odIndex, unsigned int bitLength) const {
	checkResult("writeNumber", nanolibAccessor->writeNumber(deviceId, value, odIndex, bitLength));
}

void NanoLibHelper::writeBytes(const nlc::DeviceHandle deviceHandle,
							   const std::vector<uint8_t> &data,
							   const nlc::OdIndex odIndex) const {
	checkResult("writeBytes", nanolibAccessor->writeBytes(deviceHandle, data, odIndex));
}

std::vector<std::int64_t> NanoLibHelper::readArray(const nlc::DeviceHandle &deviceId,
												   const uint16_t odIndex) const {
	return checkedResult("readNumberArray", nanolibAccessor->readNumberArray(deviceId, odIndex)).getResult();
}

std::string NanoLibHelper::readString(const nlc::DeviceHandle &deviceId,
									  const nlc::OdIndex &odIndex) const {
	return checkedResult("readString", nanolibAccessor->readString(deviceId, odIndex)).getResult();
}

void NanoLibHelper::setLoggingLevel(nlc::LogLevel logLevel) {
	nanolibAccessor->setLoggingLevel(logLevel);
}

void NanoLibHelper::configureSampler(const nlc::DeviceHandle deviceHandle,
									 const nlc::SamplerConfiguration &samplerConfiguration) {
	checkResult("SamplerInterface::configure", nanolibAccessor->getSamplerInterface().configure(deviceHandle, samplerConfiguration));
}

void NanoLibHelper::startSampler(const nlc::DeviceHandle deviceHandle,
								 nlc::SamplerNotify *samplerNotify, int64_t applicationData) {
	checkResult("SamplerInterface::start", nanolibAccessor->getSamplerInterface().start(deviceHandle, samplerNotify, applicationData));
}

void NanoLibHelper::stopSampler(const nlc::DeviceHandle deviceHandle) {
	const auto result = nanolibAccessor->getSamplerInterface().stop(deviceHandle);

	if (result.hasError() && (result.getErrorCode() != nlc::NlcErrorCode::InvalidOperation))
	checkResult("SamplerInterface::stop", result);
}

nlc::SamplerState NanoLibHelper::getSamplerState(const nlc::DeviceHandle deviceHandle) {
	return checkedResult("SamplerInterface::getState",
						 nanolibAccessor->getSamplerInterface().getState(deviceHandle))
		.getResult();
}

std::vector<nlc::SampleData> NanoLibHelper::getSamplerData(const nlc::DeviceHandle deviceHandle) {
	return checkedResult("SamplerInterface::getData",
						 nanolibAccessor->getSamplerInterface().getData(deviceHandle))
		.getResult();
}

nlc::ResultVoid NanoLibHelper::getSamplerLastError(const nlc::DeviceHandle deviceHandle) {
	return nanolibAccessor->getSamplerInterface().getLastError(deviceHandle);
}