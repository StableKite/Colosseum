// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_ArduCopterSolo_hpp
#define msr_airlib_vehicles_ArduCopterSolo_hpp

#include "vehicles/multirotor/firmwares/mavlink/ArduCopterSoloApi.hpp"

namespace msr
{
namespace airlib
{

    class ArduCopterSoloParams : public MultiRotorParams
    {

    public:
        ArduCopterSoloParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_settings, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_settings);
        }

        virtual ~ArduCopterSoloParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            unique_ptr<MultirotorApiBase> api(new ArduCopterSoloApi());
            auto api_ptr = static_cast<ArduCopterSoloApi*>(api.get());
            api_ptr->initialize(connection_info_, &getSensors(), true);

            return api;
        }

        virtual void setupParams() override
        {
            // TODO consider whether we want to make us of the 'connection_info_.model' field (perhaps to indicate different sensor configs, say?  not sure...)

            auto& params = getParams();

            const auto& physics_settings = AirSimSettings::singleton().physics_settings; // Get a singleton

            setupFrame(params, physics_settings.multirotor);
        }

    private:
        static const AirSimSettings::MavLinkConnectionInfo getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
        {
            AirSimSettings::MavLinkConnectionInfo result = vehicle_setting.connection_info;
            return result;
        }

    protected:
        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
