// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_SimpleFlightQuadX_hpp
#define msr_airlib_vehicles_SimpleFlightQuadX_hpp

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class SimpleFlightQuadXParams : public MultiRotorParams
    {
    public:
        SimpleFlightQuadXParams(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : vehicle_setting_(vehicle_setting), sensor_factory_(sensor_factory)
        {
        }

        virtual ~SimpleFlightQuadXParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            return std::unique_ptr<MultirotorApiBase>(new SimpleFlightApi(this, vehicle_setting_));
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();

            const auto& physics_settings = AirSimSettings::singleton().physics_settings; // Get a singleton

            setupFrame(params, physics_settings.multirotor);
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        const AirSimSettings::VehicleSetting* vehicle_setting_; //store as pointer because of derived classes
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
