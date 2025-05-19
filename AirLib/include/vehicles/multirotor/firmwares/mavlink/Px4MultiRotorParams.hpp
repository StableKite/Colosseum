// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Px4MultiRotor_hpp
#define msr_airlib_vehicles_Px4MultiRotor_hpp

#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"

namespace msr
{
namespace airlib
{

    class Px4MultiRotorParams : public MultiRotorParams
    {
    public:
        Px4MultiRotorParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_setting);
        }

        virtual ~Px4MultiRotorParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            unique_ptr<MultirotorApiBase> api(new MavLinkMultirotorApi());
            auto api_ptr = static_cast<MavLinkMultirotorApi*>(api.get());
            api_ptr->initialize(connection_info_, &getSensors(), true);

            return api;
        }

        virtual void setupParams() override
        {
            auto& params = getParams();

            auto& user_settings = AirSimSettings::singleton().physics_settings.multirotor; // Get a singleton
            AirSimSettings::MultirotorPhysicsSettings preset_defaults;

            // Create a set of valid models
            static const std::unordered_set<std::string> valid_presets = {
                "Blacksheep", "Flamewheel", "FlamewheelFLA", "Hexacopter", "Octocopter"
            };
            
            std::string target_preset = valid_presets.count(connection_info_.model) ? connection_info_.model : "Quadrocopter";

            preset_defaults.applyPresetDefaults(target_preset);

            // Merge settings (user ones take precedence)
            #define MERGE_FIELD(field) if (user_settings.field <= 0) user_settings.field = preset_defaults.field
            #define MERGE_VECTOR(field) if (user_settings.field.empty()) user_settings.field = preset_defaults.field

            MERGE_VECTOR(preset);
            MERGE_FIELD(rotor_count);
            MERGE_FIELD(linear_drag_coefficient);
            MERGE_FIELD(angular_drag_coefficient);
            MERGE_VECTOR(arm_lengths);
            MERGE_VECTOR(arm_angles);
            MERGE_VECTOR(rotor_directions);
            MERGE_FIELD(mass);
            MERGE_FIELD(motor_assembly_weight);
            MERGE_VECTOR(body_box);
            MERGE_FIELD(rotor_z);
        
            MERGE_FIELD(rotor_params.C_T);
            MERGE_FIELD(rotor_params.C_P);
            MERGE_FIELD(rotor_params.max_rpm);

            setupFrame(params, user_settings);
        }

    protected:
        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
        {
            return vehicle_setting.connection_info;
        }

    private:
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
