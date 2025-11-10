/*
* Device Settings BLE Service
 *
 * Provides BLE interface for device configuration with persistent storage.
 * Supports JSON format and partial updates.
 *
 * BLE Device Settings Service (0xFF20):
 * - 0xFF21: Configuration Data (READ/WRITE) - JSON format, auto-saves to NVS
 * - 0xFF22: Settings State (READ/NOTIFY) - Status flags (calibration enabled, etc.)
 * - 0xFF23: Settings Control Point (WRITE) - Commands (factory reset, reboot)
 */

#ifndef DEVICE_SETTINGS_SVC_HPP_
#define DEVICE_SETTINGS_SVC_HPP_

#include "../../lib/blex/examples/imu-streamer/device_settings.h"
#include "../log.h"
#include <cstring>

template<typename Blex>
struct DeviceSettingsServiceImpl {
    // Clean alias - use this for all callback definitions
    using CharCallbacks = typename Blex::template CharacteristicCallbacks<>;

    static constexpr char config_desc_text[] = "Configuration data (JSON, supports partial updates, auto-saves)";
    static constexpr char state_desc_text[] = "Settings state (Bit 0: apply calibration to stream)";
    static constexpr char control_desc_text[] = "Control point (0x01=factory reset, 0x02=reboot)";

    // Descriptors
    using ConfigDesc = blex_standard::descriptors::UserDescription<config_desc_text>;
    using StateDesc = blex_standard::descriptors::UserDescription<state_desc_text>;
    using ControlDesc = blex_standard::descriptors::UserDescription<control_desc_text>;

    // Callback handlers
    static void onBleReadConfig(std::string& outJson) {
        char buf[512];
        const size_t len = DeviceSettings::get().to_json(buf, sizeof(buf));
        outJson.assign(buf, len);
    }

    static void onBleWriteConfig(const std::string& json) {
        if (!DeviceSettings::modify().merge_json(json.c_str()).commit(true)) {
            BLIM_LOG_ERROR("[BLE] Failed to commit device settings changes\n");
            return;
        }

        // Always notify of the actual current state
        StateChar::setValue(DeviceSettings::get().is_calibration_enabled() ? 0x01 : 0x00);
        BLIM_LOG_DONE("[BLE] Device settings has been updated\n");
    }

    static void onBleReadState(uint8_t& outState) {
        outState = DeviceSettings::get().is_calibration_enabled() ? 0x01 : 0x00;
    }

    static void onBleWriteState(const uint8_t& newState) {
        if (const bool apply = (newState & 0x01) != 0;
            !DeviceSettings::modify().set_apply_calibration(apply).commit(true)) {
            BLIM_LOG_ERROR("Failed to commit state changes\n");
        }
        // Always notify with the actual current state
        StateChar::setValue(DeviceSettings::get().is_calibration_enabled() ? 0x01 : 0x00);
    }

    static void onBleWriteControlPoint(const uint8_t& cmd) {
        switch (cmd) {
            case 0x01:
                if (!DeviceSettings::modify().reset(true).commit(true)) {
                    BLIM_LOG_ERROR("Failed to commit factory reset\n");
                }
                // Always notify of the actual current state
                StateChar::setValue(DeviceSettings::get().is_calibration_enabled() ? 0x01 : 0x00);
                break;
            case 0x02:
                ESP.restart();
                break;
            default:
                BLIM_LOG_ERROR("Unknown control cmd: 0x%02X\n", cmd);
                break;
        }
    }

    // Callback type aliases (uses CharCallbacks for cleaner syntax)
    using ConfigCallbacks = typename CharCallbacks::template WithOnRead<onBleReadConfig>::template WithOnWrite<onBleWriteConfig>;
    using StateCallbacks = typename CharCallbacks::template WithOnRead<onBleReadState>::template WithOnWrite<onBleWriteState>;
    using ControlCallbacks = typename CharCallbacks::template WithOnWrite<onBleWriteControlPoint>;

    // Characteristics - write operations require authenticated pairing
    using ConfigChar = typename Blex::template Characteristic<
        std::string, 0xFF21, typename Blex::template Permissions<>::AllowRead::AllowAuthenticatedWrite,
        ConfigCallbacks,
        ConfigDesc
    >;

    struct StateChar : Blex::template Characteristic<
        uint8_t, 0xFF22, typename Blex::template Permissions<>::AllowRead::AllowAuthenticatedWrite::AllowNotify,
        StateCallbacks,
        StateDesc
    > {};

    using ControlChar = typename Blex::template Characteristic<
        uint8_t, 0xFF23, typename Blex::template Permissions<>::AllowAuthenticatedWrite,
        ControlCallbacks,
        ControlDesc
    >;
};


// Device Settings Service Template (policy-agnostic)
// Instantiate with your chosen Blex policy: DeviceSettingsService_<blim>
template<typename Blex, typename C = DeviceSettingsServiceImpl<Blex> >
struct DeviceSettingsService : C, Blex::template Service<
        0xFF20,
        typename C::ConfigChar, typename C::StateChar, typename C::ControlChar
    >{
};

#endif //DEVICE_SETTINGS_SVC_HPP_
