/**
 * @file standard.hpp
 * @brief Standard library - Bluetooth SIG standard types and Device Information Service
 *
 * @details
 * Provides standard BLE types, descriptors, and Device Information Service (DIS)
 * characteristics as compile-time constants. All types are policy-free (no lock
 * template parameter) because they're immutable - defined once at compile time.
 *
 * # Standard BLE Enumerations
 * - **GattFormat**: Characteristic Presentation Format field values (UUID 0x2904)
 * - **GattUnit**: Bluetooth SIG assigned unit codes for physical quantities
 * - **BleAppearance**: Device appearance values for category/subcategory indication
 *
 * # Standard Descriptors
 * - **UserDescription**: Human-readable characteristic description (UUID 0x2901)
 * - **PresentationFormat**: Value format, unit, and scaling (UUID 0x2904)
 * - **AggregateFormat**: Groups multiple format descriptors (UUID 0x2905)
 *
 * # Device Information Service (DIS 0x180A)
 * Standard characteristics: ModelNumber, SerialNumber, FirmwareRevision,
 * HardwareRevision, SoftwareRevision, ManufacturerName
 *
 * @note All types are const/immutable - no runtime state, no synchronization needed
 * @see Bluetooth SIG Assigned Numbers: https://www.bluetooth.com/specifications/assigned-numbers/
 */

#ifndef BLEX_STANDARD_HPP_
#define BLEX_STANDARD_HPP_

#include <cstdint>
#include "core.hpp"  // For Permissions

/// @brief Standard BLE types namespace
namespace blex_standard {

// ---------------------- Standard BLE Enums and Types ----------------------

/**
 * @brief Bluetooth Low Energy Characteristic Presentation Format Field values.
 *
 * These values are used in the Characteristic Presentation Format descriptor (UUID 0x2904)
 * to indicate the data type of the characteristic value.
 *
 * Reference: Bluetooth Core Specification Supplement (CSS) Part B, Section 1.3
 * Assigned Numbers: https://www.bluetooth.com/specifications/assigned-numbers/
 */
enum class GattFormat : uint8_t {
    // Unsigned Integers
    kReserved       = 0x00, // Reserved for future use
    kBoolean        = 0x01, // 1-bit boolean
    k2Bit           = 0x02, // 2-bit unsigned integer
    k4Bit           = 0x03, // 4-bit unsigned integer
    kUint8          = 0x04, // 8-bit unsigned integer
    kUint12         = 0x05, // 12-bit unsigned integer
    kUint16         = 0x06, // 16-bit unsigned integer
    kUint24         = 0x07, // 24-bit unsigned integer
    kUint32         = 0x08, // 32-bit unsigned integer
    kUint48         = 0x09, // 48-bit unsigned integer
    kUint64         = 0x0A, // 64-bit unsigned integer
    kUint128        = 0x0B, // 128-bit unsigned integer

    // Signed Integers
    kSint8          = 0x0C, // 8-bit signed integer
    kSint16         = 0x0D, // 16-bit signed integer
    kSint24         = 0x0E, // 24-bit signed integer
    kSint32         = 0x0F, // 32-bit signed integer
    kSint48         = 0x10, // 48-bit signed integer
    kSint64         = 0x11, // 64-bit signed integer
    kSint128        = 0x12, // 128-bit signed integer

    // Floating Point Types
    kFloat32        = 0x13, // IEEE-754 32-bit floating point
    kFloat64        = 0x14, // IEEE-754 64-bit floating point
    kSFloat         = 0x15, // IEEE-11073 16-bit SFLOAT
    kFloat          = 0x16, // IEEE-11073 32-bit FLOAT

    // Other Types
    kDuplicatedUInt16 = 0x17, // IEEE-11073 16-bit Duplicated
    kUtf8String     = 0x18, // UTF-8 string
    kUtf16String    = 0x19, // UTF-16 string
    kStruct         = 0x1A  // Opaque structure
};

/**
 * @brief Bluetooth SIG Assigned Unit UUIDs for GATT Characteristic Presentation Format
 *
 * Used in the Characteristic Presentation Format descriptor (UUID 0x2904) unit field
 * to indicate the physical unit of the characteristic value.
 *
 * Reference: Bluetooth SIG Assigned Numbers - Units
 * https://www.bluetooth.com/specifications/assigned-numbers/
 *
 * Source: Microsoft BluetoothLEExplorer (authoritative implementation)
 * https://github.com/microsoft/BluetoothLEExplorer
 *
 * Note: Unit codes are 16-bit UUID values. Combine with the exponent field for scaling.
 * Example: Tesla (0x272E) with exponent -6 represents microtesla (µT)
 */
enum class GattUnit : uint16_t {
    // Dimensionless
    kUnitless                                   = 0x2700,

    // SI Base Units
    kMetre                                      = 0x2701,  // length
    kKilogram                                   = 0x2702,  // mass
    kSecond                                     = 0x2703,  // time
    kAmpere                                     = 0x2704,  // electric current
    kKelvin                                     = 0x2705,  // thermodynamic temperature
    kMole                                       = 0x2706,  // amount of substance
    kCandela                                    = 0x2707,  // luminous intensity

    // Area
    kSquareMetres                               = 0x2710,

    // Volume
    kCubicMetres                                = 0x2711,

    // Velocity
    kMetrePerSecond                             = 0x2712,

    // Acceleration
    kMetrePerSecondSquared                      = 0x2713,

    // Wavenumber
    kReciprocalMetre                            = 0x2714,

    // Density
    kKilogramPerCubicMetre                      = 0x2715,

    // Surface Density
    kKilogramPerSquareMetre                     = 0x2716,

    // Specific Volume
    kCubicMetrePerKilogram                      = 0x2717,

    // Current Density
    kAmperePerSquareMetre                       = 0x2718,

    // Magnetic Field Strength
    kAmperePerMetre                             = 0x2719,

    // Amount Concentration
    kMolePerCubicMetre                          = 0x271A,

    // Mass Concentration
    kKilogramPerCubicMetre2                     = 0x271B,

    // Luminance
    kCandelaPerSquareMetre                      = 0x271C,

    // Refractive Index
    kRefractiveIndex                            = 0x271D,

    // Relative Permeability
    kRelativePermeability                       = 0x271E,

    // Plane Angle
    kRadian                                     = 0x2720,

    // Solid Angle
    kSteradian                                  = 0x2721,

    // Frequency
    kHertz                                      = 0x2722,

    // Force
    kNewton                                     = 0x2723,

    // Pressure
    kPascal                                     = 0x2724,

    // Energy
    kJoule                                      = 0x2725,

    // Power
    kWatt                                       = 0x2726,

    // Electric Charge
    kCoulomb                                    = 0x2727,

    // Electric Potential Difference
    kVolt                                       = 0x2728,

    // Capacitance
    kFarad                                      = 0x2729,

    // Electric Resistance
    kOhm                                        = 0x272A,

    // Electric Conductance
    kSiemens                                    = 0x272B,

    // Magnetic Flux
    kWeber                                      = 0x272C,

    // Magnetic Flux Density
    kTesla                                      = 0x272D,

    // Inductance
    kHenry                                      = 0x272E,

    // Celsius Temperature
    kDegreeCelsius                              = 0x272F,

    // Luminous Flux
    kLumen                                      = 0x2730,

    // Illuminance
    kLux                                        = 0x2731,

    // Activity Referred To A Radionuclide
    kBecquerel                                  = 0x2732,

    // Absorbed Dose
    kGray                                       = 0x2733,

    // Dose Equivalent
    kSievert                                    = 0x2734,

    // Catalytic Activity
    kKatal                                      = 0x2735,

    // Dynamic Viscosity
    kPascalSecond                               = 0x2740,

    // Moment of Force
    kNewtonMetre                                = 0x2741,

    // Surface Tension
    kNewtonPerMetre                             = 0x2742,

    // Angular Velocity
    kRadianPerSecond                            = 0x2743,

    // Angular Acceleration
    kRadianPerSecondSquared                     = 0x2744,

    // Heat Flux Density, Irradiance
    kWattPerSquareMetre                         = 0x2745,

    // Heat Capacity
    kJoulePerKelvin                             = 0x2746,

    // Specific Heat Capacity
    kJoulePerKilogramKelvin                     = 0x2747,

    // Specific Energy
    kJoulePerKilogram                           = 0x2748,

    // Thermal Conductivity
    kWattPerMetreKelvin                         = 0x2749,

    // Energy Density
    kJoulePerCubicMetre                         = 0x274A,

    // Electric Field Strength
    kVoltPerMetre                               = 0x274B,

    // Electric Charge Density
    kCoulombPerCubicMetre                       = 0x274C,

    // Surface Charge Density
    kCoulombPerSquareMetre                      = 0x274D,

    // Electric Flux Density
    kCoulombPerSquareMetre2                     = 0x274E,

    // Permittivity
    kFaradPerMetre                              = 0x274F,

    // Permeability
    kHenryPerMetre                              = 0x2750,

    // Molar Energy
    kJoulePerMole                               = 0x2751,

    // Molar Entropy
    kJoulePerMoleKelvin                         = 0x2752,

    // Exposure
    kCoulombPerKilogram                         = 0x2753,

    // Absorbed Dose Rate
    kGrayPerSecond                              = 0x2754,

    // Radiant Intensity
    kWattPerSteradian                           = 0x2755,

    // Radiance
    kWattPerSquareMetreSteradian                = 0x2756,

    // Catalytic Activity Concentration
    kKatalPerCubicMetre                         = 0x2757,

    // Time (non-SI units)
    kMinute                                     = 0x2760,
    kHour                                       = 0x2761,
    kDay                                        = 0x2762,

    // Plane Angle (non-SI units)
    kDegree                                     = 0x2763,
    kAngleMinute                                = 0x2764,  // arc minute
    kAngleSecond                                = 0x2765,  // arc second

    // Area (non-SI units)
    kHectare                                    = 0x2766,

    // Volume (non-SI units)
    kLitre                                      = 0x2767,

    // Mass (non-SI units)
    kTonne                                      = 0x2768,

    // Pressure (non-SI units)
    kBar                                        = 0x2780,
    kMillimetreOfMercury                        = 0x2781,

    // Length (non-SI units)
    kAngstrom                                   = 0x2782,
    kNauticalMile                               = 0x2783,

    // Area (non-SI units)
    kBarn                                       = 0x2784,

    // Velocity (non-SI units)
    kKnot                                       = 0x2785,

    // Logarithmic Radio Quantity
    kNeper                                      = 0x2786,
    kBel                                        = 0x2787,

    // Imperial/US Customary Units
    kYard                                       = 0x27A0,
    kParsec                                     = 0x27A1,
    kInch                                       = 0x27A2,
    kFoot                                       = 0x27A3,
    kMile                                       = 0x27A4,
    kPoundForcePerSquareInch                    = 0x27A5,
    kKilometrePerHour                           = 0x27A6,
    kMilePerHour                                = 0x27A7,
    kRevolutionPerMinute                        = 0x27A8,
    kGramCalorie                                = 0x27A9,
    kKilogramCalorie                            = 0x27AA,
    kKilowattHour                               = 0x27AB,
    kDegreeFahrenheit                           = 0x27AC,

    // Percentage and Related
    kPercentage                                 = 0x27AD,
    kPerMille                                   = 0x27AE,

    // Medical/Health Units
    kBeatsPerMinute                             = 0x27AF,
    kAmpereHours                                = 0x27B0,
    kMilligramPerDecilitre                      = 0x27B1,
    kMillimolePerLitre                          = 0x27B2,

    // Time (larger units)
    kYear                                       = 0x27B3,
    kMonth                                      = 0x27B4,

    // Miscellaneous
    kCountPerCubicMetre                         = 0x27B5,
    kIrradianceWattPerSquareMetre               = 0x27B6,
    kMilliliterPerKilogramPerMinute             = 0x27B7,
    kPound                                      = 0x27B8
};

/**
 * @brief Bluetooth SIG Assigned Appearance Values
 *
 * Used to indicate the external appearance of the device to the user.
 * Appearance values are organized into categories and subcategories.
 *
 * Reference: Bluetooth SIG Assigned Numbers - Appearance Values
 * https://www.bluetooth.com/specifications/assigned-numbers/
 *
 * Format: Category (bits 15-6) | Subcategory (bits 5-0)
 */
enum class BleAppearance : uint16_t {
    // Unknown
    kUnknown                            = 0x0000, // Unknown

    // Generic category
    kGenericPhone                       = 0x0040, // Generic Phone
    kGenericComputer                    = 0x0080, // Generic Computer
    kGenericWatch                       = 0x00C0, // Generic Watch
    kGenericClock                       = 0x0100, // Generic Clock
    kGenericDisplay                     = 0x0140, // Generic Display
    kGenericRemoteControl               = 0x0180, // Generic Remote Control
    kGenericEyeGlasses                  = 0x01C0, // Generic Eye-glasses
    kGenericTag                         = 0x0200, // Generic Tag
    kGenericKeyring                     = 0x0240, // Generic Keyring
    kGenericMediaPlayer                 = 0x0280, // Generic Media Player
    kGenericBarcodeScanner              = 0x02C0, // Generic Barcode Scanner
    kGenericThermometer                 = 0x0300, // Generic Thermometer
    kGenericHeartRateSensor             = 0x0340, // Generic Heart Rate Sensor
    kGenericBloodPressure               = 0x0380, // Generic Blood Pressure
    kGenericHumanInterfaceDevice        = 0x03C0, // Generic Human Interface Device
    kGenericGlucoseMeter                = 0x0400, // Generic Glucose Meter
    kGenericRunningWalkingSensor        = 0x0440, // Generic Running/Walking Sensor
    kGenericCycling                     = 0x0480, // Generic Cycling
    kGenericPulseOximeter               = 0x0C40, // Generic Pulse Oximeter
    kGenericWeightScale                 = 0x0C80, // Generic Weight Scale
    kGenericOutdoorSportsActivity       = 0x1440, // Generic Outdoor Sports Activity
    kSensor                             = 0x0540, // Generic Sensor
    kLightFixture                       = 0x0541, // Light Fixture
    kFan                                = 0x0542, // Fan
    kAirConditioning                    = 0x0543, // Air Conditioning
    kHumidifier                         = 0x0544, // Humidifier
    kHeating                            = 0x0545, // Heating
    kAccessControl                      = 0x0546, // Access Control
    kMotorizedDevice                    = 0x0547, // Motorized Device
    kPowerDevice                        = 0x0548, // Power Device
    kLightSource                        = 0x0549, // Light Source
    kWindowCovering                     = 0x054A, // Window Covering
    kFlameSensor                        = 0x054B, // Flame Sensor
    kVehicleTirePressureSensor          = 0x054C, // Vehicle Tire Pressure Sensor
};

/// @brief Presentation Format descriptor value (UUID 0x2904)
/// @details 7-byte packed structure defining the format of a characteristic value
#pragma pack(push, 1)
struct PresentationFormatValue {
    uint8_t format;         ///< Format type (see GattFormat enum)
    int8_t exponent;        ///< Decimal exponent (e.g., -2 for 0.01)
    uint16_t unit;          ///< Unit type (see GattUnit enum)
    uint8_t name_space;     ///< Namespace identifier (0x01 = Bluetooth SIG)
    uint16_t description;   ///< Description identifier
};
#pragma pack(pop)

} // namespace blex_standard

// Forward declarations
template<typename T, auto UUID, T Value, typename... Descriptors>
struct ConstCharacteristic;

/**
 * @brief Presentation Format descriptor base (backend-agnostic)
 * @details Defines the format of a characteristic value. Inherits from DescriptorBase.
 */
template<uint8_t Format, int8_t Exponent, uint16_t Unit, uint8_t Namespace, uint16_t Description>
struct PresentationFormatDescriptorBase : DescriptorBase<
    blex_standard::PresentationFormatValue,
    static_cast<uint16_t>(0x2904),
    ::Permissions<>::AllowRead,
    sizeof(blex_standard::PresentationFormatValue)
> {
    // Base is inherited from DescriptorBase via CRTP!
    static constexpr blex_standard::PresentationFormatValue value{Format, Exponent, Unit, Namespace, Description};
    using is_presentation_format_descriptor = void;  // Marker for trait detection
};

/**
 * @brief Aggregate Format descriptor base (backend-agnostic)
 * @details Groups multiple Presentation Format descriptors. Inherits from DescriptorBase.
 */
template<typename... PresentationFormatDescriptors>
struct AggregateFormatDescriptorBase : DescriptorBase<
    void,  // No value type for aggregate
    static_cast<uint16_t>(0x2905),
    ::Permissions<>::AllowRead,
    0  // Size handled by NimBLE
> {
    // Base is inherited from DescriptorBase via CRTP!
    static constexpr size_t num_descriptors = sizeof...(PresentationFormatDescriptors);

    static_assert(num_descriptors > 0, "AggregateFormat requires at least one PresentationFormat descriptor");
    static_assert((... && blex_core::has_presentation_format_marker<PresentationFormatDescriptors>()),
                  "AggregateFormat only accepts PresentationFormatDescriptor types");
};

namespace blex_standard {

/**
 * @brief Standard BLE Descriptors
 * @details Convenience type aliases for common BLE descriptors
 */
namespace descriptors {
    /**
     * User Description Descriptor (0x2901)
     * Provides a human-readable description of a characteristic
     */
    template<const char* DescText>
    using UserDescription = ::ConstDescriptor<const char*, 0x2901, DescText, ::Permissions<>::AllowRead>;

    /**
     * Presentation Format Descriptor (0x2904)
     * Defines the format of the characteristic value
     *
     * @tparam Format - GATT format (see GattFormat enum)
     * @tparam Exponent - Decimal exponent (e.g., -2 for 0.01)
     * @tparam Unit - GATT unit (see GattUnit enum)
     * @tparam Namespace - Namespace identifier (default 0x01 = Bluetooth SIG)
     * @tparam Description - Description identifier (default 0x0000 = none)
     */
    template<
        GattFormat Format,
        int8_t Exponent,
        GattUnit Unit,
        uint8_t Namespace = 0x01,
        uint16_t Description = 0x0000
    >
    using PresentationFormat = ::PresentationFormatDescriptorBase<
        static_cast<uint8_t>(Format),
        Exponent,
        static_cast<uint16_t>(Unit),
        Namespace,
        Description
    >;

    /**
     * Aggregate Format Descriptor (0x2905)
     * Groups multiple Presentation Format descriptors
     */
    template<typename... PresentationFormatDescriptors>
    using AggregateFormat = ::AggregateFormatDescriptorBase<PresentationFormatDescriptors...>;
}

/**
 * @brief Standard Device Information Service (0x180A) Characteristics
 * @details All characteristics are read-only with compile-time constant values
 */
namespace chars {
    /**
     * Model Number String (0x2A24)
     * Represents the model number assigned by the device vendor
     */
    template<const char* MdlNumber>
    using ModelNumber = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A24), MdlNumber>;

    /**
     * Serial Number String (0x2A25)
     * Represents the serial number for a particular instance of the device
     */
    template<const char* SerNumber>
    using SerialNumber = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A25), SerNumber>;

    /**
     * Firmware Revision String (0x2A26)
     * Represents the firmware revision for the firmware within the device
     */
    template<const char* FrmRevision>
    using FirmwareRevision = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A26), FrmRevision>;

    /**
     * Hardware Revision String (0x2A27)
     * Represents the hardware revision for the hardware within the device
     */
    template<const char* HwRevision>
    using HardwareRevision = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A27), HwRevision>;

    /**
     * Software Revision String (0x2A28)
     * Represents the software revision for the software within the device
     */
    template<const char* SftRevision>
    using SoftwareRevision = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A28), SftRevision>;

    /**
     * Manufacturer Name String (0x2A29)
     * Represents the name of the manufacturer of the device
     */
    template<const char* MfgName>
    using ManufacturerName = ::ConstCharacteristic<const char*, static_cast<uint16_t>(0x2A29), MfgName>;
}

} // namespace blex_standard

#endif // BLEX_STANDARD_HPP_