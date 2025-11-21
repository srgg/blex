Import("env")

# C++20 build flags with embedded optimizations
#
# This script is designed to be reusable across projects using the blex library.
# It configures C++20 compilation with strict warnings and embedded-appropriate settings.
#
# Toolchain: GCC 12.2.0 (crosstool-NG esp-12.2.0) - Partial C++20 support

# Remove -std=gnu++11 from all compiler flags (set by an ESP-IDF/Arduino framework)
for var in ["CFLAGS", "CCFLAGS", "CXXFLAGS"]:
    env[var] = [f for f in env.get(var, []) if f != "-std=gnu++11"]

# ==================== C++20 Standard (Partial Support) ====================
# NOTE: Espressif GCC 12.2.0 has PARTIAL C++20 support:
#   ✓ Supported:
#     - Concepts (template constraints)
#     - [[nodiscard]] (without reason strings)
#     - std::bool_constant
#     - Custom buffer_view<T> (std::span alternative)
#   ✗ NOT Supported:
#     - consteval (not implemented despite __cpp_consteval macro claiming support)
#     - constinit (not available)
#     - std::span (header exists in toolchain but Arduino framework doesn't expose it)
#     - [[nodiscard("reason")]] (only plain [[nodiscard]] works, no reason strings)
#   ⚠ __cplusplus = 201709L (incorrectly reports C++17 despite -std=gnu++2a flag)

# Set language standards
env.Append(CFLAGS=[
    "-std=gnu17",                # GNU C17 (C11 with bug fixes + GNU extensions)
])
env.Append(CXXFLAGS=[
    "-std=gnu++2a",              # GNU C++20 (includes GNU extensions like 'typeof')
])

# C++-only flags (would cause warnings on C files)
env.Append(CXXFLAGS=[
    "-fconcepts",                # Enable concepts (explicit enable for C++)

    # ==================== Embedded C++ ====================
    "-fno-exceptions",           # Disable exceptions (embedded best practice)
    "-fno-rtti",                 # Disable RTTI (reduces binary ~5KB, faster dispatch)

    # ==================== Code Size Optimization ====================
    "-ffunction-sections",       # Each function in separate section (enables --gc-sections)
    "-fdata-sections",           # Each data item in separate section (enables --gc-sections)

    # ==================== Warnings: Strict for Our Code ====================
    "-Wall",                     # Standard warnings
    "-Wextra",                   # Extra warnings
    "-Werror=return-type",       # Missing return statement = compile error
    "-Woverloaded-virtual",      # Warn: hidden virtual functions (not in -Wall/-Wextra)
    "-Wno-suggest-override",     # Suppress missing override warnings from Arduino/Adafruit libs
])
