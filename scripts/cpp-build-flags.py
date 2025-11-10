Import("env")

# C++-only compiler flags (applied only to .cpp files, not .c files)
#
# Rationale:
#   - PlatformIO's build_flags applies to both C and C++ files
#   - C++-specific flags would trigger warnings on C files
#   - This script appends flags only to CXXFLAGS for clean compilation
#
# Toolchain: GCC 12.2.0 (crosstool-NG esp-12.2.0) - Partial C++20 support

env.Append(CXXFLAGS=[
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
    "-std=gnu++2a",              # GNU C++20 (includes GNU extensions like 'typeof')
    "-fconcepts",                # Enable concepts (explicit enable)

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
    "-Wnon-virtual-dtor",        # Warn: virtual methods without virtual destructor
    "-Woverloaded-virtual",      # Warn: hidden virtual functions

    # ==================== Suppress Third-Party Library Warnings ====================
    "-Wno-unused-parameter",     # Embedded code often has unused params (callbacks)
    "-Wno-psabi",                # Suppress ABI compatibility warnings (cross-compilation)
    "-Wno-pedantic",             # Suppress pedantic warnings from Arduino/ESP-IDF (anonymous structs)
    "-Wno-suggest-override",     # Suppress missing override warnings from Arduino/Adafruit libs
    "-Wno-variadic-macros",      # Suppress variadic macro warnings from NimBLE
])
