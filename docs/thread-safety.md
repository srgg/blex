# Thread Safety and Concurrency Model

## Thread Safety Design

- **Zero-overhead:** eliminates unnecessary locks on critical data paths.
- **Pluggable lock policy:** Includes built-in <a href="#nolock">NoLock</a> and <a href="#free-rtos-lock">FreeRTOSLock</a> options.

The lock strategy uses a policy-based design and compile-time polymorphism, as shown in the following example:
```c++
#include "blex.hpp"

// The lock policy is automatically selected based on preprocessor definitions.
using MyBlex = blex<>;  // Uses FreeRTOSLock on ESP32 and NoLock on single-core platforms

// Explicit selection for multi-core platforms
using MultiCoreBlex = blex<FreeRTOSLock>;

// Explicit selection for single-core platforms (zero overhead)
using SingleCoreBlex = blex<NoLock>;

// Explicit selection for a custom lock policy
using SingleCoreBlex = blex<MyCustomLockPolicy>;
```

**Note:** Custom lock policies are supported, as described below.

# Synchronization Guarantees

This section outlines the concurrency and threading model used by Blex.
Blex is built on **NimBLE** and inherits its execution and synchronization behavior.


## NimBLE Guarantees

Before reviewing Blex’s guarantees, it is important to understand NimBLE’s threading model, which defines strict rules for handling BLE callbacks and state updates:

- **Serialized Callback Execution**: All BLE callbacks (`onRead`, `onWrite`, `onSubscribe`, `onStatus`) run on a **single** NimBLE thread.  
  NimBLE never executes characteristic callbacks concurrently.

- **No Callback-to-Callback Data Races**: Serialized callback execution prevents data races between callbacks, making them inherently thread-safe with respect to each other.

- **Thread-Safe Operations**: NimBLE ensures that operations such as `notify()`, `setValue()`, and `getValue()` are safe to call from any thread, meaning the internal synchronization prevents state corruption.

## Blex Guarantees

Blex extends NimBLE’s behavior to ensure predictable and safe multithreaded access to characteristic values.

**Important:** These guarantees require a lock policy that provides mutual exclusion, such as `FreeRTOSLock`. The `NoLock` policy disables synchronization entirely.
Use `NoLock` only when you are certain that concurrent synchronized access will not occur.

- **thread-safe `setValue()`**:
  - **For NOTIFY no-read characteristics**: `setValue()` may be called concurrently, resulting in lock-free atomic notifications.
  - **For READ or READ+NOTIFY characteristics**: Concurrent `setValue()` calls are safe meaning `onRead` always receives a consistent value.
  - **For all characteristic types**: `setValue()` prevents internal state corruption during concurrent access.
  > In C/C++ terms, `setValue()` behaves like an atomic “write + publish” operation.

- **Characteristic callback thread-safety:**
  All callbacks occur within the serialized callback execution enforced by NimBLE.
  - **`onRead`**: Does not race with concurrent `setValue()` calls and always receives a consistent snapshot.
  - **`onWrite`**: Does not race with `setValue()`. Internal calls to `getValue()` within the callback always return a safely synchronized value.
  - **`onSubscribe` and `onStatus`**: Executed in order with other characteristic callbacks, following NimBLE’s serialized execution model.

- **Initialization and Metadata Safety**:
  - **Service initialization (`init`)**: Must complete before any BLE callbacks occur. Initialization must run in a single-threaded context.
  - **Metadata access**: After `init()` completes, characteristic metadata is immutable and safe to read from any thread.

### Lock Policy

Lock policy is a platform-specific synchronization abstraction that defines when and how locks are acquired. Choose a lock policy based on your platform, threading model, and whether your application requires or manages synchronization.
Locks are acquired and released through the `lock()` and `unlock()` methods.

**Built-in Lock Policies:**
NoLock: has a zero overhead. Use for single-core systems or when thread safety is guaranteed.
FreeRTOSLock: Uses a FreeRTOS recursive mutex for task-context locking. Use this on multicore or RTOS-based systems to ensure callback safety, unless you provide synchronization by other means.

```cpp
template<typename Tag = void>
struct LockPolicy {
    void lock() const noexcept;    // Acquire exclusive access
    void unlock() const noexcept;  // Release exclusive access
};
```
The Tag parameter enables per-instance locking, allowing independent static locks and fine-grained concurrency control.

**Choosing the right Tag:**
- Each unique tag type provides its own independent lock.
- Use the resource type as the tag to enable per-resource locking.
- Balance granularity: Finer tags reduce contention but increase memory usage.
- Do not share tags between unrelated resources.


### Built-in Synchronization Primitives

`blex_sync` provides well-defined synchronization primitives that can be used with the selected blex lock policy or with its own lock policy.

#### ScopedLock `ScopedLock<LockPolicy, Tag>`
The RAII scope lock manages the lock's lifecycle; each tag uses its own static lock.

```cpp
 #include "blex.hpp"

  struct MySensorData {};  // Tag type for this resource

  void processData() {
       blex_sync::ScopedLock<FreeRTOSLock, MySensorData> guard;  // Lock acquired
      // ... critical section ...
  }  // Lock released
```

**Key features:**
- Per-tag static locks avoid static destruction order issues.
- Zero overhead with the `NoLock` policy, as locking is eliminated at compile time.
