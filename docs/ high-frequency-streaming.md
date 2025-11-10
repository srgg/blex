# High-Frequency Streaming

BLE supports two streaming options:

- **GATT Notifications**: simple and widely compatible, but protocol overhead can limit their speed.
  The default MTU limits payloads to 20 bytes, with a maximum rate of about 130 Hz at a 7.5 ms interval.
  Typical throughput is 10 to 20 KB/s. Latency depends on queuing and connection timing.
  Use GATT notifications for moderate-rate sensors. For real-time performance, consider alternative methods.

- **L2CAP CoC (Connection-oriented Channels):** Supports high-rate, low-latency streaming with throughput over 100 KB/s, reduced overhead, and stable latency.
  It is efficient for continuous transfers but requires manual flow control and a custom client implementation.

## Stream Data via GATT Notifications in Blex

Blex provides a simple, consistent API for streaming data via GATT notifications: for Notify-enabled characteristics, `setValue()` sends updates to subscribers.
There are built-in optimizations to reduce latency for Notify-only and Read+Notify Characteristics described below.

### Notify-only (non-readable) characteristics
For Notify-only characteristics, setValue() bypasses storage and sends notifications directly to subscribers.
This avoids locking, reduces latency, and improves performance for high-speed streams.

When possible, use Notify-only characteristics for performance-critical streaming to maximize speed and minimize latency.

### Read+Notify Characteristics
If Notify-only is not feasible and the characteristic must support both read and notify, high-rate streaming can be affected by simultaneous read requests.

To address this, Blex offers a lock-free optimization for **Read+Notify** characteristics:
When new notification data is available, Blex returns the cached value instead of running `onRead`, preventing duplicate sampling.
This results in faster response, lower resource usage, and better efficiency. Sensor reads occur only when necessary.

This optimization is most effective at high, continuous notification rates because it prevents redundant transmissions and reduces latency.
It is most effective at 50 to 100 Hz for sensors or 100 to 200 Hz for IMU data.
The maximum sustainable BLE rate is approximately 130 Hz, limited by the 7.5 ms minimum connection interval.
At 100 Hz with 6-byte IMU packets, throughput is about 600 bytes per second, which is well within BLE limits.
This optimization is best for applications that can accept 10 to 20 ms latency at 100 Hz. Make sure your use case can tolerate this delay before applying.
notify() is the primary update mechanism, with minimal external changes between notifications.