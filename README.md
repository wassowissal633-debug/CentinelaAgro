# CentinelaAgro 🌱

**Smart Irrigation Through Predictive Soil Analysis**

CentinelaAgro is a closed-loop IoT irrigation system built on the ESP32-S3. Instead of watering on a fixed schedule, it analyzes soil moisture trends in real time and predicts when conditions will become critical — then acts before the plant actually suffers.

![Dashboard Overview](docs/dashboard-overview.png)

## How It Works

1. **Sense** — ESP32-S3 reads soil moisture, temperature, humidity (AHT30), and gas concentration.
2. **Publish** — Telemetry is sent to ThingsBoard over MQTT.
3. **Analyze** — A ThingsBoard rule chain script computes the soil moisture drop rate and estimates *hours-to-critical*.
4. **Decide** — If soil is already dry, or predicted to hit critical soon, the rule chain flags the risk and raises/clears an alarm.
5. **Act** — The decision is written back to the device as an MQTT **shared attribute**, which triggers the relay and pump — with a hardware-enforced safety timeout so it can never run indefinitely.

## Tech Stack

- **Hardware:** ESP32-S3, AHT30 (I2C), analog gas sensor, analog soil moisture sensor, relay module, 5V pump
- **Firmware:** ESP-IDF, FreeRTOS (PlatformIO)
- **Communication:** MQTT (telemetry + shared attributes, no RPC)
- **Cloud:** ThingsBoard rule chain (trend analysis, alarms, control)

## Key Feature: Predictive Logic

Rather than reacting only when a fixed threshold is crossed, the rule chain tracks soil moisture over time and extrapolates a drying trend. If the estimated time to critical is short, it irrigates *before* the threshold is even reached — catching both slow gradual drying and fast drops.

## Repository Structure

```
firmware/       ESP32-S3 source (ESP-IDF / PlatformIO)
rule-chain/     Exported ThingsBoard rule chain (JSON)
docs/           Dashboard & rule chain screenshots, hardware photo
demo/           Demo video / link
```

## Demo

Soil moisture climbing from 13% → 85% after the system triggered irrigation automatically — real hardware, not a simulation. See [`demo/`](demo/) for the full video.

## Setup

1. Flash `firmware/main.c` via PlatformIO, after setting your own Wi-Fi credentials and ThingsBoard device token.
2. Import `rule-chain/centinela_rule_chain.json` into your ThingsBoard instance.
3. Wire sensors per the pin definitions at the top of `main.c`.

## License

MIT — see [LICENSE](LICENSE).
