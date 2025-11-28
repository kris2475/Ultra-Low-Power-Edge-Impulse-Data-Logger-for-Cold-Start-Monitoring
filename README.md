# Ultra-Low-Power Edge Impulse (EI) Data Logger for Cold-Start Monitoring

*A low-budget, pre-Christmas embedded ML project*

## 💡 Overview

This project combines ultra-low-power system design, embedded sensing, and TinyML-ready data logging. It focuses on measuring and analysing the **Time-to-First-Measurement (TTFM)** of an ESP32-based node waking from deep sleep — a key performance parameter in energy-constrained IoT systems.

Using the **XIAO ESP32-S3** (or a similar ESP32 variant), we build a standalone, battery-powered logger that periodically wakes, captures sensor readings (Lux, Temperature, Pressure), measures the time required to produce that reading after wakeup, and stores the result in a dataset suitable for Edge Impulse (EI) or other ML pipelines.

The goal is to determine if environmental factors (like temperature) have a measurable, non-linear impact on the chip's cold-start performance.

## 🎯 Goal

Create an **ultra-low-power data logging node** that:

* Wakes from deep sleep via **RTC timer** (currently set to 60-second intervals).

* Measures **Time-to-First-Measurement (TTFM)** with high accuracy (microsecond resolution).

* Reads environmental sensor data (Lux, Temperature, Pressure).

* Logs data in **ML-ready CSV format** for Edge Impulse or Python analysis.

* Operates on a small **LiPo battery** with minimal power consumption.

## 🛠️ Technical Details & Budget

| Area | Detail | Cost Estimate |
| :---- | :---- | :---- |
| **Microcontroller** | XIAO ESP32S3 (specifically used in this project) or standard ESP32-S3 dev board | ≈ £10–£15 (likely already owned) |
| **Sensors Used** | **BH1750** (Lux) and **BMP180** (Temperature/Pressure) | ≈ £0–£5 |
| **Storage** | microSD Card (SPI interface) for long-term logging | ≈ £0–£5 |
| **Power** | Single LiPo cell | ≈ £5 |
| **Total** | **Potentially < £20** using existing hardware | |

## ⚙️ Implementation Steps

### 1. Ultra-Low-Power Wakeup Module (C++ / Arduino)

* The core of the firmware (`ULP_EI_TTFM_Logger.ino`) implements a **deep-sleep → wake → measure → log → sleep** state machine.

* The wake source is the **RTC timer** (`esp_sleep_enable_timer_wakeup`).

* **TTFM Measurement:** The time is measured using `esp_timer_get_time()` to achieve microsecond resolution, specifically capturing the time from the sensor command to the successful data read.

### 2. Data Acquisition & TinyML-Ready Logging (C++ / Edge Impulse)

On each wake cycle:

1. **Start TTFM Timer:** Begin timing immediately before sending the BH1750 (Lux) measurement command.

2. **Capture Sensor Data:** Read Lux (BH1750) after its $180\text{ms}$ integration time, then read Temperature/Pressure (BMP180).

3. **Stop TTFM Timer:** Record the total time once the Lux value is successfully acquired.

4. **Log to CSV:** Store the data on the microSD card in the following CSV format:
   
Timestamp_ms, Lux_Value, Temperature_C, Pressure_Pa, TTFM_us

5. **Re-enter Deep Sleep:** Program the next wake time and call `esp_deep_sleep_start()`.

### 3. Safety/Boot Feature (ESP32-S3 Specific)

Due to the sensitive nature of the ESP32-S3's native USB port during deep sleep, a **SAFE BOOT** feature is implemented:

* After a fresh flash (or a cold boot/power-on), the device remains awake and prints a message.

* Deep Sleep is disabled until the user manually sends the character **`C`** via the Serial Monitor. This prevents the logger from becoming unreachable if errors occur during initialization.

---

## 📉 Expected Performance Degradation (Hypothesis)

Initial log data (from $16^\circ\text{C}$ to $30^\circ\text{C}$) shows the TTFM is extremely stable, centered around the $\mathbf{180,000 \text{ us}}$ BH1750 sensor delay. This stability is expected to be non-linear across a wider thermal range.

### Stability Rationale

The ESP32-S3's internal RTC (Real-Time Clock) oscillator includes **Temperature Compensation Circuits (TCCs)**. These circuits actively adjust the oscillator frequency to maintain time integrity.

* **Observed Behavior:** The stability of the $\approx 181 \text{ ms}$ TTFM across the tested range confirms the TCCs effectively counteract thermal drift in typical ambient conditions.

### Expected Non-Linear Degradation

The performance is hypothesized to degrade sharply only when the device is stressed beyond the TCC's effective range, typically approaching freezing temperatures. 

| Condition | Expected TTFM Trend | Primary Cause of Degradation |
| :--- | :--- | :--- |
| **$40^\circ\text{C}$ to $10^\circ\text{C}$** | Stable (Flat Baseline) | Fixed sensor delay dominates; Chip compensation is fully effective. |
| **$0^\circ\text{C}$ and Below** | **Sharp Increase** (Non-linear) | 1. **Slower Oscillator Startup:** Extreme cold degrades silicon physics, increasing the time required for the internal clock to achieve stable oscillation. 2. **Battery Voltage Sag:** LiPo battery internal resistance rises non-linearly when cold, causing severe voltage drops during the instant power spike of wake-up. This extends the boot time. |

The next step is to test at $\mathbf{\approx 5^\circ\text{C}}$ (refrigerator) to identify the low-temperature point where the TTFM stability begins to degrade.
