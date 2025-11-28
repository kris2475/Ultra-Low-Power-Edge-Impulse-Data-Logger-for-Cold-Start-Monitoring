# Ultra-Low-Power Edge Impulse (EI) Data Logger for Cold-Start Monitoring

## 💡 Overview

This project combines ultra-low-power system design, embedded sensing, and TinyML-ready data logging. It focuses on measuring and analysing the **Time-to-First-Measurement (TTFM)** of an ESP32-based node waking from deep sleep — a key performance parameter in energy-constrained IoT systems.

Using the **XIAO ESP32-S3** (or a similar ESP32 variant), I build a standalone, battery-powered logger that periodically wakes, captures sensor readings (Lux, Temperature, Pressure), measures the time required to produce that reading after wake-up, and stores the result in a dataset suitable for Edge Impulse (EI) or other ML pipelines.

The goal is to determine if environmental factors (like temperature) have a measurable, non-linear impact on the chip's cold-start performance.

## 🎯 Goal

Create an **ultra-low-power data logging node** that:

* Wakes from deep sleep via **RTC timer** (currently set to 60-second intervals).

* Measures **Time-to-First-Measurement (TTFM)** with high accuracy (microsecond resolution).

* Reads environmental sensor data (Lux, Temperature, Pressure).

* Logs data in **ML-ready CSV format** for Edge Impulse or Python analysis.

* Operates on a small **LiPo battery** with minimal power consumption.

## 🛠️ Technical Details & Wiring Connections

This set-up uses two primary I2C sensors (BH1750 and BMP180) wired to a single I2C bus on the XIAO ESP32-S3. The specific I2C pins used in the accompanying code (`sensors_read_debug.ino`) are defined below.

| Component | Function | I2C Address | XIAO ESP32-S3 Pin | Code Definition |
| :--- | :--- | :--- | :--- | :--- |
| **XIAO ESP32-S3** | Microcontroller/RTC | N/A | N/A | N/A |
| **BH1750 (Lux)** | Light Sensor | `0x23` | D43 (SDA) / D44 (SCL) | `#define SDA_PIN 43` |
| **BMP180** | Temp/Pressure Sensor | `0x77` | D43 (SDA) / D44 (SCL) | `#define SCL_PIN 44` |
| **Storage** | MicroSD Card | SPI (Optional) | N/A | N/A |
| **Power** | 3.7V LiPo Battery | N/A | JST Connector | N/A |

### Wiring Diagram

Connect the VCC and GND of both the BH1750 and BMP180 to the 3.3V and GND pins of the XIAO. The I2C data lines should be connected as follows:

| Sensor Pin | XIAO ESP32-S3 Pin |
| :--- | :--- |
| **SDA** (Data) | **D43** (GPIO 43) |
| **SCL** (Clock) | **D44** (GPIO 44) |

---

## ⚙️ Implementation Steps

### 1. Ultra-Low-Power Wake-up Module (C++ / Arduino)

* The core of the firmware (`ULP_EI_TTFM_Logger.ino`) implements a **deep-sleep → wake-up → measure → log → sleep** state machine.

* The wake source is the **RTC timer** (`esp_sleep_enable_timer_wakeup`).

* **TTFM Measurement:** The time is measured using `esp_timer_get_time()` to achieve microsecond resolution, specifically capturing the time from the sensor command to the successful data read.

### 2. Data Acquisition & TinyML-Ready Logging (C++ / Edge Impulse)

On each wake cycle:

1. **Start TTFM Timer:** Begin timing immediately before sending the BH1750 (Lux) measurement command.

2. **Capture Sensor Data:** Read Lux (BH1750) after its $180\text{ms}$ integration time, then read Temperature/Pressure (BMP180).

3. **Stop TTFM Timer:** Record the total time once the Lux value is successfully acquired.

4. **Log to CSV:** Store the data on the microSD card in the following CSV format:
   
Timestamp\_ms, Lux\_Value, Temperature\_C, Pressure\_Pa, TTFM\_us

5. **Re-enter Deep Sleep:** Programme the next wake time and call `esp_deep_sleep_start()`.
### 3. Safety/Boot Feature (ESP32-S3 Specific)

Due to the sensitive nature of the ESP32-S3's native USB port during deep sleep, a **SAFE BOOT** feature is implemented:

* **Behaviour when connected to PC via USB-C:** After a fresh flash, the device remains awake, waiting for the character **`C`** via the Serial Monitor. This prevents the logger from entering Deep Sleep immediately and becoming unreachable for debugging/re-flashing.
* **Behaviour when powered by Battery/Power Pack:** If no USB-C connection is active, the device **automatically bypasses** the wait and initiates the first Deep Sleep cycle immediately after `setup()` completes.

---

## 📈 Experimental Results: TTFM vs. Temperature Analysis

The initial hypothesis suggested a **non-linear, sharp degradation** below $0^{\circ}\text{C}$. However, the collected data from $\mathbf{-14.0^{\circ}\text{C}}$ to $\mathbf{29.6^{\circ}\text{C}}$ reveals a different, more predictable trend: the relationship is **statistically linear** across the entire tested range.

### Key Findings

The **Time-to-First-Measurement (TTFM)** exhibits a strong, positive linear correlation with a drop in temperature, confirming that cold weather significantly extends the cold-start time.

| Metric | Value | Interpretation |
| :--- | :--- | :--- |
| **Correlation ($r$)** | $\approx 0.999$ | Near-perfect positive linear relationship. |
| **Coefficient of Determination ($R^2$)** | $\approx 0.998$ | $\mathbf{99.8\%}$ of TTFM variation is explained by Temperature. |
| **Max TTFM Recorded** | $37322\ \mu\text{s}$ at $-14.0^{\circ}\text{C}$ | Represents a $\approx 7.8\%$ increase over the baseline warm-up time. |

### Linear Regression Model

The relationship between Temperature ($T$) and TTFM ($\text{TTFM}$) can be accurately modelled by the following linear equation:

$$\text{TTFM} (\mu\text{s}) \approx -13.50 \times T (^{\circ}\mathrm{C}) + 35605$$

**Rate of Change:** The TTFM increases by approximately $\mathbf{13.50\ \mu\text{s}}$ for every $\mathbf{1^{\circ}\mathrm{C}}$ decrease in temperature.

### Why the Data Appears Stepped (Granularity)

While temperature is a continuous variable, the plot shows the TTFM data points clustered into horizontal "steps" rather than following a perfectly smooth line. This is due to the combined effect of sensor hardware and software timing resolution:

1.  **Dominant Fixed Hardware Delay:** The BH1750 Lux sensor's required $\mathbf{180\text{ ms}}$ integration time ($\approx \mathbf{180,000 \ \mu\text{s}}$) forms a massive, fixed baseline for the total TTFM. The small, cold-dependent changes in the ESP32's start-up time only slightly perturb this large value.

2.  **Integer Timing Resolution (Quantisation):** The TTFM is measured using the `esp_timer_get_time()` function, which returns an **integer count** in microseconds ($\mu\text{s}$). It cannot measure fractions of a microsecond.

**The Stepping Effect:** As the temperature drops, the linear increase in start-up time is gradual, but the measured TTFM can only report the next whole microsecond. Consequently, many successive temperature readings share the exact same TTFM value until the accumulated cold-start delay is large enough to push the *total* measured time over the threshold to the next available integer microsecond. This results in the clear stepped pattern observed in the scatter plot.

### Resolution of Hypothesis

While external literature suggests the physical mechanism (crystal oscillator stabilisation) is fundamentally non-linear, the measured data indicates that the linear approximation is highly accurate for this device over the tested environmental range. This robust linearity simplifies firmware compensation, allowing for highly predictable and reliable wake-up timing adjustments based on measured temperature.
