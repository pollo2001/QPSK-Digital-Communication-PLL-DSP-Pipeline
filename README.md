# 📡 QPSK Digital Communication & PLL DSP Pipeline

This MATLAB project explains and implements a complete QPSK (Quadrature Phase-Shift Keying) communication pipeline, with a special focus on carrier synchronization using a band-edge Phase-Locked Loop (PLL).

The project demonstrates:
* Symbol shaping & Inter-Symbol Interference (ISI) mitigation
* Hilbert transforms for analytic (complex) signal processing
* Band-edge PLL design for phase tracking and error correction
* Diagnostic visualization with eye diagrams & constellations

---

## 🔬 Pipeline Component Breakdown

### 1️⃣ Transmit & Receive Filters
* **🎯 Purpose:** Shape transmitted symbols to reduce ISI and use a matched filter at the receiver to maximize Signal-to-Noise Ratio (SNR).
* **🔧 Code Choices:** `sqrtNyquistFilter` for pulse shaping, `rxMatchedFilter` for matched filtering.
* **DSP Concepts:** Nyquist ISI criterion, matched filtering, pulse shaping.

### 2️⃣ Band-Edge Matched Filter
* **🎯 Purpose:** Emphasize the signal spectrum's edges, generating an error signal for the PLL phase detector.
* **🔧 Code Choices:** Kaiser window to reduce spectral leakage; multiplying by time indices to highlight edge frequencies.
* **DSP Concepts:** Band-edge detection, windowing, spectral shaping, phase error detection.

### 3️⃣ Hilbert Transformer
* **🎯 Purpose:** Convert the real passband signal into its complex analytic form (I/Q components).
* **🔧 Code Choices:** Discrete Hilbert filter coefficients, smoothed with a Kaiser window.
* **DSP Concepts:** Analytic signals, complex baseband representation, quadrature signals.

### 4️⃣ Analytic Signal & Positive-Frequency Filter
* **🎯 Purpose:** Isolate the positive frequency components of the analytic signal for PLL processing.
* **🔧 Code Choices:** Convolve the analytic signal with the band-edge filter.
* **DSP Concepts:** Complex filtering, positive frequency isolation.

### 5️⃣ Negative-Frequency Filter
* **🎯 Purpose:** Create the conjugate of the positive-frequency filter, required for calculating the phase error.
* **🔧 Code Choices:** Helps compute the power difference that drives the PLL.
* **DSP Concepts:** Conjugate signals, negative-frequency tracking.

### 6️⃣ Shaping Filter Output & Eye Diagram
* **🎯 Purpose:** Simulate the transmitted QPSK symbols and visualize signal integrity before the channel.
* **🔧 Code Choices:** Upsample data symbols and pass through the shaping filter.
* **DSP Concepts:** Symbol shaping, eye diagrams, constellation analysis, ISI visualization.
[Image of a clear QPSK eye diagram]

### 7️⃣ Received Signal with Phase Offset
* **🎯 Purpose:** Simulate a real-world carrier frequency offset (CFO) to test the PLL.
* **🔧 Code Choices:** Multiply the transmitted signal by a complex exponential to rotate its phase.
* **DSP Concepts:** Carrier phase error, phase rotation, channel modeling.

### 8️⃣ PLL Loop Filter & Phase Tracking
* **🎯 Purpose:** Recover the carrier phase and correct the received symbols in real-time.
* **🔧 Code Choices:** Digital PLL using band-edge filters to generate a phase error signal, which feeds a PI (Proportional-Integrator) loop filter. A phase accumulator applies the correction.
* **DSP Concepts:** Phase-Locked Loop (PLL) design, loop filter, phase tracking, carrier synchronization.

### 9️⃣ Post-PLL Output Constellation & Eye Diagram
* **🎯 Purpose:** Visually verify the PLL's performance and confirm phase lock and symbol recovery.
* **🔧 Code Choices:** Plot the constellation and eye diagram of the *corrected* signal.
* **DSP Concepts:** Phase-compensated recovery, constellation analysis.
[Image of a clean QPSK constellation plot]

### 🔟 Attenuated PLL Loop Test
* **🎯 Purpose:** Test the PLL's robustness and sensitivity under weaker signal (low SNR) conditions.
* **🔧 Code Choices:** Scale down the band-edge power difference signal before it enters the loop filter.
* **DSP Concepts:** Loop sensitivity, robustness, lock range, low-signal behavior.

---

## 📝 Key Concepts & Notes
* Shift registers are used to maintain a history of samples for FIR filtering and PLL computations.
* Convolutions are the core operation for implementing all FIR filters.
* FFT visualizations are used throughout to verify the magnitude and frequency response of filters.
* Eye diagrams and constellations are the two primary diagnostic tools for any digital communication system.

---

## 🚀 Getting Started

### Prerequisites
* MATLAB (e.g., R2024a or newer)
* MATLAB DSP Toolbox

### Usage
1.  Open the project in MATLAB.
2.  Run the main script (e.g., `ppp.m`).
3.  Observe the generated figures, paying close attention to the pre- and post-PLL constellations.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
