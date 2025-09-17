# 📡 QPSK Digital Communication & PLL DSP Pipeline

This script explains each part of the MATLAB QPSK + PLL pipeline.
All sections are documented with Markdown-style comments and emojis for easy understanding.

The project demonstrates:
- Symbol shaping & ISI mitigation
- Hilbert/analytic signal processing
- Band-edge PLL phase tracking
- Eye diagrams & constellation visualization
"""

# %% === 1️⃣ Transmit & Receive Filters ===
"""
### Transmit & Receive Filters

- 🎯 Purpose: Shape transmitted symbols to reduce inter-symbol interference (ISI) and maximize SNR.
- 🔧 Code Choices:
    - `sqrtNyquistFilter` for pulse shaping.
    - Matched filter at receiver (`rxMatchedFilter`) to maximize detection performance.
    - FFT plots for frequency verification.

**DSP Concepts:** Nyquist ISI, matched filtering, frequency-domain analysis.
"""

# %% === 2️⃣ Band-Edge Matched Filter ===
"""
### Band-Edge Matched Filter

- 🎯 Purpose: Emphasizes signal spectrum edges for PLL phase error detection.
- 🔧 Code Choices:
    - Kaiser window reduces spectral leakage.
    - Multiplying by time indices highlights edge frequencies.

**DSP Concepts:** Band-edge detection, windowing, spectral shaping.
"""

# %% === 3️⃣ Hilbert Transformer ===
"""
### Hilbert Transformer

- 🎯 Purpose: Convert real signals into analytic (complex) form (I/Q components).
- 🔧 Code Choices:
    - Discrete Hilbert filter coefficients.
    - Kaiser window smooths frequency response.

**DSP Concepts:** Analytic signals, complex baseband representation.
"""

# %% === 4️⃣ Analytic Signal & Positive Frequency Filter ===
"""
### Analytic Signal & Positive-Frequency Filter

- 🎯 Purpose: Isolate positive frequency components for PLL processing.
- 🔧 Code Choices:
    - Convolve analytic signal with band-edge filter.
    - Ensures accurate phase error computation.

**DSP Concepts:** Complex filtering, positive frequency isolation.
"""

# %% === 5️⃣ Negative-Frequency Filter ===
"""
### Negative-Frequency Filter

- 🎯 Purpose: Conjugate of positive-frequency filter for phase error calculations.
- 🔧 Code Choices:
    - Helps compute power difference in PLL.

**DSP Concepts:** Conjugate signals, negative-frequency tracking.
"""

# %% === 6️⃣ Shaping Filter Output & Eye Diagram ===
"""
### Shaping Filter Output & Eye Diagram

- 🎯 Purpose: Simulate transmitted QPSK symbols; visualize signal integrity.
- 🔧 Code Choices:
    - Upsample data symbols.
    - Eye diagrams check ISI; constellations check symbol mapping.

**DSP Concepts:** Symbol shaping, eye diagrams, constellation analysis.
"""

# %% === 7️⃣ Received Signal with Phase Offset ===
"""
### Received Signal with Phase Offset

- 🎯 Purpose: Simulate real-world carrier frequency offset.
- 🔧 Code Choices:
    - Multiply transmitted signal by complex exponential to rotate phase.
    - Test PLL’s ability to correct phase.

**DSP Concepts:** Carrier phase error, phase rotation effects.
"""

# %% === 8️⃣ PLL Loop Filter & Phase Tracking ===
"""
### PLL Loop Filter & Phase Tracking

- 🎯 Purpose: Recover carrier phase and correct received symbols.
- 🔧 Code Choices:
    - Digital PLL with leaky integrator + PI loop filter.
    - Band-edge filters generate phase error input.
    - Phase accumulator adjusts the received signal.

**DSP Concepts:** PLL design, loop filter, phase tracking, synchronization.
"""

# %% === 9️⃣ Post-PLL Output Constellation & Eye Diagram ===
"""
### Post-PLL Output Constellation & Eye Diagram

- 🎯 Purpose: Verify phase recovery after PLL.
- 🔧 Code Choices:
    - Constellation and eye diagram visualization.
    - Confirms correct symbol recovery.

**DSP Concepts:** Phase-compensated recovery, eye diagram analysis.
"""

# %% === 🔟 Attenuated PLL Loop Test ===
"""
### Attenuated PLL Loop Test

- 🎯 Purpose: Test PLL under weaker signal conditions.
- 🔧 Code Choices:
    - Scale band-edge power difference.
    - Observe PLL sensitivity and stability.

**DSP Concepts:** Loop sensitivity, robustness, low-signal behavior.
"""

# %% === Notes 📝
"""
- Shift registers maintain history of samples for filtering and PLL computations.
- Convolutions implement FIR filters.
- FFT visualizations verify magnitude and frequency response.
- Eye diagrams & constellations are core diagnostics for QPSK communication.

This walkthrough can serve as a guide for anyone reading the original MATLAB `.m` file.
