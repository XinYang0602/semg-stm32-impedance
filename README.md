🧠 Real-Time Signal Quality Monitoring for Robust sEMG Gesture Recognition

ECE 6781 – Biomedical Instrumentation | Georgia Institute of Technology

A real-time single-channel sEMG system that integrates active electrode–skin impedance monitoring and adaptive bandwidth analog filtering to improve robustness of gesture recognition during prolonged use.

📌 Motivation

Most low-cost sEMG systems fail in real-world use not because of weak algorithms, but due to undetected signal degradation caused by electrode displacement, perspiration, and motion artifacts.
Commercial clinical systems solve this at high cost ($20k–$100k), while consumer devices operate as opaque “black boxes” without feedback on signal reliability.

This project addresses that gap by embedding hardware-level intelligence directly into the signal acquisition chain.

🏗️ System Overview

Single-channel surface EMG (sEMG)

STM32F103 microcontroller

Custom analog front-end (AFE)

Real-time impedance monitoring

Adaptive bandwidth filtering

Host-side gesture classification

The system continuously acquires EMG signals while periodically assessing electrode contact quality, enabling fail-safe detection of degraded signal conditions before classification performance collapses.

🔬 Key Hardware Innovations
1️⃣ Active Electrode–Skin Impedance Monitoring

A GPIO-driven voltage divider injects a microampere-level excitation through a known resistor (100 kΩ).

ADC measurements estimate electrode–skin impedance in real time.

Acceptable contact range: 10–300 kΩ (empirically determined).

Out-of-range values trigger a visual LED warning and flag low-quality data.

✔ Detects loose or detached electrodes
✔ Adds <5 ms interruption to EMG streaming
✔ Typically found only in expensive clinical systems

2️⃣ Adaptive Bandwidth Analog Front-End

Two parallel bandpass paths address the tradeoff between noise suppression and spectral fidelity:

Path	Bandwidth	Purpose
Path A	20–300 Hz	Weak / fine motor gestures, improved SNR
Path B	20–450 Hz	Strong contractions, preserves high-frequency detail

Envelope detection + hardware comparator selects the appropriate path in real time.

Improves SNR by 3.5–3.7 dB for weak contractions.

Reduces motion artifacts without software overhead.

⚙️ Analog Front-End Details

Instrumentation amplifier (AD620) for differential acquisition

20 Hz high-pass + 500 Hz low-pass filtering

Active Twin-T 60 Hz notch filter

20 dB suppression of power-line interference

Envelope detector for amplitude-based bandwidth switching

💻 Embedded Firmware

MCU: STM32F103

ADC: Continuous sampling (~1.4 kHz)

UART: Real-time streaming to host

GPIO: Impedance excitation & LED warning

Design Focus: Minimal interference between EMG acquisition and diagnostics

🧠 Digital Processing & Classification

Feature extraction: MAV, RMS, Variance, WL, ZC, SSC

Windowing: 200 ms window, 100 ms step

Classifier: KNN (k = 3)

Gestures classified (5):

Fist

Palm Opening

Wave-In

Wave-Out

Rest

Accuracy: 83.3% (held-out validation)
Misclassifications primarily occur between biomechanically similar gestures.

📊 Experimental Results

SNR Improvement: +3.5 dB for weak contractions

Impedance Validation: Correctly detects detachment and contact degradation

Classification Accuracy: 83.3% with a single EMG channel

Latency Impact: Negligible (<5 ms per impedance check)

🚀 Why This Matters

This project demonstrates that hardware-level signal quality awareness is essential for reliable wearable EMG systems.
Instead of compensating for poor data after the fact, the system prevents silent failure modes by detecting degradation as it occurs.

Applications:

Rehabilitation therapy

Myoelectric prosthetics

Human–computer interaction (HCI)

Wearable biosensing research

🔮 Future Work

Flexible multi-channel electrode arrays

Wireless wearable form factor

User-specific calibration & domain adaptation

Multimodal sensing integration

Clinical validation with patient populations

🧑‍💻 Authors

Xin Yang, Jing He, Qingyuan Meng, Evan Bryan, Sishnukeshav Balamurali
Georgia Institute of Technology
ECE 6781 – Biomedical Instrumentation

📄 Reference

Full technical details, schematics, and validation results are available in the final project report (ECE 6781, Fall 2025).
