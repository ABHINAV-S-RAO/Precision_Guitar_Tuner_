# Precision_Guitar_Tuner
# 🎸 STM32 Guitar Tuner (FFT-Based)

An **embedded guitar tuner** project built using **STM32F446xx**, **ADC sampling**, and **CMSIS-DSP FFT** to detect guitar string frequencies in real time.  
It samples analog audio from a microphone or pickup, performs a **Fast Fourier Transform (FFT)** to find the fundamental frequency, and displays the **closest note** and tuning direction on an **LCD**.

---

## 🚀 Features

✅ Real-time audio sampling using **ADC + Timer Trigger**  
✅ **Hanning Windowing** for smooth FFT performance  
✅ Frequency detection using **CMSIS DSP’s `arm_rfft_fast_f32()`**  
✅ LCD Display shows:
- Detected frequency  
- Closest standard guitar note  
- Whether the string is **High / Low / In Tune**  
✅ User button (PC13) starts each measurement  
✅ Optional **test sine-wave generator** for simulation (no hardware required)

---

## 🧠 System Overview

