# Precision_Guitar_Tuner using STM32

This project is a **fully register-level** STM32 Guitar Tuner, **featuring hand-written LCD & GPIO drivers**, and an analog front-end using a 4-pole Butterworth LPF and level shifting stage to safely sample the Electric guitar signals. Frequency detection is performed via CMSIS-DSP’s fast FFT pipeline to determine the played note in real time.
<p align="center">
  <img src="https://github.com/user-attachments/assets/01630834-6565-4276-9bb9-1619b33940d3" width="250" style="border-radius:12px;">
</p>

---

##  Features

- Real-time audio sampling using **ADC + Timer Trigger**  
- **Hanning Windowing** for smooth FFT performance  
- Frequency detection using **CMSIS DSP’s `arm_rfft_fast_f32()`**  
- LCD Display shows:
- Detected frequency  
- Closest standard guitar note  
- Whether the string is **High / Low / In Tune**  
- User button (PC13) starts each measurement  
- Optional **test sine-wave generator** for simulation (no hardware required)

---

##  System Overview
[Electric Guitar Jack]
│
>
[ ADC (PA0) ] ← Triggered by Timer2 TRGO @ 8 kHz
│
>
[ FFT Processing using CMSIS DSP ]
│
>
[ Note Detection + LCD Display ]


---

##  Hardware Setup

| Component | Description | Pin Connections |
|------------|--------------|-----------------|
| **STM32F446xx** | Main MCU | — |
| **Electret Guitar Jack** | Analog electric input signal | **PA0 (ADC Channel 0)** |
| **LCD (16x2 HD44780)** | Note & frequency display | Controlled via `lcd.h` driver |
| **Push Button** | Start sampling trigger | **PC13 (EXTI line)** |
| **Onboard LED** | Sampling indicator | **PA5** |

**Power Supply:** 3.3 V (from USB or external regulator)

---

##  Software Modules

| Function | Description |
|-----------|-------------|
| `ADC_Init()` | Configures ADC1 on PA0, triggered by Timer2 TRGO |
| `TIM2_Init()` | Sets up Timer2 for 8 kHz sample trigger |
| `Button_Init()` | Configures PC13 as EXTI interrupt (falling edge) |
| `Capture_Samples_swstart()` | Captures 1024 samples using software trigger |
| `process_buffer()` | Applies Hanning window, performs FFT, finds dominant frequency, matches closest note |
| `Start_Conversion()` / `Stop_Conversion()` | Enables/disables ADC-Timer sync for sampling |
| `fill_adc_buffer_with_sine()` | (Optional) Generates test sine wave in software for debugging |

---

##  Guitar Notes Reference (Standard Tuning)

| String | Note | Frequency (Hz) |
|---------|------|----------------|
| 6 | E2 | 82.41 |
| 5 | A2 | 110.00 |
| 4 | D3 | 146.83 |
| 3 | G3 | 196.00 |
| 2 | B3 | 246.94 |
| 1 | E4 | 329.63 |

---

##  FFT Processing Logic

1. **Apply Hanning Window:**  
   Smooths discontinuities at the edges of the sample buffer.
   ```c
   windowed_buffer[i] = adc_buffer[i] * (0.5f - 0.5f * cosf(2 * PI * i / (BUFFER_SIZE - 1)));

2. **Perform FFT**
   arm_rfft_fast_init_f32(&fft_inst, BUFFER_SIZE);
   arm_rfft_fast_f32(&fft_inst, windowed_buffer, fft_real, 0);


3. **Find Peak Magnitude**
   The bin with maximum magnitude corresponds to the dominant frequency.
   
4. **Calculate frequency:**
   detected_freq = peak_index * (SAMPLING_RATE / BUFFER_SIZE);

5. **Compare with Guitar Notes:**
   Finds the closest frequency and determines if the string is HIGH, LOW, or IN TUNE.

---

##  Development Setup
**Requirements**

1. STM32F446xx board (e.g., Nucleo-F446RE)
2. STM32CubeIDE or Keil uVision
3. CMSIS-DSP library (included with STM32CubeIDE)
4. LCD and GPIO drivers(built my own lcd & gpio drivers from scratch)(lcd.h, lcd.c)
5. arm_math.h (from CMSIS-DSP)
6. Guitar(electric) (0-3.3 V signal range)
7. Analog Front end circuit consisting of a 4th order Butterworth LPF, and a Level shifter circuit to obtain 0V-3.3V operation.

---

## **How It Works**

- On button press (PC13), an EXTI interrupt sets a flag.
- The main loop starts ADC sampling (Timer2-triggered or software).
- 1024 samples are collected at 8 kHz rate.
- FFT is applied to determine the dominant frequency.
- The closest note and tuning status are shown on the LCD.


<p align="center">
  <video src="https://github.com/user-attachments/assets/09bb81d4-c76f-45c5-a8a2-ff6248709606" width="350" controls>
  </video>
</p>



---

## **Example LCD Output**
*Note: A2  F:110Hz*
*Diff:+0.4Hz HIGH*
<p align="center">
  <img src="https://github.com/user-attachments/assets/0b4526ac-7d03-4009-8ae6-fa54c06ac23e" width="250">
</p>


---

## **Debug Mode (Software Test)**

You can test the FFT without hardware by generating a sine wave:
```c
fill_adc_buffer_with_sine(246.94f, 0.5f, 8000.0f); // Simulate B3
process_buffer();
```
---

## **Future Improvements**

- Add DMA-based ADC sampling 
- Implement auto-gain / signal detection
- Show real-time frequency bar or needle or waveform on the LCD
- Implementing Distorion in Digital Guitar Processing
- Support for other instruments

---

Developed using STM32F446, CMSIS-DSP, and HD44780 LCD.
Ideal for learning DSP, ADC sampling, and signal processing on microcontrollers.
