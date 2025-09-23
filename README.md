# ControlClock_NucleoSTM32F0Series_PlatformIO
Bare-metal firmware for STM32F072RB (Nucleo board).  
This project demonstrates:

- System clock config (PLL → 48 MHz from HSI)
- GPIO LED blink (PA5)
- Timer2 as up-counter (millis() function, overflow handling)
- UART2 serial communication (PA2/PA3)
- Serial printing of different data types

---

## Build
```bash
pio run
