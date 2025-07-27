#  Adaptive Cruise Control using STM32F411

An embedded system that dynamically adjusts vehicle speed based on obstacle distance using an ultrasonic sensor, IR speed sensor, and PWM motor control. Built using the STM32F411E Discovery board.

## 🔧 Features

- Real-time obstacle detection with HC-SR04
- Host speed measurement via IR sensor
- Motor speed control via PWM (TIM3)
- Safe distance enforcement with LEDs and buzzer alerts

## 🛠 Hardware Used

- STM32F411E Discovery Board
- HC-SR04 Ultrasonic Sensor
- L298N Motor Driver
- DC Motors + Chassis
- IR Sensor (Wheel Speed)
- Buzzer, Red/Green LEDs
- 2× 3.7V Li-ion Batteries (~7.4V)

## ⚙️ Logic Summary

- **Distance ≥ 40 cm** → 70% speed  
- **25–40 cm** → 50% speed  
- **10–25 cm** → 30% speed  
- **< 10 cm** → Stop + Alert (Buzzer + Red LED)  

PWM signals are generated using TIM3, and ultrasonic echo capture is handled via TIM4.

## 📂 Files

- `main.c` – Contains full logic for sensor input, speed control, and alerts
- `README.md` – Project overview

## ▶️ Usage

1. Flash `main.c` to STM32F411 using STM32CubeIDE.
2. Connect hardware as per pin configuration in code.
3. Power the motors and board.
4. Observe automatic speed adjustments and alerts based on distance.

## 📄 License

Open-source, MIT License.
