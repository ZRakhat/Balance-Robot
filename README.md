# Two-Wheeled Self-Balancing Robot

This Arduino-based project implements a **two-wheeled self-balancing robot** using a PID controller, an IMU for orientation, and a TLE94112 motor driver. The robot constantly adjusts its motor speeds to maintain an upright position based on sensor feedback.

---

## 🛠️ Hardware Requirements

| Component | Description |
|----------|-------------|
| **Arduino-compatible board** | e.g., Arduino Nano 33 BLE |
| **IMU Sensor** | Bosch BMI270 + BMM150 combo (via `Arduino_BMI270_BMM150` library) |
| **Motor Driver** | Infineon TLE94112EL shield or breakout board |
| **2 DC Motors with wheels** | Matched motors for differential drive |
| **Power Supply** | Suitable for your motors (e.g., 7.4V LiPo) |
| **Chassis** | Custom or prebuilt balancing robot frame |
| **Optional** | LED indicators, power switch for motor enable |

---

## 🧠 Features

- Real-time **PID control** for balance correction
- **Sensor fusion** using a complementary filter
- Low-level **motor control via TLE94112 registers**
- **Serial output** for monitoring angle and PID output

---

## 🧪 How It Works

1. IMU provides acceleration and gyroscope data.
2. A **complementary filter** fuses data into a stable pitch angle.
3. A **PID algorithm** computes how much correction is needed.
4. Motors are driven to counteract tilt and maintain balance.
5. Loop runs approximately every 10 ms.

---

## 🔧 Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/self-balancing-robot.git
   cd self-balancing-robot
   ```

2. **Install required Arduino libraries**:
   - `Arduino_BMI270_BMM150`
   - `tle94112-ino`

3. Open the `.ino` file in the Arduino IDE and upload it to your board.

---

## ⚙️ PID Tuning

Adjust these constants in the code:

```cpp
float Kp = 4.4;   // Proportional
float Ki = 0.71;  // Integral
float Kd = 0.08;  // Derivative
```

Start with tuning `Kp` (responsiveness), then add `Kd` (stability), and finally adjust `Ki` (error correction over time).

---

## 🔌 Wiring Overview

- **IMU**: Connect to I2C (typically SDA = A4, SCL = A5 on Arduino Nano)
- **Motor Driver**:
  - Controlled via pins `3` and `4`
  - Pin `4` enables the TLE94112's multi-halfbridge logic
- Ensure motors are wired identically so forward motion is balanced.

---

## 📝 Notes

- Robot must be **manually stabilized** during startup.
- Keep the **center of gravity low**.
- Motor output is limited to ±100 PWM units.
- Add safety features like kill switch or fall detection for robustness.

---

## 📄 Getting in touch

For any discussions and issues please reach me by email

---

## 👤 Author

Created by [ZRakhat](https://www.github.com/ZRakhat) with the help of Infineon Company
