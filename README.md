# Tremor-Stabilizing Robot Arm

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![License](https://img.shields.io/badge/License-Apache_2.0-green)

## 📖 Introduction
This prototype aims to alleviate the suffering of tremor among the elderly by creating a self-balancing robot arm. It incorporates AI visual recognition (Computer Vision) to perform actions that require accuracy and control, compensating for involuntary movements.

## 🎥 Demo
![Stabilization + Grip Demo](media/demo_vid.gif)
* **To run the script:** py main.py
* **To close the script:** press q

## ⚙️ Features
* **Active Stabilization:** Uses IMU sensor data to counteract tremors in real-time.
* **Computer Vision:** Powered by **YOLO** and **MediaPipe** for object detection and hand tracking.
* **Serial Communication:** Communicates with microcontrollers (Arduino/ESP32) via `pyserial`.


## 🧩 Dependencies & Licenses

This project relies on the open-source community. Below is a list of the key libraries used and their respective licenses.

### Core Computer Vision & AI
| Library | Purpose | License |
| :--- | :--- | :--- |
| **[MediaPipe](https://github.com/google/mediapipe)** | Hand & Gesture Recognition | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0) |
| **[Ultralytics (YOLO)](https://github.com/ultralytics/ultralytics)** | Object Detection Models | [AGPL-3.0](https://www.gnu.org/licenses/agpl-3.0.html) |
| **[OpenCV](https://github.com/opencv/opencv-python)** | Image Processing | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0) |
| **[cvzone](https://github.com/cvzone/cvzone)** | CV Helper Functions | [MIT](https://opensource.org/licenses/MIT) |
| **[PyTorch](https://github.com/pytorch/pytorch)** | Deep Learning Framework | [BSD-Style](https://github.com/pytorch/pytorch/blob/main/LICENSE) |

### Math & Signal Processing
| Library | Purpose | License |
| :--- | :--- | :--- |
| **[NumPy](https://github.com/numpy/numpy)** | Array Computing | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) |
| **[SciPy](https://github.com/scipy/scipy)** | Scientific Computing | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) |
| **[FilterPy](https://github.com/rlabbe/filterpy)** | Kalman Filtering | [MIT](https://opensource.org/licenses/MIT) |

### Utilities & Hardware
| Library | Purpose | License |
| :--- | :--- | :--- |
| **[pySerial](https://github.com/pyserial/pyserial)** | UART/Serial Communication | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) |
| **[lapx](https://github.com/cafebazaar/lapx)** | Linear Assignment (Tracking) | [MIT](https://opensource.org/licenses/MIT) |
| **[Hydra](https://github.com/facebookresearch/hydra)** | Configuration Management | [MIT](https://opensource.org/licenses/MIT) |

## 📚 References
* **Gimbal Tutorial:** [Arduino and MPU6050 Accelerometer and Gyroscope Tutorial](https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/)
