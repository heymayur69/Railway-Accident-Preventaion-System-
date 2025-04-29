Railway Accident Prevention System
Project Overview
This project is a railway accident prevention system. 🚆 It has two main parts:
Train Unit (ESP32): Mounted on the model train. It uses sensors (LiDAR, ultrasonic, IR) to spot obstacles or track problems.
Signal Unit (Arduino Uno): Controls the crossing. It has a red/green LED, a button (to change the signal), and a servo for the gate. When the light is red, it sends an IR beam to the train.
When the train unit sees an obstacle or the red signal, it stops the train. When the track is clear, the train keeps running. 🔧
Installation and Usage
Follow these steps to set up the project:
Clone the Repository: Open a terminal or command prompt and run:
bash
Copy
Edit
git clone https://github.com/YourUsername/RailwayAccidentPreventionSystem.git
Replace YourUsername with your GitHub username.
Install Arduino IDE: Download and install the Arduino IDE if you don't have it.
Set up Arduino IDE:
Install board support for the ESP32. In Arduino IDE, go to Tools > Board > Boards Manager, search for ESP32 and install it.
Go to Sketch > Include Library > Manage Libraries. Install these libraries:
Adafruit VL53L0X (for the LiDAR sensor)
IRremote (for infrared communication)
Servo (for controlling the gate)
BluetoothSerial (for ESP32 Bluetooth, usually included with the ESP32 core)
Open and Upload Code:
In Arduino IDE, open ON_TRAIN_UNIT_CODE.txt (you can rename it to .ino).
Select Tools > Board > ESP32 Dev Module.
Select the correct COM port for your ESP32.
Click Upload to program the ESP32.
Open SIGNAL_POST_AND_LEVEL_CROSSING_UNIT_CODE.txt in Arduino IDE.
Select Tools > Board > Arduino Uno.
Select the COM port for your Uno.
Click Upload to program the Arduino Uno.
Power Up and Test: Connect power to both boards. Use the Serial Monitor or a Bluetooth terminal app to see status messages from the ESP32. Press the button on the signal unit to toggle the red/green light and open/close the gate. The train unit will stop or go based on what it detects (obstacles or red signal). ✅
Connection Diagram
Here is a simple connection diagram for both units. The ESP32 (train) has sensors and a relay to control the train motor. The Arduino Uno (signal post) has LEDs, a button, a servo, and an IR LED. The IR LED on the signal unit sends a signal to the IR receiver on the train unit.
mermaid
Copy
Edit
graph LR
    subgraph "Train Unit (ESP32)"
        IRrecv[IR Receiver]
        LIDAR[LiDAR (VL53L0X)]
        ULTRA[Ultrasonic]
        BTN[Button (Start/Stop)]
        RELAY[Relay (Train Motor)]
        BUZZ[Buzzer]
        LED[Status LED]
    end
    subgraph "Signal Unit (Arduino Uno)"
        RedLED[Red LED (Stop)]
        GreenLED[Green LED (Go)]
        BTN2[Button (Signal Toggle)]
        Gate[Servo (Gate)]
        IRled[IR LED (Transmitter)]
    end
    IRled --> IRrecv
Directory Structure & Code Samples
Your project files should look like this:
bash
Copy
Edit
📂 RailwayAccidentPreventionSystem
├── ON_TRAIN_UNIT_CODE.txt                   # ESP32 code (train unit)
├── SIGNAL_POST_AND_LEVEL_CROSSING_UNIT_CODE.txt   # Arduino Uno code (signal unit)
└── assets/                                  # Folder for images/videos (e.g., diagrams or demo videos)
Open ON_TRAIN_UNIT_CODE.txt in Arduino IDE. It includes the necessary libraries and sets up the ESP32. For example, at the top of this file:
cpp
Copy
Edit
#include <Adafruit_VL53L0X.h>
#include <IRremote.h>
#include "BluetoothSerial.h"

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RailwayTrain");
  // Initialize sensors and pins here
}
Open SIGNAL_POST_AND_LEVEL_CROSSING_UNIT_CODE.txt in Arduino IDE. For example:
cpp
Copy
Edit
#include <IRremote.h>
#include <Servo.h>

Servo gateServo;
void setup() {
  Serial.begin(9600);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  // Initialize LEDs and servo here
}
These files contain the code that runs on each board.
Images and Video Embedding
You can add images or videos by placing them in the assets/ folder of your repo. Then link them in Markdown:
For an image (e.g., a wiring diagram):
markdown
Copy
Edit
![Wiring Diagram](assets/connections_esp32.png)
For a video (e.g., demo), use a thumbnail image that links to the video file:
markdown
Copy
Edit
[![Demo Video](assets/demo_thumbnail.png)](assets/railway_demo_video.mp4)
The first example displays connections_esp32.png. The second shows demo_thumbnail.png; clicking it will play railway_demo_video.mp4. You can also link to external videos (like YouTube) in a similar way.
Step-by-Step Setup
👤 Create a GitHub account if you don't have one.
💾 Install Git on your computer (see git-scm.com for downloads).
📦 Clone the repo:
bash
Copy
Edit
git clone https://github.com/YourUsername/RailwayAccidentPreventionSystem.git
🗂 Navigate into the project folder:
bash
Copy
Edit
cd RailwayAccidentPreventionSystem
📁 Add images/videos: Put any wiring diagrams or demo videos into the assets/ folder. Then commit and push these files to GitHub:
bash
Copy
Edit
git add assets/
git commit -m "Add diagrams and videos"
git push
💻 Open code in Arduino IDE: Copy the code from the .txt files into new sketches (or rename them to .ino).
🔧 Upload code to boards:
Select ESP32 Dev Module and upload the train unit code to the ESP32.
Select Arduino Uno and upload the signal unit code to the Arduino.
🔗 Wire your components: Connect LEDs, sensors, the servo, and other parts to the correct pins (as shown in the diagram above or pin list).
▶️ Power up and test: Start running both boards. The LEDs should indicate the signal. Press the button to change the gate status. The train should stop if it sees an obstacle or a red signal.
👍 You are all set! Now enjoy your Railway Accident Prevention System. 😊