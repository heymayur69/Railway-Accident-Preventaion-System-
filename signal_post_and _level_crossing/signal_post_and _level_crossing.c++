#include <IRremote.h>
#include <Servo.h>

// ───── Pin Definitions ─────
const uint8_t RED_LED_PIN        = 2;
const uint8_t GREEN_LED_PIN      = 3;
const uint8_t BUTTON_PIN         = 4;
const uint8_t SERVO_PIN          = 5;
const uint8_t IR_LED_PIN         = 6;

// ───── IR & Timing Constants ─────
const uint32_t IR_CODE           = 0x00FF55AA;   // Standard NEC code for address 0x00, command 0x55
const uint8_t  INITIAL_BURSTS    = 3;        // send bursts on toggle
const uint16_t BURST_DELAY_MS    = 40;       // ms between initial bursts
const uint16_t REPEAT_INTERVAL_MS= 100;      // ms between keep-alive bursts

// ───── State Variables ─────
Servo      gateServo;
IRsend     irsend(IR_LED_PIN);  // specify the IR LED pin
bool       isRed       = false;
bool       lastBtn     = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(RED_LED_PIN,    OUTPUT);
  pinMode(GREEN_LED_PIN,  OUTPUT);
  pinMode(BUTTON_PIN,     INPUT_PULLUP);
  gateServo.attach(SERVO_PIN);

  // Start in GREEN state
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN,   LOW);
  gateServo.write(0);
  Serial.println("▶ Signal Controller Ready — GREEN");
}

void loop() {
  // ─── Button Toggle with 50 ms Debounce ───
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == LOW && lastBtn == HIGH) {
    delay(50);  // debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      toggleState();
    }
  }
  lastBtn = btn;

  // ─── Keep-alive bursts while RED ───
  static uint32_t lastTx = 0;
  if (isRed && (millis() - lastTx >= REPEAT_INTERVAL_MS)) {
    irsend.sendNEC(IR_CODE, 32);  
    lastTx = millis();
    Serial.println("🔁 RED keep-alive burst");
  }
}

void toggleState() {
  isRed = !isRed;
  if (isRed) {
    // → Enter RED
    digitalWrite(RED_LED_PIN,   HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    gateServo.write(90);
    Serial.println("🔴 RED: Gate Closed");

    // Send a few immediate bursts for fast detection
    for (uint8_t i = 0; i < INITIAL_BURSTS; i++) {
      irsend.sendNEC(IR_CODE, 32);
      Serial.print("   → Burst ");
      Serial.print(i+1);
      Serial.print("/");
      Serial.println(INITIAL_BURSTS);
      delay(BURST_DELAY_MS);
    }
  } else {
    // → Return to GREEN
    digitalWrite(RED_LED_PIN,   LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    gateServo.write(0);
    Serial.println("🟢 GREEN: Gate Open");
  }
}