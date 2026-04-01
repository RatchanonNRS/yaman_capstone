// Mock bidirectional serial test
// - Sends O: lines at 50Hz (like real firmware)
// - Receives SEQ:START, replies with SEQ:STEP:1..3 then SEQ:DONE
// - Echoes any unknown command back for debugging

unsigned long lastOdom = 0;
unsigned long stepTimer = 0;
bool seqRunning = false;
int seqStep = 0;
String inputBuffer = "";

void setup() {
  Serial.begin(115200);
}

void loop() {
  // --- Send O: at 50Hz ---
  if (millis() - lastOdom >= 20) {
    lastOdom = millis();
    Serial.println("O:0.0000,0.0000,0.0000,0.0000,0.0000");
  }

  // --- Read incoming commands ---
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        if (inputBuffer == "SEQ:START") {
          seqRunning = true;
          seqStep = 1;
          stepTimer = millis();
          Serial.println("SEQ:STEP:1:retrieve_box");
        } else if (inputBuffer.startsWith("V:")) {
          // ignore velocity commands silently
        } else {
          Serial.print("UNKNOWN:");
          Serial.println(inputBuffer);
        }
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // --- Mock sequence progression (1s per step) ---
  if (seqRunning && millis() - stepTimer >= 1000) {
    stepTimer = millis();
    seqStep++;
    if (seqStep == 2) Serial.println("SEQ:STEP:2:pick_medicine");
    else if (seqStep == 3) Serial.println("SEQ:STEP:3:return_box");
    else if (seqStep >= 4) {
      Serial.println("SEQ:DONE");
      seqRunning = false;
      seqStep = 0;
    }
  }
}
