
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("blink on");
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("blink off");
    delay(500);
}
