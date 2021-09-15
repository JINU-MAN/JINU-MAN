#define PIN7 7

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN7, LOW);
  delay(1000);
  
  digitalWrite(PIN7, HIGH);
  delay(200);
  digitalWrite(PIN7, LOW);
  delay(200);
  digitalWrite(PIN7, HIGH);
  delay(200);
  digitalWrite(PIN7, LOW);
  delay(200);
  digitalWrite(PIN7, HIGH);
  delay(200);
  digitalWrite(PIN7, LOW);
  delay(200);
  digitalWrite(PIN7, HIGH);
  delay(200);
  digitalWrite(PIN7, LOW);
  delay(200);
  digitalWrite(PIN7, HIGH);
  delay(200);
  digitalWrite(PIN7, LOW);
  delay(200);

  while(1){
    digitalWrite(PIN7, HIGH);
  }

}
