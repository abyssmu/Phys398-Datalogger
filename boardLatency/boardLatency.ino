void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float avg = 0.0;
  int sum = 0;
  int tot = 5000;
  
  for(int i = 0; i < tot; ++i)
  {
    int start = micros();
    int end = micros();
    sum += end - start;
  }
  
  avg = float(sum) / float(tot);
  Serial.println(avg);

  delay(100);
}
