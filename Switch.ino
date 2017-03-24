int R;
int i,mode;

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
  digitalWrite(A0,HIGH);
}

void loop() {
  mode=0;
  R=analogRead(A0);
  Serial.println(R);
//  R/=22;
//  mode=R;
  for(i=0;i<140;i+=20){
    if((i-20)<R&&R<i){
      mode=i/18;
      delay(100);
    }
  }  
  delay(100);
  Serial.print("Mode: ");
  Serial.println(mode);
}
