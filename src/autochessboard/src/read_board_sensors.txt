void readDigits(int[]);
bool compareArrays(int[], int[]);
void setOldDigits(int[], int[]);
void printDigits(int[]);


//PL pin 1
const int load = 2;
//Ce pin 15
const int clockEnablePin = 3;
//Q7 pin 7
const int dataIn = 4;
//CP pin 2
const int clockIn = 5;
const int button = 6;
const int numBits = 64;
int oldDigits[numBits];
int curDigits[numBits];
int gradient[numBits];




//connect pins 1,2,15 in parallel
//connect pin 9 -> 10
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  pinMode(load, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(dataIn, INPUT);
  pinMode(clockIn, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  for(int i = 0; i < numBits; ++i){
    oldDigits[i] = 1;
  }
  //Serial.println("Loading Board...");
  //Serial.println("");
  readDigits(curDigits);
  //printDigits(curDigits);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(button) == HIGH) {
    delay(100);    
  } else {
    readDigits(curDigits);
    if(!compareArrays(curDigits, oldDigits)){
      //printDigits(curDigits);
      //printDigits(gradient);
      computeGradient(curDigits, oldDigits, gradient);
      setOldDigits(curDigits, oldDigits);
      sendSerial(curDigits);
      sendSerial(gradient);
    }
    delay(1000);
  }
}

void computeGradient(int current[], int old[], int gradient[]) {
  for (int i = 0; i < numBits; i++) {
    // Add one so that when we serialize we send only positive integers
    gradient[i] = current[i] - old[i] + 1;
  }
}

void readDigits(int arr[]){
  digitalWrite(load, LOW);
  delay(5);
  digitalWrite(load, HIGH);
  delay(5);

  for(int i = 0; i < numBits; i++){
    int value = digitalRead(dataIn);
    arr[i] = value;
    digitalWrite(clockIn, LOW);
    digitalWrite(clockIn, HIGH);
  }
  delay(5);
}

bool compareArrays(int arr1[], int arr2[]){
  for(int i = 0; i < numBits; ++i){
    if(arr1[i] != arr2[i]){
      /*
      if((arr1[i] == 0) && (arr2[i] == 1)){
        Serial.print("piece removed from spot: ");
        Serial.println(i);
      }
      else if((arr1[i] == 1) && (arr2[i] == 0)){
        Serial.print("piece placed at spot: ");
        Serial.println(i);
      }
      */
      return false;
    }
  } 
  return true;
}

void setOldDigits(int curDigits[], int oldDigits[]){
  for(int i = 0; i < numBits; i++){
    oldDigits[i] = curDigits[i];
  }
}

void printDigits(int digits[]){
  for(int i = 63; i >= 0; i){
    for(int j = 7; j >= 0; --j){
      Serial.print(digits[i - j]);
    }
    Serial.println("");
    i = i - 8;
  }
  Serial.println("");
  Serial.println("");
}

void sendSerial(int digits[]){
  char str_buf[numBits];
  for (int i = 0; i < numBits; i++) {
    str_buf[i] = digits[i] + '0';
  }
  Serial.write(str_buf, numBits);
}
