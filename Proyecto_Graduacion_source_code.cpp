// This #include statement was automatically added by the Particle IDE.
#include <Particle-GPS.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_DHT.h>
#include <SparkFunMAX17043.h>
#include <blynk.h>


// DHT parameters
#define DHTPIN 2
#define DHTTYPE DHT11
#define lm35 A5


// ad8232 ports
#define ouput_pin A1
#define LO_minus 8
#define LO_plus 9


SYSTEM_THREAD(ENABLED);  //run the cloud processing in a separate system thread

//Global variables

//String inWord;
//char inByte;
String data,latitude,longitude;
float lat,lng,voltage;
int lastPublish=0;
int percentage;
int PUBLISH_PERIOD =1;
int temperature;
int delayMinutes =1;
double dogTemp =0.0;
int UpperThreshold = 1023;
int LowerThreshold = 620;
int reading = 0;
float BPM = 0.0;
bool IgnoreReading = false;
bool FirstPulseDetected = false;
unsigned long FirstPulseTime = 0;
unsigned long SecondPulseTime = 0;
unsigned long PulseInterval = 0;

//create a gps instance
Gps _gps = Gps(&Serial1);

//create a timer that fires every 1 ms to capture incoming serial port data from the gps.
Timer _timer = Timer(1, onSerialData);

//Authorization character to run Blynk
char auth[] = "QygpAoLuJQg73f7cW6cPFhtau9n4agpp";

//DHT sensor
DHT dht(DHTPIN, DHTTYPE);

//connects the virtual 0 pin on the Blynk app with the map 
WidgetMap myMap(V0);


void setup() {
    // cloud variable
    Particle.variable("STU", data);
    pinMode(lm35, INPUT);
    Blynk.begin(auth);
    dht.begin();
    Serial1.begin(9600);
    
    //initialize the GPS.
    _gps.begin(9600);

    //start the timer.
    _timer.start();

}

void onSerialData() {
    _gps.onSerialData();
}

void loop() {
    Blynk.run();
    myPetGPS();
    DogEnvTemperature();
    DogBodyTemperature();
    DogHeartRate();
    BatteryLife();
} 


void myPetGPS(){
    Rmc rmc = Rmc(_gps);
        if (rmc.parse()){
            String myLat = rmc.latitude;
            String myLong = rmc.longitude;
    
            float strfltlatitude = myLat.toFloat();  // converting string to float
            float strfltlongitude = myLong.toFloat();
            float latitude = ((strfltlatitude/100) + 0.0344); // adjusting location
            float longitude = ((strfltlongitude/-100) - 0.04375);
    
            myMap.location(0, latitude, longitude, "DogLocation");  //send lat and long to the app

    /*if ((millis()-lastPublish > delayMinutes*1*1000)) { 
        lastPublish = millis();
        Particle.publish("Latitue", String(latitude), PRIVATE);
        Particle.publish("Longitude", String(longitude), PRIVATE);
        
    }*/ // used for debug purposes
  }

    
}
    
void BatteryLife() {
    voltage = analogRead(BATT) * 0.0011224; // The constant 0.0011224 is based on the voltage divider circuit (R1 = 806K, R2 = 2M) 
    percentage = (voltage/3.7)*100;
    Blynk.virtualWrite(3, percentage);
    //Blynk.notify("Battery is full");
    
    if(percentage < 10){
        Blynk.notify("Your iPet device is under 10% of battery life");
    }
}

void DogEnvTemperature() {
    temperature = dht.getTempCelcius();
    Blynk.virtualWrite(2, temperature);
    
    if(temperature > 30){
        Blynk.notify("The enviromental temperature is hot for your dog");
    }
    else if(temperature < 10){
        Blynk.notify("The enviromental temperature is cold for your dog");
    }
  
}

void DogBodyTemperature() {
    double  analogValue = analogRead(lm35);
    double tempvoltage = (3.3 * analogValue / 5000); //4095
    dogTemp =(tempvoltage * 100);
    String tempDog = String(dogTemp);
    tempDog = tempDog.substring( 0, 4);
    Blynk.virtualWrite(1, tempDog);
    
    if(temperature > 39){
        Blynk.notify("Your dog body temperature is higher than usual and should be check");
    }
    else if(temperature < 37.5){
        Blynk.notify("Your dog body temperature is lower than usual and should be check");
    }

}

void DogHeartRate() {
    if ((millis()-lastPublish > delayMinutes*1*1000)) {
        lastPublish = millis();
        reading = analogRead(ouput_pin); 

        // Heart beat leading edge detected.
        if(reading > UpperThreshold && IgnoreReading == false){
            if(FirstPulseDetected == false){
                FirstPulseTime = millis();
                FirstPulseDetected = true;
            }
        else{
          SecondPulseTime = millis();
          PulseInterval = SecondPulseTime - FirstPulseTime;
          FirstPulseTime = SecondPulseTime;
        }
        IgnoreReading = true;
      }

      // Heart beat trailing edge detected.
        if(reading < LowerThreshold){
            IgnoreReading = false;
      }  

    BPM = (1.0/PulseInterval) * 60.0 * 1000;
    Blynk.virtualWrite(4, BPM);
    //Particle.publish("BPM_calc", String(BPM), PRIVATE);  // used for debug
    //Particle.publish("analog", String(reading), PRIVATE);
    
    if(BPM > 100){
        Blynk.notify("Your dog heart rate is higher than usual and should be check");
    }
    else if(BPM < 60){
        Blynk.notify("Your dog hear rate is lower than usual and should be check");
    }

 }
 
}