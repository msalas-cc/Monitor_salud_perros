// This #include statement was automatically added by the Particle IDE.
#include <Particle-GPS.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_DHT.h>
#include <SparkFunMAX17043.h>
#include <blynk.h>


// DHT parameters
#define DHTPIN 2
#define DHTTYPE DHT11
#define lm36 A5
#define lm36_2 A4

// ad8232 ports
#define ouput_pin A1
#define LO_minus 8
#define LO_plus 9


SYSTEM_THREAD(ENABLED);  //run the cloud processing in a separate system thread
SYSTEM_MODE(AUTOMATIC);
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

// Create a Gps instance. The RX an TX pins are connected to the TX and RX pins on the argon (Serial1).
Gps _gps = Gps(&Serial1);

// Create a timer that fires every 1 ms to capture incoming serial port data from the GPS.
Timer _timer = Timer(1, onSerialData);

//Authorization character to run Blynk
char auth[] = "KuXk-2Gq_lLnKbvwDPYEGM_FY27ixaeF";

//DHT sensor
DHT dht(DHTPIN, DHTTYPE);

//connects the virtual 0 pin on the Blynk app with the map 
WidgetMap myMap(V0);

BlynkTimer timertemp;

WidgetLCD lcd(V5);

void setup() {

    pinMode(lm36, INPUT);
    pinMode(lm36_2, INPUT);
    Blynk.begin(auth);
    dht.begin();
    Serial1.begin(115200);
    //initialize the GPS.
    _gps.begin(115200);
    // Start the timer.
    _timer.start();
    lcd.clear(); //Use it to clear the LCD Widget
    timertemp.setInterval(10000, DogEnvTemperature);  //read the env temp each 10s
    timertemp.setInterval(10000, DogBodyTemperature);  //read the dog temp each 10s
}

void onSerialData() {
    _gps.onSerialData();
}

void loop() {
    Blynk.run();
    timertemp.run();
    myPetGPS();
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
        float latitude = (strfltlatitude/100) + 0.0344; // adjusting location
        float longitude = (strfltlongitude/-100) - 0.04375;
    
        myMap.location(0, latitude, longitude, "DogLocation");  //send lat and long to the app

    /*if ((millis()-lastPublish > delayMinutes*1*1000)) { 
        lastPublish = millis();
        Particle.publish("Latitue", String(latitude), PRIVATE);
        Particle.publish("Longitude", String(longitude), PRIVATE);
        
    } // used for debug purposes*/
	}

    
}
    
void BatteryLife() {
    voltage = analogRead(BATT) * 0.0011224; // The constant 0.0011224 is based on the voltage divider circuit (R1 = 806K, R2 = 2M) 
    percentage = (voltage/3.7)*100;
    if(percentage>100){
        percentage =100;
    }
    Blynk.virtualWrite(3, percentage);
    //Blynk.notify("Battery is full");

    if(percentage < 10){
        Blynk.notify("Your iPet device is under 10% of battery life");
    }
}

void DogEnvTemperature() {
  
    double  analogValue2 = analogRead(lm36_2);
    float voltage = analogValue2 * 3.3;
    voltage /= 4095.0; 
    float temperature = ((voltage - 0.5) * 100) ; 

    Blynk.virtualWrite(2, temperature);
    
	if(temperature > 29.5){
		Blynk.notify("The enviromental temperature is hot for your dog");
    }
    else if(temperature < 10){
        Blynk.notify("The enviromental temperature is cold for your dog");
    }
   
       /*if ((millis()-lastPublish > delayMinutes*1*1000)) { 
        lastPublish = millis();
        Particle.publish("env", String(temperature), PRIVATE);} //Used for Debug*/
}

void DogBodyTemperature() {
    double  analogValue = analogRead(lm36);
    float voltage = analogValue * 3.3;
    voltage /= 4095.0; 
    float Dogtemperature = ((voltage - 0.5) * 100) ; 
    
    Blynk.virtualWrite(1, Dogtemperature);

    if(temperatureC > 39){
        Blynk.notify("Your dog body temperature is higher than usual and should be check");
    }
    else if(temperatureC < 37.5){
        Blynk.notify("Your dog body temperature is lower than usual and should be check");
    }
    
    /*if ((millis()-lastPublish > delayMinutes*1*1000)) { 
        lastPublish = millis();
        Particle.publish("dog", String(temperatureC), PRIVATE);} //Used for Debug*/
    
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
		
		if(reading<3700){
			BPM = 0;
		}
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

