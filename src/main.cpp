// #include <MPU9250_WE.h>
// #include <Wire.h>
// #include <PID_v1.h>
// #include <WiFi.h>
// #include <WiFiUdp.h>

// // --- WIFI / UDP SETTINGS ---
// const char *ssid = "Drone_ESP32";  // Name of the WiFi Network
// const char *password = "12345678"; // Password (min 8 chars)
// WiFiUDP udp;
// const int udpPort = 4210;
// char packetBuffer[255];            // Buffer to hold incoming packet

// // --- PIN DEFINITIONS ---
// const int FR_PIN = 25;
// const int FL_PIN = 32;
// const int BR_PIN = 13;
// const int BL_PIN = 18;

// // --- PWM CHANNELS ---
// const int FR_CH = 0;
// const int FL_CH = 1;
// const int BR_CH = 2;
// const int BL_CH = 3;

// // --- PWM SETTINGS ---
// const int PWM_FREQ = 5000;
// const int PWM_RES = 8;

// // --- PID VARIABLES ---
// double pitchInput, pitchOutput, pitchSetpoint;
// double rollInput, rollOutput, rollSetpoint;
// // Note: Yaw PID requires magnetometer or gyro integration logic 
// // not present in the original code, so we will use open-loop Yaw for now.
// double yawCommand = 0;

// // --- PID TUNING ---
// // Adjust these based on flight tests
// double Kp = 2.0;
// double Ki = 0.05;
// double Kd = 1.0;

// PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
// PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

// MPU9250_WE myMPU9250 = MPU9250_WE();

// // --- FLIGHT VARIABLES ---
// int baseThrottle = 0;
// unsigned long lastPacketTime = 0; // For Safety Failsafe

// void writeMotors(int fl, int fr, int bl, int br) {
// 	ledcWrite(FL_CH, constrain(fl, 0, 255));
// 	ledcWrite(FR_CH, constrain(fr, 0, 255));
// 	ledcWrite(BL_CH, constrain(bl, 0, 255));
// 	ledcWrite(BR_CH, constrain(br, 0, 255));
// }
// void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
// 	Serial.println("---------------------------------------");
// 	Serial.println(">>> NEW DEVICE CONNECTED TO WIFI! <<<");
// 	Serial.print("MAC Address: ");

// 	// Print the MAC address of the phone that connected
// 	for (int i = 0; i < 6; i++) {
// 		Serial.printf("%02X", info.wifi_ap_staconnected.mac[i]);
// 		if (i < 5) Serial.print(":");
// 	}
// 	Serial.println("\n---------------------------------------");
// }

// // This function runs when a device disconnects
// void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
// 	Serial.println("!!! DEVICE DISCONNECTED !!!");
// }

// void setup() {
// 	Serial.begin(115200);
// 	Wire.begin();

// 	// --- MOTOR SETUP ---
// 	ledcSetup(FR_CH, PWM_FREQ, PWM_RES);
// 	ledcSetup(FL_CH, PWM_FREQ, PWM_RES);
// 	ledcSetup(BR_CH, PWM_FREQ, PWM_RES);
// 	ledcSetup(BL_CH, PWM_FREQ, PWM_RES);

// 	ledcAttachPin(FR_PIN, FR_CH);
// 	ledcAttachPin(FL_PIN, FL_CH);
// 	ledcAttachPin(BR_PIN, BR_CH);
// 	ledcAttachPin(BL_PIN, BL_CH);
// 	writeMotors(0, 0, 0, 0);

// 	// --- WIFI SETUP ---
// 	Serial.println("Setting up Access Point...");
// 	WiFi.softAP(ssid, password);
// 	WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
// 	WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

// 	IPAddress IP = WiFi.softAPIP();
// 	Serial.print("AP IP address: ");
// 	Serial.println(IP); // Should be 192.168.4.1

// 	udp.begin(udpPort);
// 	Serial.println("UDP Listening...");

// 	// --- MPU SETUP ---
// 	if (!myMPU9250.init()) {
// 		Serial.println("MPU9250 does not respond");
// 	}
// 	else {
// 		Serial.println("MPU9250 connected");
// 	}

// 	Serial.println("Calibrating... Keep Flat!");
// 	delay(1000);
// 	myMPU9250.autoOffsets();
// 	Serial.println("Done!");

// 	myMPU9250.setSampleRateDivider(5);
// 	myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
// 	myMPU9250.enableAccDLPF(true);
// 	myMPU9250.setAccDLPF(MPU9250_DLPF_6);

// 	// --- PID SETUP ---
// 	pitchPID.SetMode(AUTOMATIC);
// 	rollPID.SetMode(AUTOMATIC);
// 	pitchPID.SetOutputLimits(-100, 100);
// 	rollPID.SetOutputLimits(-100, 100);
// 	pitchPID.SetSampleTime(10);
// 	rollPID.SetSampleTime(10);
// }
// void loop() {
// 	Serial.println("Looping...");
// 	delay(1000);
// 	// 1. CHECK FOR UDP COMMANDS
// 	int packetSize = udp.parsePacket();
// 	if (packetSize) {
// 		int len = udp.read(packetBuffer, 255);
// 		if (len > 0) {
// 			packetBuffer[len] = 0; // Null terminate string

// 			// --- PRINT DEBUGGING START ---
// 			Serial.print("RX Packet: ");
// 			Serial.println(packetBuffer);
// 			// --- PRINT DEBUGGING END ---

// 			// Parse String: "throttle,pitch,roll,yaw"
// 			char *ptr = strtok(packetBuffer, ",");
// 			if (ptr) {
// 				int rawThrottle = atoi(ptr);
// 				ptr = strtok(NULL, ",");
// 				int rawPitch = atoi(ptr);
// 				ptr = strtok(NULL, ",");
// 				int rawRoll = atoi(ptr);
// 				ptr = strtok(NULL, ",");
// 				int rawYaw = atoi(ptr);

// 				// -- Mapping & Scaling --
// 				baseThrottle = map(rawThrottle, 0, 1023, 0, 255);

// 				// Map Joystick (-100 to 100) to Target Angles (-30 to 30 degrees)
// 				pitchSetpoint = map(rawPitch, -100, 100, -30, 30);
// 				rollSetpoint = map(rawRoll, -100, 100, -30, 30);
// 				yawCommand = map(rawYaw, -100, 100, -50, 50);

// 				lastPacketTime = millis();
// 			}
// 		}
// 	}

// 	// 2. FAILSAFE (Safety Cutoff)
// 	if (millis() - lastPacketTime > 1000) {
// 		if (baseThrottle > 0) Serial.println("Failsafe: Signal Lost!");
// 		baseThrottle = 0;
// 		pitchSetpoint = 0;
// 		rollSetpoint = 0;
// 	}

// 	// 3. GET SENSOR DATA
// 	float pitch = myMPU9250.getPitch();
// 	float roll = myMPU9250.getRoll();

// 	// 4. CRASH PROTECTION
// 	if (abs(pitch) > 60 || abs(roll) > 60) {
// 		writeMotors(0, 0, 0, 0);
// 		return;
// 	}

// 	// 5. PID CALCULATION
// 	pitchInput = pitch;
// 	rollInput = roll;
// 	pitchPID.Compute();
// 	rollPID.Compute();

// 	// 6. MOTOR MIXING
// 	int pwm_FL = baseThrottle - pitchOutput + rollOutput + yawCommand;
// 	int pwm_FR = baseThrottle - pitchOutput - rollOutput - yawCommand;
// 	int pwm_BL = baseThrottle + pitchOutput + rollOutput - yawCommand;
// 	int pwm_BR = baseThrottle + pitchOutput - rollOutput + yawCommand;


// 	writeMotors(pwm_FL, pwm_FR, pwm_BL, pwm_BR);

// }

#include <MPU9250_WE.h>
#include <Wire.h>
#include <PID_v1.h>
// --- PIN DEFINITIONS ---
// Make sure your MOSFET Gates are connected to these pins
const int FR_PIN = 25;
const int FL_PIN = 32;
const int BR_PIN = 13;
const int BL_PIN = 18;
// --- PWM CHANNELS (ESP32 Specific) ---
const int FR_CH = 0;
const int FL_CH = 1;
const int BR_CH = 2;
const int BL_CH = 3;
// --- PWM SETTINGS ---
// 5000 Hz is good for brushed motors + MOSFETs.
// Too high (20kHz+) might overheat IRLZ44N if gate drive isn't perfect.
const int PWM_FREQ = 5000;
const int PWM_RES = 8;     // 8 bit resolution (Values 0-255)
// --- PID VARIABLES ---
double pitchInput, pitchOutput, pitchSetpoint;
double rollInput, rollOutput, rollSetpoint;
// --- PID TUNING ---
// STARTING VALUES: P=1.5, I=0.02, D=0.8
// If it wobbles fast -> Lower P
// If it drifts -> Increase I
// If it wobbles slowly -> Increase D
double Kp = 2;
double Ki = 0.05;
double Kd = 0.0;
// Initialize PID Objects
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);
// MPU Object
MPU9250_WE myMPU9250 = MPU9250_WE();
// --- FLIGHT VARIABLES ---
// WARNING: 0 = Motors OFF.
// Set this to ~50 to test stabilization in your hand.
// Max is 255.
int baseThrottle = 0;
void writeMotors(int fl, int fr, int bl, int br) {
	ledcWrite(FL_CH, fl);
	ledcWrite(FR_CH, fr);
	ledcWrite(BL_CH, bl);
	ledcWrite(BR_CH, br);
}
void setup() {
	Serial.begin(115200);
	Wire.begin();
	delay(500);
	Wire.setClock(100000);
	delay(500);
	// --- MOTOR PWM SETUP ---
	ledcSetup(FR_CH, PWM_FREQ, PWM_RES);
	ledcSetup(FL_CH, PWM_FREQ, PWM_RES);
	ledcSetup(BR_CH, PWM_FREQ, PWM_RES);
	ledcSetup(BL_CH, PWM_FREQ, PWM_RES);

	ledcAttachPin(FR_PIN, FR_CH);
	ledcAttachPin(FL_PIN, FL_CH);
	ledcAttachPin(BR_PIN, BR_CH);
	ledcAttachPin(BL_PIN, BL_CH);

	// Ensure motors are off
	writeMotors(0, 0, 0, 0);

	// --- MPU SETUP ---
	if (!myMPU9250.init()) {
		Serial.println("MPU9250 does not respond");
	}
	else {
		Serial.println("MPU9250 is connected");
	}

	// 1. SET SETTINGS FIRST
	myMPU9250.setSampleRateDivider(5);
	myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
	myMPU9250.enableAccDLPF(true);


	myMPU9250.setAccDLPF(MPU9250_DLPF_6);

	// 2. NOW CALIBRATE
	Serial.println("Keep flat - Calibrating...");
	delay(1000);
	myMPU9250.autoOffsets(); // Calibrate using the settings above
	Serial.println("Calibration Done!");

	// --- PID SETUP ---
	pitchSetpoint = 0; // Target is level
	rollSetpoint = 0;  // Target is level

	pitchPID.SetMode(AUTOMATIC);
	rollPID.SetMode(AUTOMATIC);

	// Limits: Don't let PID take over 100% of the motor power
	pitchPID.SetOutputLimits(-100, 100);
	rollPID.SetOutputLimits(-100, 100);

	// Fast update rate for drone (10ms = 100Hz)
	pitchPID.SetSampleTime(10);
	rollPID.SetSampleTime(10);
}
long lastdebug = 0;
void loop() {
	if (Serial.available()) {
		char cmd = Serial.read();
		if (cmd == 'w') baseThrottle += 10;
		if (cmd == 's') baseThrottle -= 10;
		if (cmd == 'x') baseThrottle = 0;
		baseThrottle = constrain(baseThrottle, 0, 255);  // QX95 safe max
		Serial.print("Throttle: "); Serial.println(baseThrottle);
	}
	// 1. Get Angles
	float pitch = myMPU9250.getPitch();
	float roll = myMPU9250.getRoll();

	// 2. Safety Cutoff (Crash protection)
	// If tilted > 45 degrees, shut down
	if (abs(pitch) > 45 || abs(roll) > 45) {
		writeMotors(0, 0, 0, 0);
		Serial.println("EMERGENCY STOP: TILT > 45");
		return;
	}

	// 3. Update PID
	pitchInput = pitch;
	rollInput = roll;

	pitchPID.Compute();
	rollPID.Compute();



	// FL: Front (Boost) + Left (Drop)
	int pwm_FL = baseThrottle - pitchOutput + rollOutput;

	// FR: Front (Boost) + Right (Boost)
	int pwm_FR = baseThrottle - pitchOutput - rollOutput;

	// BL: Back (Drop) + Left (Drop)
	int pwm_BL = baseThrottle + pitchOutput + rollOutput;

	// BR: Back (Drop) + Right (Boost)
	int pwm_BR = baseThrottle + pitchOutput - rollOutput;

	// 5. Constraints
	pwm_FL = constrain(pwm_FL, 0, 255);
	pwm_FR = constrain(pwm_FR, 0, 255);
	pwm_BL = constrain(pwm_BL, 0, 255);
	pwm_BR = constrain(pwm_BR, 0, 255);

	// 6. Actuate Motors
	// Only spin if we have throttle (safety)
	if (baseThrottle > 10) {
		writeMotors(pwm_FL, pwm_FR, pwm_BL, pwm_BR);
	}
	else {
		writeMotors(0, 0, 0, 0);
	}
	if (millis() - lastdebug > 500) {
		lastdebug = millis();
		// Optional Debugging
		Serial.print("P:"); Serial.print(pitch);
		Serial.print(" R:"); Serial.print(roll);
		Serial.print(" FL:"); Serial.println(pwm_FL);
		Serial.print(" FR:"); Serial.println(pwm_FR);
		Serial.print(" BL:"); Serial.println(pwm_BL);
		Serial.print(" BR:"); Serial.println(pwm_BR);
	}
}