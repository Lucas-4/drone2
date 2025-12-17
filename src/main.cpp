
// #include <Wire.h>
// #include <Arduino.h>
// #include <MPU9250_WE.h>
// #include <WiFi.h>
// #include <WiFiUdp.h>

// // ================= WIFI SETTINGS =================
// const char *ssid = "Drone_ESP32";
// const char *password = "12345678";

// WiFiUDP udp;
// const int udpPort = 4210;
// char packetBuffer[255];
// unsigned long lastPacketTime = 0;

// // ================= MOTOR PINS (YOUR CONFIG) =================
// const int FR_PIN = 32; const int FR_CH = 1;
// const int FL_PIN = 13; const int FL_CH = 2;
// const int BR_PIN = 25; const int BR_CH = 3;
// const int BL_PIN = 18; const int BL_CH = 4;

// // ================= PWM SETTINGS =================
// const int PWM_FREQ = 20000;
// const int PWM_RES = 8;

// // ================= TUNING (P-CONTROLLER) =================
// double Kp = 1.7;

// // ================= FILTER VARIABLES (THE FIX) =================
// // 0.98 means 98% Gyro (Smooth) + 2% Accel (Correction)
// float alpha = 0.98;

// float pitch = 0.0;
// float roll = 0.0;
// unsigned long prevTime = 0;
// xyzFloat gyroOffsets = { 0, 0, 0 }; // Stores calibration data

// // ================= FLIGHT VARIABLES =================
// int baseThrottle = 0;
// float targetPitch = 0;
// float targetRoll = 0;

// float pitchOutput = 0;
// float rollOutput = 0;

// MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

// // ================= HELPER FUNCTIONS =================

// void writeMotors(int fl, int fr, int bl, int br) {
// 	ledcWrite(FL_CH, constrain(fl, 0, 255));
// 	ledcWrite(FR_CH, constrain(fr, 0, 255));
// 	ledcWrite(BL_CH, constrain(bl, 0, 255));
// 	ledcWrite(BR_CH, constrain(br, 0, 255));
// }

// void setup() {
// 	Serial.begin(115200);
// 	Wire.begin();

// 	// 1. Motor Setup
// 	ledcSetup(FR_CH, PWM_FREQ, PWM_RES); ledcAttachPin(FR_PIN, FR_CH);
// 	ledcSetup(FL_CH, PWM_FREQ, PWM_RES); ledcAttachPin(FL_PIN, FL_CH);
// 	ledcSetup(BR_CH, PWM_FREQ, PWM_RES); ledcAttachPin(BR_PIN, BR_CH);
// 	ledcSetup(BL_CH, PWM_FREQ, PWM_RES); ledcAttachPin(BL_PIN, BL_CH);
// 	writeMotors(0, 0, 0, 0);

// 	// 2. WiFi Setup
// 	Serial.println("Creating WiFi Network...");
// 	WiFi.softAP(ssid, password);
// 	udp.begin(udpPort);
// 	Serial.println("UDP Listener Started...");

// 	// 3. MPU Setup
// 	if (!myMPU9250.init()) {
// 		Serial.println("❌ MPU Error!");
// 	}
// 	else {
// 		Serial.println("✅ MPU Connected.");
// 	}

// 	// --- FILTER SETUP (CRITICAL FOR STABILITY) ---
// 	// Enable Digital Low Pass Filter to remove motor vibration noise
// 	myMPU9250.enableAccDLPF(true);
// 	myMPU9250.setAccDLPF(MPU9250_DLPF_6); // Setting 6 = ~5Hz bandwidth (Strong smoothing)
// 	myMPU9250.enableGyrDLPF();
// 	myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

// 	myMPU9250.setSampleRateDivider(5);
// 	myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
// 	myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);

// 	// 4. GYRO CALIBRATION
// 	Serial.println("--------------------------------");
// 	Serial.println("CALIBRATING GYRO... KEEP STILL!");
// 	delay(1000);
// 	xyzFloat gSum = { 0,0,0 };
// 	for (int i = 0; i < 500; i++) {
// 		xyzFloat g = myMPU9250.getGyrValues();
// 		gSum.x += g.x;
// 		gSum.y += g.y;
// 		gSum.z += g.z;
// 		delay(2);
// 	}
// 	gyroOffsets.x = gSum.x / 500.0;
// 	gyroOffsets.y = gSum.y / 500.0;
// 	gyroOffsets.z = gSum.z / 500.0;

// 	prevTime = micros(); // Start timer
// 	Serial.println("READY TO FLY.");
// 	Serial.println("--------------------------------");
// }

// void loop() {
// 	// ================= 1. RECEIVE FLUTTER COMMANDS =================
// 	int packetSize = udp.parsePacket();
// 	if (packetSize) {
// 		int len = udp.read(packetBuffer, 255);
// 		if (len > 0) {
// 			packetBuffer[len] = 0;
// 			char *ptr = strtok(packetBuffer, ",");
// 			if (ptr) {
// 				// Throttle
// 				int rawThrottle = atoi(ptr);
// 				baseThrottle = map(rawThrottle, 0, 1023, 0, 255);

// 				// Pitch (Map -100..100 -> -30..30 deg)
// 				ptr = strtok(NULL, ",");
// 				int rawPitch = atoi(ptr);
// 				targetPitch = map(rawPitch, -100, 100, 30, -30);

// 				// Roll (Map -100..100 -> -30..30 deg)
// 				ptr = strtok(NULL, ",");
// 				int rawRoll = atoi(ptr);
// 				targetRoll = map(rawRoll, -100, 100, 30, -30);

// 				lastPacketTime = millis();
// 			}
// 		}
// 	}

// 	// ================= 2. SAFETY FAILSAFE =================
// 	if (millis() - lastPacketTime > 1000) {
// 		baseThrottle = 0;
// 		targetPitch = 0;
// 		targetRoll = 0;
// 	}

// 	// ================= 3. COMPLEMENTARY FILTER (NO MORE JITTER) =================
// 	// Calculate how much time passed since last loop (in seconds)
// 	unsigned long currTime = micros();
// 	float dt = (currTime - prevTime) / 1000000.0;
// 	prevTime = currTime;

// 	// A. Get Raw Data
// 	xyzFloat g = myMPU9250.getGyrValues();
// 	xyzFloat a = myMPU9250.getGValues();

// 	// Subtract Calibration Offsets
// 	float gyrX = g.x - gyroOffsets.x;
// 	float gyrY = g.y - gyroOffsets.y;

// 	// B. Calculate Accelerometer Angles (Trigonometry)
// 	// These are noisy but don't drift
// 	float accPitch = atan2(a.x, a.z) * 57.296;
// 	float accRoll = atan2(-a.y, a.z) * 57.296;

// 	// C. Combine!
// 	// Angle = 0.98 * (OldAngle + GyroChange)  +  0.02 * (AccelAngle)
// 	pitch = alpha * (pitch + gyrY * dt) + (1.0 - alpha) * accPitch;
// 	roll = alpha * (roll + gyrX * dt) + (1.0 - alpha) * accRoll;

// 	// ================= 4. CRASH PROTECTION =================
// 	if (abs(pitch) > 60 || abs(roll) > 60) {
// 		writeMotors(0, 0, 0, 0);
// 		baseThrottle = 0;
// 		return;
// 	}

// 	// ================= 5. P-CONTROLLER CALCULATION =================
// 	// Compare Filtered Angle to Target Angle
// 	pitchOutput = (pitch - targetPitch) * Kp;
// 	rollOutput = (roll - targetRoll) * Kp;

// 	// ================= 6. MOTOR MIXING =================
// 	// FL: Pitch(+), Roll(-)
// 	int pwm_FL = baseThrottle - pitchOutput + rollOutput;
// 	// FR: Pitch(+), Roll(+)
// 	int pwm_FR = baseThrottle - pitchOutput - rollOutput;
// 	// BL: Pitch(-), Roll(-)
// 	int pwm_BL = baseThrottle + pitchOutput + rollOutput;
// 	// BR: Pitch(-), Roll(+)
// 	int pwm_BR = baseThrottle + pitchOutput - rollOutput;

// 	// Constrain
// 	pwm_FL = constrain(pwm_FL, 0, 255);
// 	pwm_FR = constrain(pwm_FR, 0, 255);
// 	pwm_BL = constrain(pwm_BL, 0, 255);
// 	pwm_BR = constrain(pwm_BR, 0, 255);

// 	// ================= 7. ACTUATE =================
// 	if (baseThrottle > 15) {
// 		writeMotors(pwm_FL, pwm_FR, pwm_BL, pwm_BR);
// 	}
// 	else {
// 		writeMotors(0, 0, 0, 0);
// 	}

// 	// ================= 8. DEBUG =================
// 	static long lastPrint = 0;
// 	if (millis() - lastPrint > 100) {
// 		lastPrint = millis();
// 		// Print Filtered Angles to Serial Plotter to verify smoothness
// 		Serial.print("Pitch:"); Serial.print(pitch);
// 		Serial.print(" Roll:"); Serial.print(roll);
// 		Serial.print(" | FL:"); Serial.print(pwm_FL);
// 		Serial.print(" FR:"); Serial.print(pwm_FR);
// 		Serial.print(" BL:"); Serial.print(pwm_BL);
// 		Serial.print(" BR:"); Serial.println(pwm_BR);
// 	}
// }
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ================= WIFI SETTINGS =================
const char *ssid = "Drone_ESP32";
const char *password = "12345678";
WiFiUDP udp;
const int udpPort = 4210;
char packetBuffer[255];
unsigned long lastPacketTime = 0;

// ================= MOTOR PINS (MATCHING YOUR PCB) =================
const int FR_PIN = 32; const int FR_CH = 1;
const int FL_PIN = 13; const int FL_CH = 2;
const int BR_PIN = 25; const int BR_CH = 3;
const int BL_PIN = 18; const int BL_CH = 4;

// ================= PWM SETTINGS =================
const int PWM_FREQ = 20000; // High freq for brushed motors
const int PWM_RES = 8;      // 0-255 resolution

// ================= PID VARIABLES (From the code you found) =================
float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007; float DAnglePitch = DAngleRoll;

float PRateRoll = 0.625; float IRateRoll = 2.1; float DRateRoll = 0.0088;
float PRatePitch = PRateRoll; float IRatePitch = IRateRoll; float DRatePitch = DRateRoll;
float PRateYaw = 4; float IRateYaw = 3; float DRateYaw = 0;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

float DesiredAngleRoll, DesiredAnglePitch, DesiredRateYaw;
float ErrorAngleRoll, ErrorAnglePitch, PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float InputRoll, InputPitch, InputYaw, InputThrottle;
uint32_t LoopTimer;
float t = 0.004; // 250Hz Loop

// ================= HELPER FUNCTIONS =================

void writeMotors(int fl, int fr, int bl, int br) {
	ledcWrite(FL_CH, constrain(fl, 0, 255));
	ledcWrite(FR_CH, constrain(fr, 0, 255));
	ledcWrite(BL_CH, constrain(bl, 0, 255));
	ledcWrite(BR_CH, constrain(br, 0, 255));
}

void gyro_signals(void) {
	Wire.beginTransmission(0x68);
	Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
	Wire.beginTransmission(0x68);
	Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
	Wire.beginTransmission(0x68);
	Wire.write(0x3B); Wire.endTransmission();
	Wire.requestFrom(0x68, 6);
	int16_t AccXLSB = Wire.read() << 8 | Wire.read();
	int16_t AccYLSB = Wire.read() << 8 | Wire.read();
	int16_t AccZLSB = Wire.read() << 8 | Wire.read();
	Wire.beginTransmission(0x68);
	Wire.write(0x1B); Wire.write(0x8); Wire.endTransmission();
	Wire.beginTransmission(0x68);
	Wire.write(0x43); Wire.endTransmission();
	Wire.requestFrom(0x68, 6);
	int16_t GyroX = Wire.read() << 8 | Wire.read();
	int16_t GyroY = Wire.read() << 8 | Wire.read();
	int16_t GyroZ = Wire.read() << 8 | Wire.read();

	RateRoll = (float)GyroX / 65.5;
	RatePitch = (float)GyroY / 65.5;
	RateYaw = (float)GyroZ / 65.5;
	AccX = (float)AccXLSB / 4096;
	AccY = (float)AccYLSB / 4096;
	AccZ = (float)AccZLSB / 4096;

	AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
	AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}

void setup() {
	Serial.begin(115200);
	Wire.setClock(400000);
	Wire.begin();
	delay(250);

	// Wake up MPU
	Wire.beginTransmission(0x68);
	Wire.write(0x6B); Wire.write(0x00);
	Wire.endTransmission();

	// Motor Setup
	ledcSetup(FR_CH, PWM_FREQ, PWM_RES); ledcAttachPin(FR_PIN, FR_CH);
	ledcSetup(FL_CH, PWM_FREQ, PWM_RES); ledcAttachPin(FL_PIN, FL_CH);
	ledcSetup(BR_CH, PWM_FREQ, PWM_RES); ledcAttachPin(BR_PIN, BR_CH);
	ledcSetup(BL_CH, PWM_FREQ, PWM_RES); ledcAttachPin(BL_PIN, BL_CH);
	writeMotors(0, 0, 0, 0);

	// WiFi Setup
	WiFi.softAP(ssid, password);
	udp.begin(udpPort);

	// Calibration (Keep drone still on start!)
	Serial.println("Calibrating...");
	for (int i = 0; i < 2000; i++) {
		gyro_signals();
		RateCalibrationRoll += RateRoll;
		RateCalibrationPitch += RatePitch;
		RateCalibrationYaw += RateYaw;
		delay(1);
	}
	RateCalibrationRoll /= 2000;
	RateCalibrationPitch /= 2000;
	RateCalibrationYaw /= 2000;

	LoopTimer = micros();
}

void loop() {
	// 1. READ GYRO
	gyro_signals();
	RateRoll -= RateCalibrationRoll;
	RatePitch -= RateCalibrationPitch;
	RateYaw -= RateCalibrationYaw;

	// Complementary Filter
	complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
	complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;

	// 2. READ WIFI (FLUTTER)
	int packetSize = udp.parsePacket();
	if (packetSize) {
		int len = udp.read(packetBuffer, 255);
		if (len > 0) {
			packetBuffer[len] = 0;
			char *ptr = strtok(packetBuffer, ",");
			if (ptr) {
				int rawThrottle = atoi(ptr);
				// Map throttle to 0-255 range
				InputThrottle = map(rawThrottle, 0, 1023, 0, 255);

				ptr = strtok(NULL, ",");
				int rawPitch = atoi(ptr);
				// Map -100..100 input to -30..30 degrees angle
				DesiredAnglePitch = map(rawPitch, -100, 100, -30, 30);

				ptr = strtok(NULL, ",");
				int rawRoll = atoi(ptr);
				DesiredAngleRoll = map(rawRoll, -100, 100, -30, 30);

				lastPacketTime = millis();
			}
		}
	}

	// Safety: If no WiFi signal for 1 sec, kill motors
	if (millis() - lastPacketTime > 1000) {
		InputThrottle = 0;
		DesiredAnglePitch = 0;
		DesiredAngleRoll = 0;
	}

	// ================= PID CALCULATIONS =================

	// --- ROLL PID ---
	ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
	float PtermRoll = PAngleRoll * ErrorAngleRoll;
	float ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
	ItermRoll = constrain(ItermRoll, -100, 100);
	float DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
	float PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
	float DesiredRateRoll = PIDOutputRoll;
	PrevErrorAngleRoll = ErrorAngleRoll;
	PrevItermAngleRoll = ItermRoll;

	ErrorRateRoll = DesiredRateRoll - RateRoll;
	PtermRoll = PRateRoll * ErrorRateRoll;
	ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
	ItermRoll = constrain(ItermRoll, -100, 100);
	DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
	InputRoll = PtermRoll + ItermRoll + DtermRoll;
	PrevErrorRateRoll = ErrorRateRoll;
	PrevItermRateRoll = ItermRoll;

	// --- PITCH PID ---
	ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
	float PtermPitch = PAnglePitch * ErrorAnglePitch;
	float ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
	ItermPitch = constrain(ItermPitch, -100, 100);
	float DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
	float PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
	float DesiredRatePitch = PIDOutputPitch;
	PrevErrorAnglePitch = ErrorAnglePitch;
	PrevItermAnglePitch = ItermPitch;

	ErrorRatePitch = DesiredRatePitch - RatePitch;
	PtermPitch = PRatePitch * ErrorRatePitch;
	ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
	ItermPitch = constrain(ItermPitch, -100, 100);
	DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
	InputPitch = PtermPitch + ItermPitch + DtermPitch;
	PrevErrorRatePitch = ErrorRatePitch;
	PrevItermRatePitch = ItermPitch;

	// ================= MOTOR MIXING =================
	// Only mix if throttle is active
	if (InputThrottle > 20) {
		int m1 = InputThrottle - InputRoll - InputPitch; // FR
		int m2 = InputThrottle + InputRoll - InputPitch; // FL
		int m3 = InputThrottle + InputRoll + InputPitch; // BL
		int m4 = InputThrottle - InputRoll + InputPitch; // BR

		writeMotors(m2, m1, m3, m4);
	}
	else {
		writeMotors(0, 0, 0, 0);
		// Reset I-Terms to prevent "windup" on ground
		PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
		PrevItermRateRoll = 0; PrevItermRatePitch = 0;
	}

	// ================= LOOP TIMING (250Hz) =================
	while (micros() - LoopTimer < (t * 1000000));
	LoopTimer = micros();
}