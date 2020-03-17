
#include<Arduino.h>
#include "ODriveArduino.h"
#include "ODriveErrors.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}
    

void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

/* Work in progress */
void ODriveArduino::SetPosition1(int motor_number, float position, float speed_limit) {
    serial_ << "q " << motor_number  << " " << position << " " << speed_limit << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current) {
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position){
    serial_ << "t " << motor_number << " " << position << "\n";
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

float ODriveArduino::GetVelocity(int motor_number){
	serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODriveArduino::readFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait) {
        do {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

void  ODriveArduino::checkForErrors(int motor_number) {
  serial_ << "r axis"<< motor_number <<".error\n";
  axisError = readInt();
  serial_ << "r axis"<< motor_number <<".motor.error\n";
  motorError = readInt();
  serial_ << "r axis"<< motor_number <<".controller.error\n";
  controllerError = readInt();
  serial_ << "r axis"<< motor_number <<".encoder.error\n";
  encoderError = readInt();
  
   if(axisError != 0 || motorError != 0 || controllerError != 0 || encoderError != 0) {
    if(axisError != 0) {
      for(int i = 0;i < 12;i++) {
        if(axisError & 1<<i) {
          Serial << "Axis"<< motor_number <<" error: " << axisErrors[i] << '\n';
        }
      }
    }
    if(motorError != 0) {
      for(int i = 0;i < 13;i++) {
        if(motorError & 1<<i) {
          Serial << "Motor"<< motor_number <<" error: " << motorErrors[i] << '\n';
        }
      }
    }
    if(controllerError != 0) {
      Serial << "Controller"<< motor_number <<" error: " << controllerErrors[0] << '\n';
    }
    if(encoderError != 0) {
      for(int i = 0;i < 6;i++) {
        if(encoderError & 1<<i) {
          Serial << "Encoder"<< motor_number <<" error: " << encoderErrors[i] << '\n';
        }
      }
    }
  }
}

void  ODriveArduino::resetErrors(int motor_number) {
        delay(10);
        serial_ << "w axis" << motor_number << ".error " << 0 << "\n";
        delay(20);
        serial_ << "w axis" << motor_number <<".motor.error " << 0 << "\n";
        delay(20);
        serial_ << "w axis" << motor_number << ".controller.error " << 0 << "\n";
        delay(20);
        serial_ << "w axis" << motor_number << ".encoder.error " << 0 << "\n";
        delay(20);
        //Serial.println(readString());
}

void ODriveArduino::readConfig(int motor_number){
serial_ << "r axis" << motor_number << ".controller.config.pos_gain\n";
Serial << "Pos gain: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".controller.config.vel_limit\n";
Serial << "Vel limit: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".controller.config.vel_integrator_gain\n";
Serial << "Vel integrator gain: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".motor.config.pole_pairs\n";
Serial << "Pole pairs: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".motor.config.resistance_calib_max_voltage\n";
Serial << "Calib max voltage: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".encoder.config.cpr\n";
Serial << "Encoder cpr: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".motor.config.current_lim\n";
Serial << "Current lim: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".motor.config.calibration_current\n";
Serial << "Calibration current: " << readString() << '\n';

serial_ << "r axis" << motor_number << ".motor.config.pre_calibrated\n";
Serial << "Pre calibrated: " << readString() << '\n';
Serial <<'\n';

}
void ODriveArduino::writeConfig(int motor_number){
serial_ << "w axis" << motor_number << ".controller.config.pos_gain " << 20.0 << "\n";
serial_ << "w axis" << motor_number << ".controller.config.vel_limit " << 50000.0 << "\n";
serial_ << "w axis" << motor_number << ".controller.config.vel_integrator_gain " << 0 << "\n";
serial_ << "w axis" << motor_number << ".motor.config.pole_pairs " << 11 << "\n";
serial_ << "w axis" << motor_number << ".motor.config.resistance_calib_max_voltage " << 4.0 << "\n";
serial_ << "w axis" << motor_number << ".encoder.config.cpr " << 2000 << "\n";
serial_ << "w axis" << motor_number << ".motor.config.current_lim " << 40 << "\n";
serial_ << "w axis" << motor_number << ".motor.config.calibration_current " << 10 << "\n";
}

void ODriveArduino::setPreCalibrated(int motor_number, bool var){
  serial_ << "w axis" << motor_number << ".motor.config.pre_calibrated " << var << "\n";
  delay(50);
}

void  ODriveArduino::saveConfig() {
serial_ << "ss \n";
}

void  ODriveArduino::reboot() {
serial_ << "sr \n";
}


