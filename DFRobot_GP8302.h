#define GP8302_DEF_I2C_ADDR                 0x58  ///< The default I2C address of the I2C current DAC module
#define GP8302_CONFIG_CURRENT_REG           0x02  ///< Configure current sensor register
#define GP8302_CURRENT_RESOLUTION           4095 ///< Current resolution: 12 bits, 0x0FFF
#define GP8302_MAX_CURRENT                  20    ///< Maximum switching current: 25mA
#define GP8302_STORE_TIMING_HEAD            0x02  ///< Store function timing start head
#define GP8302_STORE_TIMING_ADDR            0x10  ///< The first address for entering store timing
#define GP8302_STORE_TIMING_CMD1            0x03  ///< The command 1 to enter store timing
#define GP8302_STORE_TIMING_CMD2            0x00  ///< The command 2 to enter store timing
#define GP8302_STORE_TIMING_DELAY           10    ///< Store procedure interval delay time: 10ms, more than 7ms

#include <Arduino.h>
#include <Wire.h>

class DFRobot_GP8302 {
public:
    DFRobot_GP8302(uint8_t twiaddr = GP8302_DEF_I2C_ADDR);
    bool begin();
    void calibration4_20mA(uint16_t dac_4, uint16_t dac_20);
    float output_mA(uint16_t dac);
    uint16_t output(float current_mA);
    void store();

private:
    uint8_t _twiaddr; // The I2C address
    bool _calibration; // Calibration flag
    uint16_t _dac_4; // DAC value at 4mA
    uint16_t _dac_20; // DAC value at 20mA
    uint16_t _digital; // Last DAC value set
};

DFRobot_GP8302::DFRobot_GP8302(uint8_t twiaddr) : _twiaddr(twiaddr), _calibration(false), _dac_4(0), _dac_20(0), _digital(0) {}

bool DFRobot_GP8302::begin() {
    Wire.begin(); // Initialize I2C bus
    Wire.beginTransmission(_twiaddr);
    if (Wire.endTransmission() == 0) {
        return true; // Success, device found
    } else {
        return false; // Device not found
    }
}

void DFRobot_GP8302::calibration4_20mA(uint16_t dac_4, uint16_t dac_20) {
    if ((dac_4 >= dac_20) || (dac_20 > GP8302_CURRENT_RESOLUTION)) return;
    _dac_4 = dac_4;
    _dac_20 = dac_20;
    _calibration = true;
}

float DFRobot_GP8302::output_mA(uint16_t dac) {
    if (dac > GP8302_CURRENT_RESOLUTION) return -1; // Invalid input
    _digital = dac & GP8302_CURRENT_RESOLUTION;

    Wire.beginTransmission(_twiaddr);
    Wire.write(GP8302_CONFIG_CURRENT_REG);
    Wire.write((_digital << 4) & 0xF0); // LSB second
    Wire.write((_digital >> 4) & 0xFF); // MSB first
    Wire.endTransmission();

    return float((_digital / float(GP8302_CURRENT_RESOLUTION)) * GP8302_MAX_CURRENT);
}

uint16_t DFRobot_GP8302::output(float current_mA) {
    if (current_mA < 0) current_mA = 0;
    if (current_mA > GP8302_MAX_CURRENT) current_mA = GP8302_MAX_CURRENT;

    if (_calibration && (current_mA >= 4) && (current_mA <= 20)) {
        _digital = _dac_4 + static_cast<uint16_t>((current_mA - 4) * (_dac_20 - _dac_4) / (20 - 4));
        // Rounding adjustment
        if (((_dac_4 + (current_mA - 4) * (_dac_20 - _dac_4) / (20.0 - 4)) - _digital) * 10 >= 5) _digital += 1;
    } else {
        _digital = static_cast<uint16_t>((current_mA * GP8302_CURRENT_RESOLUTION) / GP8302_MAX_CURRENT);
        // Rounding adjustment
        if ((((current_mA * GP8302_CURRENT_RESOLUTION) / (GP8302_MAX_CURRENT * 1.0)) - _digital) * 10 >= 5) _digital += 1;
    }

    // Send the calculated DAC value via I2C
    return output_mA(_digital);
}

void DFRobot_GP8302::store() {
    // Step 1: Enter Store Timing Mode
    Wire.beginTransmission(_twiaddr);
    Wire.write(GP8302_STORE_TIMING_HEAD); // Command to prepare for store operation
    Wire.endTransmission();

    // Short delay to ensure the device is ready for the next command
    delay(GP8302_STORE_TIMING_DELAY);

    // Step 2: Send Store Command
    Wire.beginTransmission(_twiaddr);
    Wire.write(GP8302_STORE_TIMING_ADDR); // Address indicating start of store operation
    Wire.write(GP8302_STORE_TIMING_CMD1); // Command to initiate store operation
    Wire.endTransmission();

    // Short delay between commands as per device requirements
    delay(GP8302_STORE_TIMING_DELAY);

    // Step 3: Send GP8302_STORE_TIMING_CMD2 repeatedly as required by the device
    Wire.beginTransmission(_twiaddr);
    for (int i = 0; i < 8; ++i) {
        Wire.write(GP8302_STORE_TIMING_CMD2); // Repeated command to confirm/finalize store operation
    }
    Wire.endTransmission();

    // Final delay to ensure completion of the store operation
    delay(GP8302_STORE_TIMING_DELAY);
}
