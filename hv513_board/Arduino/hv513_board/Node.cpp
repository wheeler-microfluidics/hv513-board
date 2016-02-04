#include <SPI.h>
#include "Node.h"

namespace hv513_board {

void Node::begin() {
  pinMode(POL_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(HI_Z_PIN, OUTPUT);
  pinMode(HV513_CS_PIN, OUTPUT);
  pinMode(MCP41050_CS_PIN, OUTPUT);
  pinMode(SHDN_PIN, OUTPUT);

  // ensure SS pins stay high for now
  digitalWrite(HV513_CS_PIN, HIGH);
  digitalWrite(MCP41050_CS_PIN, HIGH);

  // set not HI-Z to take chip out of high impedance state
  digitalWrite(HI_Z_PIN, HIGH);

  // set not blank to LOW (blanked)
  digitalWrite(BL_PIN, LOW);

  // set polarity to normal 
  digitalWrite(POL_PIN, HIGH);

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin();

  config_.set_buffer(get_buffer());
  config_.validator_.set_node(*this);
  config_.reset();
  config_.load();
  state_.set_buffer(get_buffer());
  state_.validator_.set_node(*this);
  state_.reset();
  // Mark voltage, frequency state for validation.
  state_._.has_voltage = true;
  state_._.has_frequency = true;
  state_._.has_output_enabled = true;
  // Validate state to trigger on-changed handling for state fields that are
  // set (which initializes the state to the default values supplied in the
  // state protocol buffer definition).
  state_.validate();
  // Start Serial after loading config to set baud rate.
#if !defined(DISABLE_SERIAL)
  Serial.begin(115200);
#endif  // #ifndef DISABLE_SERIAL
  // Set i2c clock-rate to 400kHz.
  TWBR = 12;

  Timer1.initialize(50); // initialize timer1, and set a 0.05 ms period

  // attach timer_callback() as a timer overflow interrupt
  Timer1.attachInterrupt(timer_callback);

  _initialize_switching_boards();
}

void Node::_initialize_switching_boards() {
  // Check how many switching boards are connected.  Each additional board's
  // address must equal the previous boards address +1 to be valid.
  number_of_channels_ = 0;

  uint8_t data[2];
  for (uint8_t chip = 0; chip < 8; chip++) {
    // set IO ports as inputs
    data[0] = PCA9505_CONFIG_IO_REGISTER_;
    data[1] = 0xFF;
    i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
              UInt8Array_init(2, (uint8_t *)&data[0]));

    // read back the register value
    // if it matches what we previously set, this might be a PCA9505 chip
    if (i2c_read((uint8_t)config_._.switching_board_i2c_address + chip, 1).data[0] == 0xFF) {
      // try setting all ports in output mode and initialize to ground
      uint8_t port=0;
      for (; port<5; port++) {
        data[0] = PCA9505_CONFIG_IO_REGISTER_ + port;
        data[1] = 0x00;
        i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
                  UInt8Array_init(2, (uint8_t *)&data[0]));
        data[0] = i2c_read((uint8_t)config_._.switching_board_i2c_address + chip, 1).data[0];

        // check that we successfully set the IO config register to 0x00
        if (i2c_read((uint8_t)config_._.switching_board_i2c_address + chip, 1).data[0] != 0x00) {
          return;
        }
        data[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
        data[1] = 0xFF;
        i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
                  UInt8Array_init(2, (uint8_t *)&data[0]));
      }

      // if port=5, it means that we successfully initialized all IO config
      // registers to 0x00, and this is probably a PCA9505 chip
      if (port==5) {
        if (number_of_channels_ == 40 * chip) {
          number_of_channels_ = 40 * (chip + 1);
        }
      }
    }
  }
}

void Node::timer_callback() {
  uint8_t state = digitalRead(Node::BL_PIN);
  digitalWrite(Node::BL_PIN, !state);
}

void Node::set_i2c_address(uint8_t value) {
  // Validator expects `uint32_t` by reference.
  uint32_t address = value;
  /* Validate address and update the active `Wire` configuration if the
    * address is valid. */
  config_.validator_.i2c_address_(address, config_._.i2c_address);
  config_._.i2c_address = address;
}

}  // namespace hv513_board
