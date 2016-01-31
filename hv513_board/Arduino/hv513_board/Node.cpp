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

  // set shutdown pin LOW to enable the boost converter
  digitalWrite(SHDN_PIN, LOW);

  // set not HI-Z to take chip out of high impedance state 
  digitalWrite(HI_Z_PIN, HIGH);

  // set not blank to LOW (blanked)
  digitalWrite(BL_PIN, LOW);

  // set polarity to normal 
  digitalWrite(POL_PIN, HIGH);

  // ensure SS pins stay high for now
  digitalWrite(HV513_CS_PIN, HIGH);
  digitalWrite(MCP41050_CS_PIN, HIGH);

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

  // set all channels into off state
  digitalWrite(HV513_CS_PIN, 0);
  for (uint16_t i = 0; i < CHANNEL_COUNT / 8; i++) {
    SPI.transfer(state_of_channels_[i]);
  }
  digitalWrite(HV513_CS_PIN, 1);

  Timer1.initialize(50); // initialize timer1, and set a 0.05 ms period

  // attach timer_callback() as a timer overflow interrupt
  Timer1.attachInterrupt(timer_callback);
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
