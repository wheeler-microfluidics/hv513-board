#include <SPI.h>
#include "Node.h"

namespace hv507_switching_board {

void Node::begin() {
  config_.set_buffer(get_buffer());
  config_.validator_.set_node(*this);
  config_.reset();
  config_.load();
  state_.set_buffer(get_buffer());
  state_.validator_.set_node(*this);
  // Start Serial after loading config to set baud rate.
#if !defined(DISABLE_SERIAL)
  Serial.begin(config_._.baud_rate);
#endif  // #ifndef DISABLE_SERIAL
  // Set i2c clock-rate to 400kHz.
  TWBR = 12;

  pinMode(DIR_PIN, OUTPUT);
  pinMode(POL_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(LE_PIN, OUTPUT);
  pinMode(BOOST_CONVERTER_CS_PIN, OUTPUT);

  // set DIOA as data in
  digitalWrite(DIR_PIN, LOW);

  // set not blank to LOW (blanked)
  digitalWrite(BL_PIN, LOW);

  // set polarity to ?
  digitalWrite(POL_PIN, HIGH);

  digitalWrite(LE_PIN, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin();

  // set all channels into off state
  digitalWrite(LE_PIN, 0);
  for (uint16_t i = 0; i < CHANNEL_COUNT / 8; i++) {
    SPI.transfer(state_of_channels_[i]);
  }
  digitalWrite(LE_PIN, 1);

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

}  // namespace hv507_switching_board
