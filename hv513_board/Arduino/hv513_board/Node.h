#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <CArrayDefs.h>
#include "RPCBuffer.h"  // Define packet sizes
#include "Hv513Board/Properties.h"  // Define package name, URL, etc.
#include <BaseNodeRpc/BaseNode.h>
#include <BaseNodeRpc/BaseNodeRpc.h>
#include <BaseNodeRpc/BaseNodeEeprom.h>
#include <BaseNodeRpc/BaseNodeI2c.h>
#include <BaseNodeRpc/BaseNodeSpi.h>
#include <BaseNodeRpc/BaseNodeConfig.h>
#include <BaseNodeRpc/BaseNodeState.h>
#include <BaseNodeRpc/BaseNodeSerialHandler.h>
#include <BaseNodeRpc/BaseNodeI2cHandler.h>
#include <BaseNodeRpc/I2cHandler.h>
#include <BaseNodeRpc/SerialHandler.h>
#include <pb_validate.h>
#include <pb_eeprom.h>
#include "hv513_board_config_validate.h"
#include "hv513_board_state_validate.h"
#include "Hv513Board/config_pb.h"
#include "Hv513Board/state_pb.h"
#include <TimerOne.h>


namespace hv513_board {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<hv513_board_Config,
                              config_validate::Validator<Node> > config_t;
typedef nanopb::Message<hv513_board_State,
                        state_validate::Validator<Node> > state_t;

class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeSpi,
  public BaseNodeConfig<config_t>,
  public BaseNodeState<state_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
public:
  typedef PacketParser<FixedPacket> parser_t;

  static void timer_callback();

  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string
  static const uint16_t MAX_NUMBER_OF_CHANNELS = 120;

  // pins connected to the HV513
  static const uint8_t POL_PIN = 5;
  static const uint8_t BL_PIN = 4;
  static const uint8_t HI_Z_PIN = 7;
  static const uint8_t HV513_CS_PIN = 6;

  // pins connected to the boost converter
  static const uint8_t MCP41050_CS_PIN = 3;
  static const uint8_t SHDN_PIN = 2;

  // PCA9505 (gpio) chip/register addresses
  static const uint8_t PCA9505_CONFIG_IO_REGISTER_ = 0x18;
  static const uint8_t PCA9505_OUTPUT_PORT_REGISTER_ = 0x08;

  uint8_t buffer_[BUFFER_SIZE];
  uint8_t state_of_channels_[MAX_NUMBER_OF_CHANNELS / 8];
  uint16_t number_of_channels_;

  Node() : BaseNode(), BaseNodeConfig<config_t>(hv513_board_Config_fields),
           BaseNodeState<state_t>(hv513_board_State_fields) {}

  UInt8Array get_buffer() { return UInt8Array_init(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  void set_i2c_address(uint8_t value);  // Override to validate i2c address

  /****************************************************************************
   * # User-defined methods #
   *
   * Add new methods below.  When Python package is generated using the
   * command, `paver sdist` from the project root directory, the signatures of
   * the methods below will be scanned and code will automatically be generated
   * to support calling the methods from Python over a serial connection.
   *
   * e.g.
   *
   *     bool less_than(float a, float b) { return a < b; }
   *
   * See [`arduino_rpc`][1] and [`base_node_rpc`][2] for more details.
   *
   * [1]: https://github.com/wheeler-microfluidics/arduino_rpc
   * [2]: https://github.com/wheeler-microfluidics/base_node_rpc
   */
  uint16_t number_of_channels() const { return number_of_channels_; }

  UInt8Array state_of_channels() {
    for (uint8_t chip = 0; chip < number_of_channels_ / 40; chip++) {
      for (uint8_t port = 0; port < 5; port++) {
        Wire.beginTransmission((uint8_t)config_._.switching_board_i2c_address + chip);
        Wire.write(PCA9505_OUTPUT_PORT_REGISTER_ + port);
        Wire.endTransmission();
        Wire.requestFrom(config_._.switching_board_i2c_address + chip, 1);
        if (Wire.available()) {
          state_of_channels_[chip*5 + port] = ~Wire.read();
        } else {
          return UInt8Array_init_default();
        }
      }
    }
    return UInt8Array_init(number_of_channels_ / 8,
                      (uint8_t *)&state_of_channels_[0]);
  }

  bool set_state_of_channels(UInt8Array channel_states) {
    if (channel_states.length == number_of_channels_ / 8) {
      for (uint16_t i = 0; i < channel_states.length; i++) {
        state_of_channels_[i] = channel_states.data[i];
      }
      // Each PCA9505 chip has 5 8-bit output registers for a total of 40 outputs
      // per chip. We can have up to 8 of these chips on an I2C bus, which means
      // we can control up to 320 channels.
      //   Each register represent 8 channels (i.e. the first register on the
      // first PCA9505 chip stores the state of channels 0-7, the second register
      // represents channels 8-15, etc.).
      for (uint8_t chip = 0; chip < number_of_channels_ / 40; chip++) {
        for (uint8_t port = 0; port < 5; port++) {
          buffer_[0] = PCA9505_OUTPUT_PORT_REGISTER_ + port;
          buffer_[1] = ~state_of_channels_[chip*5 + port];
          i2c_write(config_._.switching_board_i2c_address + chip,
                    UInt8Array_init(2, (uint8_t *)&buffer_[0]));
          delayMicroseconds(200); // this delay is necessary if we are operating with a 400kbps i2c clock
        }
      }
      return true;
    }
    return false;
  }

  bool on_state_frequency_changed(float frequency) {
    /* This method is triggered whenever a frequency is included in a state
     * update. */
    if ((config_._.min_waveform_frequency <= frequency) &&
                (frequency <= config_._.max_waveform_frequency)) {
      if (frequency == 0) { // DC mode
        digitalWrite(Node::BL_PIN, 1); // set not blanked pin high
        Timer1.stop(); // stop timer
      } else {
        Timer1.setPeriod(500000.0 / frequency); // set timer period in ms
        Timer1.restart();
      }
      return true;
    }
    return false;
  }

  bool on_state_voltage_changed(float voltage) {
    float R2 = 2e6;
    float R1 = 10e3;
    const float POT_MAX = 50e3;
    float value = R2 / ( 2 * voltage / 1.5 - 1 ) - R1;
    if ( value < POT_MAX && value > 0 ) {
      // This method is triggered whenever a voltage is included in a state
      // update.
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      // take the SS pin low to select the chip:
      digitalWrite(MCP41050_CS_PIN, LOW);
      // Send Command to write value and enable the pot
      SPI.transfer(0x1F);
      //  send in the value via SPI:
      SPI.transfer(255 - value / POT_MAX * 255);
      // take the SS pin high to de-select the chip:
      digitalWrite(MCP41050_CS_PIN, HIGH);
      SPI.endTransaction();

      // for some reason, the HV513 seems to lose its state (i.e., all outputs
      // are set to zero) when we change the voltage, so whenever the voltage
      // changes, reapply the state
      digitalWrite(HV513_CS_PIN, 0);
      SPI.transfer((uint8_t)state_._.output_enabled * 0xFF);
      digitalWrite(HV513_CS_PIN, 1);
      return true;
    }
    return false;
  }

  bool on_state_output_enabled_changed(bool value) {
    digitalWrite(SHDN_PIN, !value);
    digitalWrite(HV513_CS_PIN, 0);
    SPI.transfer((uint8_t)value * 0xFF);
    digitalWrite(HV513_CS_PIN, 1);
    return true;
  }

  void _initialize_switching_boards();
};

}  // namespace hv513_board


#endif  // #ifndef ___NODE__H___
