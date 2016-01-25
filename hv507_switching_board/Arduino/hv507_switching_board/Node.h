#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <CArrayDefs.h>
#include "RPCBuffer.h"  // Define packet sizes
#include "Hv507SwitchingBoard/Properties.h"  // Define package name, URL, etc.
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
#include "hv507_switching_board_config_validate.h"
#include "hv507_switching_board_state_validate.h"
#include "Hv507SwitchingBoard/config_pb.h"
#include <TimerOne.h>


namespace hv507_switching_board {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<hv507_switching_board_Config,
                              config_validate::Validator<Node> > config_t;
typedef nanopb::Message<hv507_switching_board_State,
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
  static const uint16_t CHANNEL_COUNT = 128;

  // pins connected to the HV507
  static const uint8_t DIR_PIN = 10;
  static const uint8_t POL_PIN = 9;
  static const uint8_t BL_PIN = 8;
  static const uint8_t LE_PIN = 4;
  static const uint8_t BOOST_CONVERTER_CS_PIN = 3;

  uint8_t buffer_[BUFFER_SIZE];
  uint8_t state_of_channels_[CHANNEL_COUNT / 8];  // 8 channels per byte

  Node() : BaseNode(), BaseNodeConfig<config_t>(hv507_switching_board_Config_fields),
           BaseNodeState<state_t>(hv507_switching_board_State_fields) {}

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
  uint16_t channel_count() const { return CHANNEL_COUNT; }

  UInt8Array state_of_channels() const {
    return UInt8Array_init(sizeof(state_of_channels_),
                      (uint8_t *)&state_of_channels_[0]);
  }

  bool set_state_of_channels(UInt8Array channel_states) {
    if (channel_states.length == sizeof(state_of_channels_)) {
      digitalWrite(LE_PIN, 0);
      for (uint16_t i = 0; i < channel_states.length; i++) {
        state_of_channels_[i] = channel_states.data[i];
        SPI.transfer(state_of_channels_[i]);
      }
      digitalWrite(LE_PIN, 1);
      return true;
    }
    return false;
  }

  bool set_voltage(uint8_t voltage) {
    digitalWrite(BOOST_CONVERTER_CS_PIN, 0);
    SPI.transfer(voltage);
    digitalWrite(BOOST_CONVERTER_CS_PIN, 1);
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
};

}  // namespace hv507_switching_board


#endif  // #ifndef ___NODE__H___
