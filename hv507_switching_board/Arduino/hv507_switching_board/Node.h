#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <CArrayDefs.h>
#include "RPCBuffer.h"  // Define packet sizes
#include "Hv507SwitchingBoard/Properties.h"  // Define package name, URL, etc.
#include <BaseNodeRpc/BaseNodeRpc.h>
#include <BaseNodeRpc/BaseNodeEeprom.h>
#include <BaseNodeRpc/BaseNodeI2c.h>
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
  public BaseNodeConfig<config_t>,
  public BaseNodeState<state_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
public:
  typedef PacketParser<FixedPacket> parser_t;

  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string
  static const uint16_t CHANNEL_COUNT = 64;

  uint8_t buffer_[BUFFER_SIZE];

  Node() : BaseNode(), BaseNodeConfig<config_t>(hv507_switching_board_Config_fields),
           BaseNodeState<state_t>(hv507_switching_board_State_fields) {}

  UInt8Array get_buffer() { return UInt8Array(sizeof(buffer_), buffer_); }
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
};

}  // namespace hv507_switching_board


#endif  // #ifndef ___NODE__H___
