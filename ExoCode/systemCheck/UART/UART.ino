#include "src\Utilities.h"
#include "src\UARTHandler.h"
#include "src\UART_msg_t.h"

namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  
}

void loop() {
  static uint8_t first_run{1};
  if (first_run) {
    
  }
  
  static ExoData exo_data(config_info::config_to_send);

  /* How to check for, receive, and handle a uart command */
  UARTHandler* inst = UARTHandler::get_instance();
  UART_msg_t msg = inst->poll();
  if (msg.command) {
    uart_command_utils::handle_msg(inst, msg, exo_data);
  }

  /* How to send a uart command (Using inst from above)*/
  inst
}
