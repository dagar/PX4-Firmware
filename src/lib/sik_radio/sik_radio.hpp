
#include <cstdint>


/**
 * Check the configuration of a connected radio
 *
 * This convenience function allows to re-configure a connected
 * radio without removing it from the main system harness.
 */
void check_radio_config(int uart_fd, int32_t radio_id);
