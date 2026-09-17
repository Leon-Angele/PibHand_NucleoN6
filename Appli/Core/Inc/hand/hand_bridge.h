#ifndef HAND_BRIDGE_H
#define HAND_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hand_bridge_init(void);
bool hand_bridge_set_target_grip(uint8_t grip);
void hand_bridge_update(void);
void hand_bridge_service(void);
void hand_bridge_ping_all_servos(void);

bool commander_bridge_feed_byte(uint8_t b);
void commander_bridge_process(void);

void bridge_on_uart_tx(void* huart);
void bridge_on_uart_rx(void* huart);
void bridge_on_uart_error(void* huart);

#ifdef __cplusplus
}
#endif

#endif // HAND_BRIDGE_H
