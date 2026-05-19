#ifndef HAND_BRIDGE_H
#define HAND_BRIDGE_H

/**
 * @file hand_bridge.h
 * @brief C bridge header for controlling the HandControl subsystem from C code.
 * @author Leon Angele
 * @date 2026-05-08
 *
 * Exposes a minimal C API to set targets, register executors and bridge
 * UART callbacks into the C++ implementation.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t hand_side_t; /* 0 = Left, 1 = Right */
typedef uint8_t hand_grip_t; /* GripType as uint8_t */

typedef bool (*hand_grip_executor_t)(uint8_t side, uint8_t grip);

/* Executor registration (main registers adapter callback) */
void hand_bridge_set_executor(hand_grip_executor_t cb) ;
/* Initialize bridge (must be called after peripheral init) */
void hand_bridge_init(void) ;

/* Trigger a grip on given side (speed from config) */
bool hand_bridge_set_target_grip(uint8_t side, uint8_t grip);

/* Called from main loop to update both hands (100Hz) */
void hand_bridge_update(void) ;

/* Ping all configured servos and print availability to VCP */
void hand_bridge_ping_all_servos(void);

/* Commander helpers */
bool commander_bridge_feed_byte(uint8_t b) ;
void commander_bridge_process(void) ;

/* HAL UART callback bridges (call from HAL C callbacks) */
void bridge_on_uart_tx(void* huart);
void bridge_on_uart_rx(void* huart);

#ifdef __cplusplus
}
#endif

#endif // HAND_BRIDGE_H
