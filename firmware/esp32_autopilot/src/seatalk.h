#ifndef SEATALK_H
#define SEATALK_H

#include "app_state.h"

// Initialize Seatalk/p70 integration (call after n2k_init)
void seatalk_init();

// Process an incoming N2K message for Seatalk PGNs.
// Returns true if the message was handled (caller can skip further parsing).
bool seatalk_handle_msg(const unsigned char* data, int data_len, unsigned long pgn,
                        unsigned char source, AppState& state);

// Send Seatalk status PGNs to the bus.
// Internally manages per-PGN timing.
void seatalk_send(const AppState& state);

// Apply pending p70 commands to state (call from main loop alongside web commands)
void seatalk_apply_commands(AppState& state);

// Returns true if a p70 keypad heartbeat has been received recently
bool seatalk_p70_present();

// Returns millis() of last p70 heartbeat
uint32_t seatalk_p70_last_heartbeat();

#endif // SEATALK_H
