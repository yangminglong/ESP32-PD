#pragma once


#define PD_REQUEST_REFRESH_MS 2000

void pd_mode(pd_mode_t mode);
void pd_init();
void pd_state_reset();

void pd_request_timer();
void pd_refresh_request(bool immediate);
void pd_request(uint8_t object, uint32_t current_ma, bool immediate);
void pd_request_pps(uint8_t object, uint32_t voltage_mv, uint32_t current_ma, bool immediate);
