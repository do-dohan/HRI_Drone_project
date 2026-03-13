std_msgs/Header header

bool engaged
uint8 state
uint8 phase

uint32 reason_bits
uint8 reason_primary
uint32 hard_fail_mask
uint32 soft_mask

bool button_on_off
bool button_valid
bool wearable_fresh
bool px4_safe

bool soft_valid

float32 s_gate
float32 s_internal
float32 s_sig
float32 s_phase

float32 s_fresh
float32 s_button
float32 s_px4
float32 s_packet
float32 s_motion
float32 s_acc
float32 s_mag
float32 s_clip
float32 s_entry

float32 wearable_last_rx_dt
float32 drop_ratio
float32 jitter_metric
float32 hardgate_persist_time
float32 entry_progress