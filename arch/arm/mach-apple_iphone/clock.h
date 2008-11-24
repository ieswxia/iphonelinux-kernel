extern int iphone_power_ctrl(u32 device, int on_off);
extern void iphone_clock_gate_switch(u32 gate, int on_off);
extern u64 iphone_microtime(void);
extern int iphone_has_elapsed(u64 startTime, u64 elapsedTime);
