#include <stdint.h>

typedef struct {
    uint32_t ts;
    int16_t raw_speed;
    int16_t filtered_speed;
    int16_t var1;
    int16_t var2;
} debug_log_entry_t;

void debug_test(void);
