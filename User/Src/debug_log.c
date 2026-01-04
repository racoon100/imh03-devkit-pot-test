#include <string.h>
#include "debug_log.h"

#define NUM_lOG_ENTRIES    64

static uint32_t log_write_id = 0;
static debug_log_entry_t debug_log_entries[NUM_lOG_ENTRIES];

void debug_test(void) {
    return;
}

void debug_log_update(debug_log_entry_t debug_log_entry) {

    memcpy(&debug_log_entries[log_write_id], &debug_log_entry, sizeof(debug_log_entry_t));

    log_write_id++;
    if (log_write_id >= NUM_lOG_ENTRIES) {
        log_write_id = 0;
    }
    return;
}
