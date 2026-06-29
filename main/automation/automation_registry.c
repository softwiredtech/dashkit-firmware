// THE central registry: the one place edited to add or remove an automation.
// Each automation lives in its own .c file and defines a single automation_t;
// list it here (and in main/CMakeLists.txt SRCS) to activate it.

#include "automation.h"

extern automation_t wiper_off_automation;
extern automation_t multi_finger_automation;
extern automation_t battery_preheat_automation;

automation_t *const g_automations[] = {
    &wiper_off_automation,
    &multi_finger_automation,
    &battery_preheat_automation,
};

const int g_automation_count =
    (int)(sizeof(g_automations) / sizeof(g_automations[0]));
