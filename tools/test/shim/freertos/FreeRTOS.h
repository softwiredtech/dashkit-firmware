#pragma once
// Minimal FreeRTOS shim for host unit tests. The test is single-threaded, so the
// critical-section primitives the DBC value cache uses are no-ops here.

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux)  ((void)(mux))
#define portEXIT_CRITICAL(mux)   ((void)(mux))
