#pragma once
// Minimal semaphore shim for host unit tests: the tests are single-threaded, so a mutex is a token.
typedef int *SemaphoreHandle_t;
#define portMAX_DELAY 0xFFFFFFFFu
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { static int token; return &token; }
static inline int xSemaphoreTake(SemaphoreHandle_t sem, unsigned ticks) { (void)sem; (void)ticks; return 1; }
static inline int xSemaphoreGive(SemaphoreHandle_t sem) { (void)sem; return 1; }
