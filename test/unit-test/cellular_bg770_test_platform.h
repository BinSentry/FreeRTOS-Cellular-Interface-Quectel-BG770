/**
 * @file cellular_bg770_test_platform.h
 * @brief Declarations the BG770 module needs in the unit test build that the FreeRTOS-Cellular-Interface test
 * platform header doesn't provide: the FreeRTOS task and queue functions and BinSentry's debugASSERT, which the
 * module calls on code paths the tests don't reach. The definitions are in cellular_bg770_api_utest.c.
 *
 * Force-included into the module's compile (see CMakeLists.txt) so the module builds without implicit
 * function declarations.
 */

#ifndef CELLULAR_BG770_TEST_PLATFORM_H
#define CELLULAR_BG770_TEST_PLATFORM_H

#include "cellular_platform.h"

void vTaskDelay( TickType_t xTicksToDelay );
BaseType_t xQueueReset( QueueHandle_t xQueue );
int debugASSERT( int condition );

#endif /* CELLULAR_BG770_TEST_PLATFORM_H */
