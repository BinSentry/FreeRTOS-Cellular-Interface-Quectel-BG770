/**
 * @file cellular_bg770_api_utest.c
 * @brief Unit tests for the BG770 modem file functions in cellular_bg770_api.c.
 *
 * The CME error parser (_tryParseCmeErrorCode) is static, so it is tested through
 * Cellular_GetModemFileCRC32 and Cellular_DeleteFileOnModem. A request that fails with
 * "+CME ERROR: 405" (file not found) must return CELLULAR_FILE_NOT_FOUND, any other
 * error must be translated with _Cellular_TranslatePktStatus.
 */

#include <string.h>
#include <stdbool.h>

#include "unity.h"

#include "cellular_config.h"
#include "cellular_config_defaults.h"

/* Include paths for public enums, structures, and macros. */
#include "cellular_platform.h"
#include "cellular_common_internal.h"
#include "cellular_types.h"
#include "cellular_api.h"
#include "cellular_bg770.h"

#include "mock_cellular_common.h"
#include "mock_cellular_common_api.h"
#include "mock_cellular_at_core.h"

#define TEST_FILE_NAME           "UFS:root.crt"
#define TEST_QFCRC_COMMAND       "AT+QFCRC=\"" TEST_FILE_NAME "\""
#define TEST_QFDEL_COMMAND       "AT+QFDEL=\"" TEST_FILE_NAME "\""
#define TEST_CRC32_SENTINEL      ( 0xA5A5A5A5UL )

/* Not declared by the test platform headers, used by code paths these tests don't reach. */
void vTaskDelay( TickType_t xTicksToDelay );
BaseType_t xQueueReset( QueueHandle_t xQueue );
int debugASSERT( int condition );

static CellularContext_t context;

/* The modem error response the mocked AT command request returns. */
static CellularPktStatus_t requestPktStatus = CELLULAR_PKT_STATUS_OK;
static bool errorResponseReceived = false;
static bool errorResponseTruncated = false;
static char errorResponseLine[ CELLULAR_AT_ERROR_RESPONSE_MAX_LENGTH ];
static const char * pExpectedAtCmd = NULL;

/* ============================   UNITY FIXTURES ============================ */

/* Called before each test method. */
void setUp()
{
    memset( &context, 0, sizeof( context ) );
    requestPktStatus = CELLULAR_PKT_STATUS_OK;
    errorResponseReceived = false;
    errorResponseTruncated = false;
    memset( errorResponseLine, 0, sizeof( errorResponseLine ) );
    pExpectedAtCmd = NULL;
}

/* Called after each test method. */
void tearDown()
{
}

/* Called at the beginning of the whole suite. */
void suiteSetUp()
{
}

/* Called at the end of the whole suite. */
int suiteTearDown( int numFailures )
{
    return numFailures;
}

/* ========================================================================== */

void MockPlatformMutex_Unlock( PlatformMutex_t * pMutex )
{
    ( void ) pMutex;
}

void MockPlatformMutex_Lock( PlatformMutex_t * pMutex )
{
    ( void ) pMutex;
}

bool MockPlatformMutex_Create( PlatformMutex_t * pNewMutex,
                               bool recursive )
{
    ( void ) recursive;
    pNewMutex->created = true;
    return true;
}

void MockPlatformMutex_Destroy( PlatformMutex_t * pMutex )
{
    pMutex->created = false;
}

QueueHandle_t MockxQueueCreate( int32_t uxQueueLength,
                                uint32_t uxItemSize )
{
    ( void ) uxQueueLength;
    ( void ) uxItemSize;
    TEST_FAIL_MESSAGE( "Unexpected call to xQueueCreate" );
    return NULL;
}

uint16_t MockvQueueDelete( QueueHandle_t queue )
{
    ( void ) queue;
    TEST_FAIL_MESSAGE( "Unexpected call to vQueueDelete" );
    return 0;
}

BaseType_t MockxQueueSend( QueueHandle_t queue,
                           void * data,
                           uint32_t time )
{
    ( void ) queue;
    ( void ) data;
    ( void ) time;
    TEST_FAIL_MESSAGE( "Unexpected call to xQueueSend" );
    return 0;
}

BaseType_t MockxQueueReceive( QueueHandle_t queue,
                              void * data,
                              uint32_t time )
{
    ( void ) queue;
    ( void ) data;
    ( void ) time;
    TEST_FAIL_MESSAGE( "Unexpected call to xQueueReceive" );
    return 0;
}

BaseType_t xQueueReset( QueueHandle_t xQueue )
{
    ( void ) xQueue;
    TEST_FAIL_MESSAGE( "Unexpected call to xQueueReset" );
    return 0;
}

MockPlatformEventGroupHandle_t MockPlatformEventGroup_Create( void )
{
    TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_Create" );
    return NULL;
}

uint16_t MockPlatformEventGroup_Delete( PlatformEventGroupHandle_t groupEvent )
{
    ( void ) groupEvent;
    TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_Delete" );
    return 0;
}

uint16_t MockPlatformEventGroup_WaitBits( PlatformEventGroupHandle_t groupEvent,
                                          EventBits_t uxBitsToWaitFor,
                                          BaseType_t xClearOnExit,
                                          BaseType_t xWaitForAllBits,
                                          TickType_t xTicksToWait )
{
    ( void ) groupEvent;
    ( void ) uxBitsToWaitFor;
    ( void ) xClearOnExit;
    ( void ) xWaitForAllBits;
    ( void ) xTicksToWait;
    TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_WaitBits" );
    return 0;
}

uint16_t MockPlatformEventGroup_SetBits( PlatformEventGroupHandle_t groupEvent,
                                         EventBits_t event )
{
    ( void ) groupEvent;
    ( void ) event;
    TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_SetBits" );
    return 0;
}

uint16_t MockPlatformEventGroup_ClearBits( PlatformEventGroupHandle_t groupEvent,
                                           TickType_t uxBitsToClear )
{
    ( void ) groupEvent;
    ( void ) uxBitsToClear;
    TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_ClearBits" );
    return 0;
}

void vTaskDelay( TickType_t xTicksToDelay )
{
    ( void ) xTicksToDelay;
    TEST_FAIL_MESSAGE( "Unexpected call to vTaskDelay" );
}

int debugASSERT( int condition )
{
    TEST_ASSERT_TRUE_MESSAGE( condition, "debugASSERT failed" );
    return 0;
}

/* ========================================================================== */

/* Mocked AT command request, captures the configured modem error response like the packet handler does. */
static CellularPktStatus_t _Cellular_AtcmdRequestWithCallback_ModemResponse( CellularContext_t * pContext,
                                                                             CellularAtReq_t atReq,
                                                                             int cmock_num_calls )
{
    ( void ) cmock_num_calls;

    TEST_ASSERT_EQUAL_PTR( &context, pContext );
    TEST_ASSERT_NOT_NULL( atReq.pErrorResponse );
    TEST_ASSERT_FALSE( atReq.pErrorResponse->received );

    if( pExpectedAtCmd != NULL )
    {
        TEST_ASSERT_EQUAL_STRING( pExpectedAtCmd, atReq.pAtCmd );
    }

    if( requestPktStatus != CELLULAR_PKT_STATUS_OK )
    {
        atReq.pErrorResponse->received = errorResponseReceived;
        atReq.pErrorResponse->truncated = errorResponseTruncated;
        memcpy( atReq.pErrorResponse->line, errorResponseLine, sizeof( atReq.pErrorResponse->line ) );
    }

    return requestPktStatus;
}

/* Sets up the modem to fail the next request with a received, untruncated error line. */
static void _setModemErrorResponse( const char * pLine )
{
    requestPktStatus = CELLULAR_PKT_STATUS_FAILURE;
    errorResponseReceived = true;
    errorResponseTruncated = false;
    memset( errorResponseLine, 0, sizeof( errorResponseLine ) );
    strncpy( errorResponseLine, pLine, sizeof( errorResponseLine ) - 1U );
}

/* Expectations are queued in call order, CMock enforces strict ordering. */
static CellularError_t _getModemFileCRC32( uint32_t * pCrc32,
                                           bool expectPktStatusTranslation )
{
    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_ModemResponse );
    pExpectedAtCmd = TEST_QFCRC_COMMAND;

    if( expectPktStatusTranslation == true )
    {
        _Cellular_TranslatePktStatus_ExpectAndReturn( CELLULAR_PKT_STATUS_FAILURE, CELLULAR_INTERNAL_FAILURE );
    }

    return Cellular_GetModemFileCRC32( &context, TEST_FILE_NAME, pCrc32 );
}

/* The modem error must not be recognised as file not found, so the packet status is translated. */
static void _assertGetModemFileCRC32ErrorTranslated( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    cellularStatus = _getModemFileCRC32( &crc32, true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
}

static void _assertGetModemFileCRC32FileNotFound( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    cellularStatus = _getModemFileCRC32( &crc32, false );

    TEST_ASSERT_EQUAL( CELLULAR_FILE_NOT_FOUND, cellularStatus );
}

/* ========================================================================== */

/**
 * @brief Test that a successful AT+QFCRC request returns success.
 */
void test_Cellular_GetModemFileCRC32_Success( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    cellularStatus = _getModemFileCRC32( &crc32, false );

    TEST_ASSERT_EQUAL( CELLULAR_SUCCESS, cellularStatus );
}

/**
 * @brief Test that a NULL file name is rejected before a request is sent.
 */
void test_Cellular_GetModemFileCRC32_Null_File_Name( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );

    cellularStatus = Cellular_GetModemFileCRC32( &context, NULL, &crc32 );

    TEST_ASSERT_EQUAL( CELLULAR_BAD_PARAMETER, cellularStatus );
}

/**
 * @brief Test that "+CME ERROR: 405" (file not found) returns CELLULAR_FILE_NOT_FOUND.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_File_Not_Found( void )
{
    _setModemErrorResponse( "+CME ERROR: 405" );
    _assertGetModemFileCRC32FileNotFound();
}

/**
 * @brief Test that a CME error code without a space after the colon is parsed.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_File_Not_Found_No_Space( void )
{
    _setModemErrorResponse( "+CME ERROR:405" );
    _assertGetModemFileCRC32FileNotFound();
}

/**
 * @brief Test that a CME error code after multiple spaces is parsed.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_File_Not_Found_Multiple_Spaces( void )
{
    _setModemErrorResponse( "+CME ERROR:   405" );
    _assertGetModemFileCRC32FileNotFound();
}

/**
 * @brief Test that the longest line that fits the error response buffer is parsed.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_File_Not_Found_Max_Length_Line( void )
{
    const char prefix[] = "+CME ERROR: ";
    const char code[] = "405";
    char line[ CELLULAR_AT_ERROR_RESPONSE_MAX_LENGTH ];
    const size_t lineLength = sizeof( line ) - 1U;
    const size_t zeroCount = lineLength - ( sizeof( prefix ) - 1U ) - ( sizeof( code ) - 1U );

    /* "+CME ERROR: 000...0405" filling the buffer, leading zeros don't change the code. */
    memcpy( line, prefix, sizeof( prefix ) - 1U );
    memset( &line[ sizeof( prefix ) - 1U ], '0', zeroCount );
    memcpy( &line[ lineLength - ( sizeof( code ) - 1U ) ], code, sizeof( code ) );
    TEST_ASSERT_EQUAL( lineLength, strlen( line ) );

    _setModemErrorResponse( line );
    _assertGetModemFileCRC32FileNotFound();
}

/**
 * @brief Test that a different CME error code is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Other_Code( void )
{
    _setModemErrorResponse( "+CME ERROR: 406" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a code ending in 405 is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Code_Ending_In_405( void )
{
    _setModemErrorResponse( "+CME ERROR: 1405" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that the largest uint32_t CME error code is parsed and not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Max_Code( void )
{
    _setModemErrorResponse( "+CME ERROR: 4294967295" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a CME error code that overflows uint32_t is rejected, 4294967701 wraps to 405.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Code_Overflow( void )
{
    _setModemErrorResponse( "+CME ERROR: 4294967701" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a negative CME error code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Negative_Code( void )
{
    _setModemErrorResponse( "+CME ERROR: -405" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that the verbose (AT+CMEE=2) format is not parsed.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Verbose_Format( void )
{
    _setModemErrorResponse( "+CME ERROR: File not found" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that trailing characters after the CME error code are rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Trailing_Characters( void )
{
    _setModemErrorResponse( "+CME ERROR: 405x" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a trailing space after the CME error code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Trailing_Space( void )
{
    _setModemErrorResponse( "+CME ERROR: 405 " );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a CME error prefix without a code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_No_Code( void )
{
    _setModemErrorResponse( "+CME ERROR:" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a CME error prefix followed only by spaces is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Only_Spaces( void )
{
    _setModemErrorResponse( "+CME ERROR:   " );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that the prefix match is case sensitive.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Lower_Case_Prefix( void )
{
    _setModemErrorResponse( "+cme error: 405" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a plain ERROR final result is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Plain_Error( void )
{
    _setModemErrorResponse( "ERROR" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a CMS error with code 405 is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cms_Error( void )
{
    _setModemErrorResponse( "+CMS ERROR: 405" );
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that the error line is ignored when no error response was received.
 */
void test_Cellular_GetModemFileCRC32_Error_Response_Not_Received( void )
{
    _setModemErrorResponse( "+CME ERROR: 405" );
    errorResponseReceived = false;
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a truncated error line is rejected, the code could have more digits.
 */
void test_Cellular_GetModemFileCRC32_Error_Response_Truncated( void )
{
    _setModemErrorResponse( "+CME ERROR: 405" );
    errorResponseTruncated = true;
    _assertGetModemFileCRC32ErrorTranslated();
}

/**
 * @brief Test that a line not null terminated within the buffer is rejected.
 */
void test_Cellular_GetModemFileCRC32_Error_Response_Not_Terminated( void )
{
    const char prefix[] = "+CME ERROR: ";
    const char code[] = "405";

    _setModemErrorResponse( "" );

    /* "+CME ERROR: 000...0405" filling the whole buffer, would parse as 405 if read past the buffer end. */
    memcpy( errorResponseLine, prefix, sizeof( prefix ) - 1U );
    memset( &errorResponseLine[ sizeof( prefix ) - 1U ], '0',
            sizeof( errorResponseLine ) - ( sizeof( prefix ) - 1U ) );
    memcpy( &errorResponseLine[ sizeof( errorResponseLine ) - ( sizeof( code ) - 1U ) ], code, sizeof( code ) - 1U );

    _assertGetModemFileCRC32ErrorTranslated();
}

/* ========================================================================== */

/* Expectations are queued in call order, CMock enforces strict ordering. */
static CellularError_t _deleteFileOnModem( bool expectPktStatusTranslation )
{
    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_ModemResponse );
    pExpectedAtCmd = TEST_QFDEL_COMMAND;

    if( expectPktStatusTranslation == true )
    {
        _Cellular_TranslatePktStatus_ExpectAndReturn( CELLULAR_PKT_STATUS_FAILURE, CELLULAR_INTERNAL_FAILURE );
    }

    return Cellular_DeleteFileOnModem( &context, TEST_FILE_NAME );
}

/**
 * @brief Test that a successful AT+QFDEL request returns success.
 */
void test_Cellular_DeleteFileOnModem_Success( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _deleteFileOnModem( false );

    TEST_ASSERT_EQUAL( CELLULAR_SUCCESS, cellularStatus );
}

/**
 * @brief Test that a NULL file name is rejected before a request is sent.
 */
void test_Cellular_DeleteFileOnModem_Null_File_Name( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );

    cellularStatus = Cellular_DeleteFileOnModem( &context, NULL );

    TEST_ASSERT_EQUAL( CELLULAR_BAD_PARAMETER, cellularStatus );
}

/**
 * @brief Test that "+CME ERROR: 405" (file not found) returns CELLULAR_FILE_NOT_FOUND.
 */
void test_Cellular_DeleteFileOnModem_Cme_Error_File_Not_Found( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _setModemErrorResponse( "+CME ERROR: 405" );

    cellularStatus = _deleteFileOnModem( false );

    TEST_ASSERT_EQUAL( CELLULAR_FILE_NOT_FOUND, cellularStatus );
}

/**
 * @brief Test that a different CME error code is translated, not mapped to file not found.
 */
void test_Cellular_DeleteFileOnModem_Cme_Error_Other_Code( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _setModemErrorResponse( "+CME ERROR: 406" );

    cellularStatus = _deleteFileOnModem( true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
}

/**
 * @brief Test that a plain ERROR final result is translated, not mapped to file not found.
 */
void test_Cellular_DeleteFileOnModem_Plain_Error( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _setModemErrorResponse( "ERROR" );

    cellularStatus = _deleteFileOnModem( true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
}
