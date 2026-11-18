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
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#include "unity.h"

#include "cellular_config.h"
#include "cellular_config_defaults.h"

/* Include paths for public enums, structures, and macros. */
#include "cellular_platform.h"
#include "cellular_common_internal.h"
#include "cellular_types.h"
#include "cellular_api.h"
#include "cellular_bg770.h"
#include "cellular_common_portable.h"

#include "mock_cellular_common.h"
#include "mock_cellular_common_api.h"
#include "mock_cellular_at_core.h"

#define TEST_FILE_NAME           "UFS:root.crt"
#define TEST_QFCRC_COMMAND       "AT+QFCRC=\"" TEST_FILE_NAME "\""
#define TEST_QFDEL_COMMAND       "AT+QFDEL=\"" TEST_FILE_NAME "\""
#define TEST_CRC32_SENTINEL      ( 0xA5A5A5A5UL )

/* Prototypes of the definitions below, shared with the module's compile. */
#include "cellular_bg770_test_platform.h"

static CellularContext_t context;

/* The modem error response the mocked AT command request returns. */
static CellularPktStatus_t requestPktStatus = CELLULAR_PKT_STATUS_OK;
static bool errorResponseReceived = false;
static bool errorResponseTruncated = false;
static char errorResponseLine[ CELLULAR_AT_ERROR_RESPONSE_MAX_LENGTH ];
static const char * pExpectedAtCmd = NULL;

/* The response line the mocked AT command request delivers to the request's callback on success, NULL for none.
 * Copied to a writable buffer first since the response parsers modify the line in place. */
static const char * pResponseLine = NULL;
static char responseLineBuffer[ 64 ];

/* Cellular_ModuleEnableUE support: the module context handed out by the mocked _Cellular_GetModuleContext, the
 * result of the APP_RDY wait, and every vTaskDelay the code under test asked for. Both are unexpected in the file
 * command tests, so they fail there unless a test allows them. */
static cellularModuleContext_t moduleContext;
static bool waitBitsAllowed = false;
static uint16_t waitBitsResult = 0;
static bool delaysAllowed = false;
#define MAX_RECORDED_DELAYS    ( 16U )
static TickType_t recordedDelayTicks[ MAX_RECORDED_DELAYS ];
static size_t recordedDelayCount = 0;

/* A scripted AT command exchange: what Cellular_ModuleEnableUE is expected to send next, and the packet status the
 * mocked request answers with. The script is consumed in order; a command that doesn't match or goes past the end
 * fails the test. */
typedef struct ScriptedCommand
{
    const char * pAtCmd;
    CellularATCommandType_t atCmdType;
    const char * pAtRspPrefix;          /* NULL for none */
    bool withTimeout;                   /* sent through _Cellular_TimeoutAtcmdRequestWithCallback (true) or _Cellular_AtcmdRequestWithCallback */
    uint32_t timeoutMS;                 /* only checked when withTimeout */
    CellularPktStatus_t pktStatus;
} ScriptedCommand_t;

static const ScriptedCommand_t * pScript = NULL;
static size_t scriptLength = 0;
static size_t scriptIndex = 0;

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
    pResponseLine = NULL;
    memset( &moduleContext, 0, sizeof( moduleContext ) );
    waitBitsAllowed = false;
    waitBitsResult = 0;
    delaysAllowed = false;
    memset( recordedDelayTicks, 0, sizeof( recordedDelayTicks ) );
    recordedDelayCount = 0;
    pScript = NULL;
    scriptLength = 0;
    scriptIndex = 0;
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
    if( waitBitsAllowed == false )
    {
        TEST_FAIL_MESSAGE( "Unexpected call to PlatformEventGroup_WaitBits" );
    }

    /* Cellular_ModuleEnableUE waits up to 10 s for the APP_RDY URC on the module's init event, clearing the bit. */
    TEST_ASSERT_EQUAL_PTR( moduleContext.pInitEvent, groupEvent );
    TEST_ASSERT_EQUAL( INIT_EVT_MASK_APP_RDY_RECEIVED, uxBitsToWaitFor );
    TEST_ASSERT_EQUAL( pdTRUE, xClearOnExit );
    TEST_ASSERT_EQUAL( pdFALSE, xWaitForAllBits );
    TEST_ASSERT_EQUAL( pdMS_TO_TICKS( 10000U ), xTicksToWait );

    return waitBitsResult;
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
    if( delaysAllowed == false )
    {
        TEST_FAIL_MESSAGE( "Unexpected call to vTaskDelay" );
    }

    TEST_ASSERT_TRUE_MESSAGE( recordedDelayCount < MAX_RECORDED_DELAYS, "More vTaskDelay calls than recorded" );
    recordedDelayTicks[ recordedDelayCount ] = xTicksToDelay;
    recordedDelayCount++;
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
    else if( ( pResponseLine != NULL ) && ( atReq.respCallback != NULL ) )
    {
        /* Deliver the response line to the request's callback the way the packet handler does, the request's
         * status is the callback's. */
        CellularATCommandLine_t responseItem = { .pNext = NULL, .pLine = responseLineBuffer };
        CellularATCommandResponse_t response = { .status = true, .pItm = &responseItem };

        TEST_ASSERT_TRUE( strlen( pResponseLine ) < sizeof( responseLineBuffer ) );
        ( void ) strcpy( responseLineBuffer, pResponseLine );

        return atReq.respCallback( pContext, &response, atReq.pData, atReq.dataLen );
    }

    return requestPktStatus;
}

/* ========================================================================== */

/* cellular_at_core.h is mocked, these stubs give the response parsers the real functions' behaviour for the
 * well-formed and malformed lines the tests use (see cellular_at_core.c). */

static CellularATError_t _Cellular_ATRemovePrefix_Real( char ** ppString,
                                                       int cmock_num_calls )
{
    char * pColon = NULL;

    ( void ) cmock_num_calls;

    if( ( ppString == NULL ) || ( *ppString == NULL ) )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    pColon = strchr( *ppString, ':' );

    if( pColon == NULL )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    *ppString = pColon + 1;
    return CELLULAR_AT_SUCCESS;
}

static CellularATError_t _removeAllMatching( char * pString,
                                             int ( * matches )( int ) )
{
    char * pRead = pString;
    char * pWrite = pString;

    if( pString == NULL )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    for( ; *pRead != '\0'; pRead++ )
    {
        if( matches( ( unsigned char ) *pRead ) == 0 )
        {
            *pWrite = *pRead;
            pWrite++;
        }
    }

    *pWrite = '\0';
    return CELLULAR_AT_SUCCESS;
}

static int _isDoubleQuote( int c )
{
    return ( c == '"' ) ? 1 : 0;
}

static CellularATError_t _Cellular_ATRemoveAllDoubleQuote_Real( char * pString,
                                                               int cmock_num_calls )
{
    ( void ) cmock_num_calls;
    return _removeAllMatching( pString, _isDoubleQuote );
}

static CellularATError_t _Cellular_ATRemoveAllWhiteSpaces_Real( char * pString,
                                                               int cmock_num_calls )
{
    ( void ) cmock_num_calls;
    return _removeAllMatching( pString, isspace );
}

static CellularATError_t _Cellular_ATGetNextTok_Real( char ** ppString,
                                                     char ** ppTokOutput,
                                                     int cmock_num_calls )
{
    char * pToken = NULL;
    char * pDelimiter = NULL;

    ( void ) cmock_num_calls;

    if( ( ppString == NULL ) || ( *ppString == NULL ) || ( ppTokOutput == NULL ) )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    pToken = *ppString;

    if( *pToken == '\0' )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    /* Terminate this token at the delimiter and move the string on past it, or to its end. */
    pDelimiter = strchr( pToken, ',' );

    if( pDelimiter != NULL )
    {
        *pDelimiter = '\0';
        *ppString = pDelimiter + 1;
    }
    else
    {
        *ppString = pToken + strlen( pToken );
    }

    *ppTokOutput = pToken;
    return CELLULAR_AT_SUCCESS;
}

static CellularATError_t _Cellular_ATStrtoui_Real( const char * pStr,
                                                  int32_t base,
                                                  uint32_t * pResult,
                                                  int cmock_num_calls )
{
    char * pEnd = NULL;
    long long value = 0;

    ( void ) cmock_num_calls;

    if( ( pStr == NULL ) || ( pResult == NULL ) )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    value = strtoll( pStr, &pEnd, base );

    if( ( pEnd == pStr ) || ( *pEnd != '\0' ) || ( value < 0 ) || ( value > ( long long ) UINT32_MAX ) )
    {
        return CELLULAR_AT_ERROR;
    }

    *pResult = ( uint32_t ) value;
    return CELLULAR_AT_SUCCESS;
}

static CellularATError_t _Cellular_ATStrtoi_Real( const char * pStr,
                                                 int32_t base,
                                                 int32_t * pResult,
                                                 int cmock_num_calls )
{
    char * pEnd = NULL;
    long long value = 0;

    ( void ) cmock_num_calls;

    if( ( pStr == NULL ) || ( pResult == NULL ) )
    {
        return CELLULAR_AT_BAD_PARAMETER;
    }

    value = strtoll( pStr, &pEnd, base );

    if( ( pEnd == pStr ) || ( *pEnd != '\0' ) || ( value < INT32_MIN ) || ( value > INT32_MAX ) )
    {
        return CELLULAR_AT_ERROR;
    }

    *pResult = ( int32_t ) value;
    return CELLULAR_AT_SUCCESS;
}

/* Sets up the modem to answer the next request with pLine, parsed by the real-behaviour stubs above. */
static void _setModemResponse( const char * pLine )
{
    requestPktStatus = CELLULAR_PKT_STATUS_OK;
    pResponseLine = pLine;
    Cellular_ATRemovePrefix_StubWithCallback( _Cellular_ATRemovePrefix_Real );
    Cellular_ATRemoveAllDoubleQuote_StubWithCallback( _Cellular_ATRemoveAllDoubleQuote_Real );
    Cellular_ATRemoveAllWhiteSpaces_StubWithCallback( _Cellular_ATRemoveAllWhiteSpaces_Real );
    Cellular_ATGetNextTok_StubWithCallback( _Cellular_ATGetNextTok_Real );
    Cellular_ATStrtoui_StubWithCallback( _Cellular_ATStrtoui_Real );
    Cellular_ATStrtoi_StubWithCallback( _Cellular_ATStrtoi_Real );
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

/* No error response was received from the modem, so the packet status is translated. */
static void _assertGetModemFileCRC32ErrorTranslated( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    cellularStatus = _getModemFileCRC32( &crc32, true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
}

/* The modem answered with an error final result that is not file not found, the packet status is not translated. */
static void _assertGetModemFileCRC32ModemErrorResponse( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    cellularStatus = _getModemFileCRC32( &crc32, false );

    TEST_ASSERT_EQUAL( CELLULAR_MODEM_ERROR_RESPONSE, cellularStatus );
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
 * @brief Test that a successful AT+QFCRC request returns success and the CRC32 parsed from the response.
 */
void test_Cellular_GetModemFileCRC32_Success( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _setModemResponse( "+QFCRC: D5E1FF1D,1A2B,3C4D" );

    cellularStatus = _getModemFileCRC32( &crc32, false );

    TEST_ASSERT_EQUAL( CELLULAR_SUCCESS, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0xD5E1FF1DUL, crc32 );
}

/**
 * @brief Test that the documented response for an empty file ("+QFCRC: 0,0,0") is a success with CRC32 0.
 */
void test_Cellular_GetModemFileCRC32_Empty_File( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _setModemResponse( "+QFCRC: 0,0,0" );

    cellularStatus = _getModemFileCRC32( &crc32, false );

    TEST_ASSERT_EQUAL( CELLULAR_SUCCESS, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0, crc32 );
}

/**
 * @brief Test that a request ending OK without a +QFCRC line (a status that wasn't this request's answer) is a
 * failure reporting CRC32 0, not a success with a CRC32 of 0.
 */
void test_Cellular_GetModemFileCRC32_Ok_Without_Response_Line( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_ModemResponse );
    pExpectedAtCmd = TEST_QFCRC_COMMAND;
    /* No response line: the stub returns OK without delivering anything to the callback. */
    _Cellular_TranslatePktStatus_ExpectAndReturn( CELLULAR_PKT_STATUS_BAD_RESPONSE, CELLULAR_INTERNAL_FAILURE );

    cellularStatus = Cellular_GetModemFileCRC32( &context, TEST_FILE_NAME, &crc32 );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0, crc32 );
}

/**
 * @brief Test that a response with a missing CRC is a failure, not a file not found, and reports CRC32 0.
 */
void test_Cellular_GetModemFileCRC32_Response_Missing_Token( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _setModemResponse( "+QFCRC: D5E1FF1D,1A2B" );

    cellularStatus = _getModemFileCRC32( &crc32, true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0, crc32 );
}

/**
 * @brief Test that a response whose CRC32 isn't hexadecimal is a failure, not a file not found.
 */
void test_Cellular_GetModemFileCRC32_Response_Not_Hex( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    _setModemResponse( "+QFCRC: XYZ,1A2B,3C4D" );

    cellularStatus = _getModemFileCRC32( &crc32, true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0, crc32 );
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
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a code ending in 405 is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Code_Ending_In_405( void )
{
    _setModemErrorResponse( "+CME ERROR: 1405" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that the largest uint32_t CME error code is parsed and not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Max_Code( void )
{
    _setModemErrorResponse( "+CME ERROR: 4294967295" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a CME error code that overflows uint32_t is rejected, 4294967701 wraps to 405.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Code_Overflow( void )
{
    _setModemErrorResponse( "+CME ERROR: 4294967701" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a negative CME error code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Negative_Code( void )
{
    _setModemErrorResponse( "+CME ERROR: -405" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that the verbose (AT+CMEE=2) format is not parsed.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Verbose_Format( void )
{
    _setModemErrorResponse( "+CME ERROR: File not found" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that trailing characters after the CME error code are rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Trailing_Characters( void )
{
    _setModemErrorResponse( "+CME ERROR: 405x" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a trailing space after the CME error code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Trailing_Space( void )
{
    _setModemErrorResponse( "+CME ERROR: 405 " );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a CME error prefix without a code is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_No_Code( void )
{
    _setModemErrorResponse( "+CME ERROR:" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a CME error prefix followed only by spaces is rejected.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Only_Spaces( void )
{
    _setModemErrorResponse( "+CME ERROR:   " );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that the prefix match is case sensitive.
 */
void test_Cellular_GetModemFileCRC32_Cme_Error_Lower_Case_Prefix( void )
{
    _setModemErrorResponse( "+cme error: 405" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a plain ERROR final result is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Plain_Error( void )
{
    _setModemErrorResponse( "ERROR" );
    _assertGetModemFileCRC32ModemErrorResponse();
}

/**
 * @brief Test that a CMS error with code 405 is not mapped to file not found.
 */
void test_Cellular_GetModemFileCRC32_Cms_Error( void )
{
    _setModemErrorResponse( "+CMS ERROR: 405" );
    _assertGetModemFileCRC32ModemErrorResponse();
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
 * @brief Test that a request the modem didn't answer is passed through as the library's translation of the
 * timeout (CELLULAR_TIMEOUT), with CRC32 0: the driver relies on it to report nothing is known about the file.
 */
void test_Cellular_GetModemFileCRC32_No_Answer_Is_A_Timeout( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint32_t crc32 = TEST_CRC32_SENTINEL;

    requestPktStatus = CELLULAR_PKT_STATUS_TIMED_OUT;
    errorResponseReceived = false;

    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_ModemResponse );
    pExpectedAtCmd = TEST_QFCRC_COMMAND;
    _Cellular_TranslatePktStatus_ExpectAndReturn( CELLULAR_PKT_STATUS_TIMED_OUT, CELLULAR_TIMEOUT );

    cellularStatus = Cellular_GetModemFileCRC32( &context, TEST_FILE_NAME, &crc32 );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
    TEST_ASSERT_EQUAL_HEX32( 0, crc32 );
}

/**
 * @brief Test that a truncated error line is rejected, the code could have more digits.
 */
void test_Cellular_GetModemFileCRC32_Error_Response_Truncated( void )
{
    _setModemErrorResponse( "+CME ERROR: 405" );
    errorResponseTruncated = true;
    _assertGetModemFileCRC32ModemErrorResponse();
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

    _assertGetModemFileCRC32ModemErrorResponse();
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
 * @brief Test that a different CME error code is a modem error response, not file not found.
 */
void test_Cellular_DeleteFileOnModem_Cme_Error_Other_Code( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _setModemErrorResponse( "+CME ERROR: 406" );

    cellularStatus = _deleteFileOnModem( false );

    TEST_ASSERT_EQUAL( CELLULAR_MODEM_ERROR_RESPONSE, cellularStatus );
}

/**
 * @brief Test that a plain ERROR final result is a modem error response, not file not found.
 */
/**
 * @brief Test that a delete the modem didn't answer is passed through as the library's translation of the timeout
 * (CELLULAR_TIMEOUT), not as a modem error response.
 */
void test_Cellular_DeleteFileOnModem_No_Answer_Is_A_Timeout( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    requestPktStatus = CELLULAR_PKT_STATUS_TIMED_OUT;
    errorResponseReceived = false;

    _Cellular_CheckLibraryStatus_ExpectAndReturn( &context, CELLULAR_SUCCESS );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_ModemResponse );
    pExpectedAtCmd = TEST_QFDEL_COMMAND;
    _Cellular_TranslatePktStatus_ExpectAndReturn( CELLULAR_PKT_STATUS_TIMED_OUT, CELLULAR_TIMEOUT );

    cellularStatus = Cellular_DeleteFileOnModem( &context, TEST_FILE_NAME );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
}

void test_Cellular_DeleteFileOnModem_Plain_Error( void )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    _setModemErrorResponse( "ERROR" );

    cellularStatus = _deleteFileOnModem( false );

    TEST_ASSERT_EQUAL( CELLULAR_MODEM_ERROR_RESPONSE, cellularStatus );
}

/* ========================================================================== */
/* Cellular_ModuleEnableUE: the AT+CMEE=1 step (numeric +CME ERROR codes, which the file commands depend on).      */
/* ========================================================================== */

static CellularError_t _Cellular_GetModuleContext_ReturnModuleContext( const CellularContext_t * pContext,
                                                                       void ** ppModuleContext,
                                                                       int cmock_num_calls )
{
    ( void ) cmock_num_calls;

    TEST_ASSERT_EQUAL_PTR( &context, pContext );
    TEST_ASSERT_NOT_NULL( ppModuleContext );
    *ppModuleContext = &moduleContext;

    return CELLULAR_SUCCESS;
}

/* The library's translation, enough of it for these tests. */
static CellularError_t _Cellular_TranslatePktStatus_Library( CellularPktStatus_t status,
                                                             int cmock_num_calls )
{
    CellularError_t cellularStatus = CELLULAR_INTERNAL_FAILURE;

    ( void ) cmock_num_calls;

    if( status == CELLULAR_PKT_STATUS_OK )
    {
        cellularStatus = CELLULAR_SUCCESS;
    }
    else if( status == CELLULAR_PKT_STATUS_TIMED_OUT )
    {
        cellularStatus = CELLULAR_TIMEOUT;
    }

    return cellularStatus;
}

/* Checks the request against the next scripted command and answers with its packet status. */
static CellularPktStatus_t _nextScriptedCommand( const CellularContext_t * pContext,
                                                 const CellularAtReq_t * pAtReq,
                                                 bool withTimeout,
                                                 uint32_t timeoutMS )
{
    const ScriptedCommand_t * pCommand = NULL;

    TEST_ASSERT_EQUAL_PTR( &context, pContext );
    TEST_ASSERT_NOT_NULL( pAtReq->pAtCmd );

    if( scriptIndex >= scriptLength )
    {
        char message[ 96 ];

        ( void ) snprintf( message, sizeof( message ), "Command sent past the end of the script: %s", pAtReq->pAtCmd );
        TEST_FAIL_MESSAGE( message );
    }

    pCommand = &pScript[ scriptIndex ];
    scriptIndex++;

    TEST_ASSERT_EQUAL_STRING( pCommand->pAtCmd, pAtReq->pAtCmd );
    TEST_ASSERT_EQUAL( pCommand->atCmdType, pAtReq->atCmdType );

    if( pCommand->pAtRspPrefix == NULL )
    {
        TEST_ASSERT_NULL( pAtReq->pAtRspPrefix );
    }
    else
    {
        TEST_ASSERT_EQUAL_STRING( pCommand->pAtRspPrefix, pAtReq->pAtRspPrefix );
    }

    /* The enable sequence doesn't capture error responses. */
    TEST_ASSERT_NULL( pAtReq->pErrorResponse );

    TEST_ASSERT_EQUAL( pCommand->withTimeout, withTimeout );

    if( withTimeout == true )
    {
        TEST_ASSERT_EQUAL( pCommand->timeoutMS, timeoutMS );
    }

    return pCommand->pktStatus;
}

static CellularPktStatus_t _Cellular_TimeoutAtcmdRequestWithCallback_Scripted( CellularContext_t * pContext,
                                                                              CellularAtReq_t atReq,
                                                                              uint32_t timeoutMS,
                                                                              int cmock_num_calls )
{
    ( void ) cmock_num_calls;

    return _nextScriptedCommand( pContext, &atReq, true, timeoutMS );
}

static CellularPktStatus_t _Cellular_AtcmdRequestWithCallback_Scripted( CellularContext_t * pContext,
                                                                       CellularAtReq_t atReq,
                                                                       int cmock_num_calls )
{
    ( void ) cmock_num_calls;

    return _nextScriptedCommand( pContext, &atReq, false, 0U );
}

/* Runs Cellular_ModuleEnableUE against the script with the APP_RDY URC received (or not), and checks that the whole
 * script was consumed. */
static CellularError_t _enableUEWithScript( const ScriptedCommand_t * pCommands,
                                            size_t commandCount,
                                            bool appReadyReceived )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    pScript = pCommands;
    scriptLength = commandCount;
    scriptIndex = 0;
    moduleContext.pInitEvent = ( PlatformEventGroupHandle_t ) 0x1234;
    waitBitsAllowed = true;
    waitBitsResult = appReadyReceived ? ( uint16_t ) INIT_EVT_MASK_APP_RDY_RECEIVED : 0U;
    delaysAllowed = true;

    _Cellular_GetModuleContext_Stub( _Cellular_GetModuleContext_ReturnModuleContext );
    _Cellular_TranslatePktStatus_Stub( _Cellular_TranslatePktStatus_Library );
    _Cellular_TimeoutAtcmdRequestWithCallback_Stub( _Cellular_TimeoutAtcmdRequestWithCallback_Scripted );
    _Cellular_AtcmdRequestWithCallback_Stub( _Cellular_AtcmdRequestWithCallback_Scripted );

    cellularStatus = Cellular_ModuleEnableUE( &context );

    TEST_ASSERT_EQUAL_MESSAGE( scriptLength, scriptIndex, "Not every scripted command was sent" );

    return cellularStatus;
}

/* Checks the recorded vTaskDelay calls, in order. */
static void _assertDelays( const TickType_t * pExpectedTicks,
                           size_t expectedCount )
{
    size_t i = 0;

    TEST_ASSERT_EQUAL( expectedCount, recordedDelayCount );

    for( i = 0; i < expectedCount; i++ )
    {
        TEST_ASSERT_EQUAL_MESSAGE( pExpectedTicks[ i ], recordedDelayTicks[ i ], "Delay at this index differs" );
    }
}

/* The enable sequence up to AT+CMEE=1: the modem probe (short timeout, short backoff base), echo off, DTR off. */
#define SCRIPT_PROBE_AT         { "AT", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, 1000U, CELLULAR_PKT_STATUS_OK }
#define SCRIPT_ECHO_OFF         { "ATE0", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_OK }
#define SCRIPT_DTR_OFF          { "AT&D0", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_OK }
#define SCRIPT_CMEE( status )   { "AT+CMEE=1", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, ( status ) }

/* The 5 s settle after APP_RDY, then the three increasing backoffs (1 s, 4 s, 9 s) of a four-try command. */
#define DELAY_POST_APP_READY    pdMS_TO_TICKS( 5000U )
#define DELAY_BACKOFF_1         pdMS_TO_TICKS( 1000U )
#define DELAY_BACKOFF_2         pdMS_TO_TICKS( 4000U )
#define DELAY_BACKOFF_3         pdMS_TO_TICKS( 9000U )
#define DELAY_SHORT             pdMS_TO_TICKS( 10U )

/* The flow control step that follows AT+CMEE=1, all unanswered so the sequence ends there with CELLULAR_TIMEOUT:
 * the query's four tries, then the one-shot set. */
#define SCRIPT_FLOW_CONTROL_STEP_UNANSWERED \
        { "AT+IFC?", CELLULAR_AT_WITH_PREFIX, "+IFC", true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_TIMED_OUT }, \
        { "AT+IFC?", CELLULAR_AT_WITH_PREFIX, "+IFC", true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_TIMED_OUT }, \
        { "AT+IFC?", CELLULAR_AT_WITH_PREFIX, "+IFC", true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_TIMED_OUT }, \
        { "AT+IFC?", CELLULAR_AT_WITH_PREFIX, "+IFC", true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_TIMED_OUT }, \
        { "AT+IFC=2,2", CELLULAR_AT_NO_RESULT, NULL, false, 0U, CELLULAR_PKT_STATUS_TIMED_OUT }

/* The delays of that step: the 10 ms settle, then the query's three backoffs. */
#define DELAYS_FLOW_CONTROL_STEP_UNANSWERED    DELAY_SHORT, DELAY_BACKOFF_1, DELAY_BACKOFF_2, DELAY_BACKOFF_3

/**
 * @brief Test that AT+CMEE=1 is sent right after AT&D0 with the enable sequence's request shape, that a modem that
 * doesn't answer it is given four tries with increasing backoff, and that the sequence then continues without it
 * (to the flow control step, whose failure is used here to end the sequence).
 */
void test_Cellular_ModuleEnableUE_Cmee_Sent_After_Dtr_Off_And_Four_Timeouts_Dont_Stop_The_Sequence( void )
{
    static const ScriptedCommand_t script[] =
    {
        SCRIPT_PROBE_AT,
        SCRIPT_ECHO_OFF,
        SCRIPT_DTR_OFF,
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_TIMED_OUT ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_TIMED_OUT ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_TIMED_OUT ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_TIMED_OUT ),
        SCRIPT_FLOW_CONTROL_STEP_UNANSWERED,
    };
    static const TickType_t expectedDelays[] =
    {
        DELAY_POST_APP_READY, DELAY_BACKOFF_1, DELAY_BACKOFF_2, DELAY_BACKOFF_3,
        DELAYS_FLOW_CONTROL_STEP_UNANSWERED
    };
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _enableUEWithScript( script, sizeof( script ) / sizeof( script[ 0 ] ), true );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
    _assertDelays( expectedDelays, sizeof( expectedDelays ) / sizeof( expectedDelays[ 0 ] ) );
}

/**
 * @brief Test that a modem answering AT+CMEE=1 with an error on all four tries doesn't stop the sequence either.
 */
void test_Cellular_ModuleEnableUE_Cmee_Four_Error_Responses_Dont_Stop_The_Sequence( void )
{
    static const ScriptedCommand_t script[] =
    {
        SCRIPT_PROBE_AT,
        SCRIPT_ECHO_OFF,
        SCRIPT_DTR_OFF,
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_FAILURE ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_FAILURE ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_FAILURE ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_FAILURE ),
        SCRIPT_FLOW_CONTROL_STEP_UNANSWERED,
    };
    static const TickType_t expectedDelays[] =
    {
        DELAY_POST_APP_READY, DELAY_BACKOFF_1, DELAY_BACKOFF_2, DELAY_BACKOFF_3,
        DELAYS_FLOW_CONTROL_STEP_UNANSWERED
    };
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _enableUEWithScript( script, sizeof( script ) / sizeof( script[ 0 ] ), true );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
    _assertDelays( expectedDelays, sizeof( expectedDelays ) / sizeof( expectedDelays[ 0 ] ) );
}

/**
 * @brief Test that AT+CMEE=1 answered on its third try is accepted: the sequence goes on to the flow control step
 * (whose failure is used here to end the sequence) instead of giving up on it.
 */
void test_Cellular_ModuleEnableUE_Cmee_Transient_Failure_Is_Retried_Then_The_Sequence_Continues( void )
{
    static const ScriptedCommand_t script[] =
    {
        SCRIPT_PROBE_AT,
        SCRIPT_ECHO_OFF,
        SCRIPT_DTR_OFF,
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_FAILURE ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_TIMED_OUT ),
        SCRIPT_CMEE( CELLULAR_PKT_STATUS_OK ),
        SCRIPT_FLOW_CONTROL_STEP_UNANSWERED,
    };
    static const TickType_t expectedDelays[] =
    {
        DELAY_POST_APP_READY, DELAY_BACKOFF_1, DELAY_BACKOFF_2,
        DELAYS_FLOW_CONTROL_STEP_UNANSWERED
    };
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _enableUEWithScript( script, sizeof( script ) / sizeof( script[ 0 ] ), true );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
    _assertDelays( expectedDelays, sizeof( expectedDelays ) / sizeof( expectedDelays[ 0 ] ) );
}

/**
 * @brief Test that AT+CMEE=1 is not sent when AT&D0 before it failed on all four tries.
 */
void test_Cellular_ModuleEnableUE_Cmee_Not_Sent_When_Dtr_Off_Fails( void )
{
    static const ScriptedCommand_t script[] =
    {
        SCRIPT_PROBE_AT,
        SCRIPT_ECHO_OFF,
        { "AT&D0", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_FAILURE },
        { "AT&D0", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_FAILURE },
        { "AT&D0", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_FAILURE },
        { "AT&D0", CELLULAR_AT_NO_RESULT, NULL, true, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS, CELLULAR_PKT_STATUS_FAILURE },
    };
    static const TickType_t expectedDelays[] =
    {
        DELAY_POST_APP_READY, DELAY_BACKOFF_1, DELAY_BACKOFF_2, DELAY_BACKOFF_3
    };
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _enableUEWithScript( script, sizeof( script ) / sizeof( script[ 0 ] ), true );

    TEST_ASSERT_EQUAL( CELLULAR_INTERNAL_FAILURE, cellularStatus );
    _assertDelays( expectedDelays, sizeof( expectedDelays ) / sizeof( expectedDelays[ 0 ] ) );
}

/**
 * @brief Test that the sequence still probes the modem when the APP_RDY URC never came, with the probe's own
 * short timeout and backoff (100 ms base), and stops when the probe gets no answer.
 */
void test_Cellular_ModuleEnableUE_Probes_Modem_Without_App_Ready( void )
{
    static const ScriptedCommand_t script[] =
    {
        { "AT", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, 1000U, CELLULAR_PKT_STATUS_TIMED_OUT },
        { "AT", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, 1000U, CELLULAR_PKT_STATUS_TIMED_OUT },
        { "AT", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, 1000U, CELLULAR_PKT_STATUS_TIMED_OUT },
        { "AT", CELLULAR_AT_MULTI_WO_PREFIX, NULL, true, 1000U, CELLULAR_PKT_STATUS_TIMED_OUT },
    };
    static const TickType_t expectedDelays[] =
    {
        DELAY_POST_APP_READY, pdMS_TO_TICKS( 100U ), pdMS_TO_TICKS( 400U ), pdMS_TO_TICKS( 900U )
    };
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    cellularStatus = _enableUEWithScript( script, sizeof( script ) / sizeof( script[ 0 ] ), false );

    TEST_ASSERT_EQUAL( CELLULAR_TIMEOUT, cellularStatus );
    _assertDelays( expectedDelays, sizeof( expectedDelays ) / sizeof( expectedDelays[ 0 ] ) );
}
