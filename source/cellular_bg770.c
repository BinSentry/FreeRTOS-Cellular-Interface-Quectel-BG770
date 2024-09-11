/*
 * FreeRTOS-Cellular-Interface v1.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 */

/* The config header is always included first. */


#include <stdint.h>
#include <stdio.h>
/* The config header is always included first. */
#ifndef CELLULAR_DO_NOT_USE_CUSTOM_CONFIG
#include "cellular_config.h"
#endif
#include "cellular_config_defaults.h"
#include "cellular_platform.h"
#include "cellular_common.h"
#include "cellular_common_portable.h"
#include "cellular_bg770.h"

/*-----------------------------------------------------------*/

#define ENABLE_MODULE_UE_RETRY_COUNT       ( 4U )
#define ENABLE_MODULE_UE_RETRY_TIMEOUT     ( 18000U )   /* observed at least 17113 ms */
#define BG770_NWSCANSEQ_CMD_MAX_SIZE       ( 30U ) /* Need at least the length of AT+QCFG="nwscanseq",020301,1\0. */

#define BG770_MAX_SUPPORTED_LTE_BAND       ( 66U )
#define BG770_MAX_SUPPORTED_NB_IOT_BAND    ( 66U )

#define GET_BYTE_COUNT(maxBitsNeeded)       ( ( maxBitsNeeded + 7U ) / 8U )             // round up to next byte
#define GET_HEX_STRING_COUNT(maxBitsNeeded) ( GET_BYTE_COUNT( maxBitsNeeded ) * 2U )    // each nibble takes a character

// NOTE: +2 is for hex value prefix "0x"
#define BG770_LTE_BAND_HEX_STRING_MAX_LENGTH    ( GET_HEX_STRING_COUNT( BG770_MAX_SUPPORTED_LTE_BAND ) + 2 )
#define BG770_NB_IOT_BAND_HEX_STRING_MAX_LENGTH ( GET_HEX_STRING_COUNT( BG770_MAX_SUPPORTED_NB_IOT_BAND ) + 2 )

typedef struct BG770FrequencyBands
{
    char lteBands_hexString[BG770_LTE_BAND_HEX_STRING_MAX_LENGTH + 1];
    char nbIotBands_hexString[BG770_NB_IOT_BAND_HEX_STRING_MAX_LENGTH + 1];
} BG770FrequencyBands_t;

typedef enum BG77URCIndicationOptionType
{
    BG770_URC_INDICATION_OPTION_MAIN = 0,  /**< URC output on main UART. */
    BG770_URC_INDICATION_OPTION_AUX,       /**< URC output on aux UART. */
    BG770_URC_INDICATION_OPTION_EMUX,      /**< URC output on emux UART. */
    BG770_URC_INDICATION_OPTION_UNKNOWN    /**< Unknown URC output. */
} BG770URCIndicationOptionType_t;

static const char *const URCCFG_URCPORT_MAIN = "\"main\"";
static const char *const URCCFG_URCPORT_AUX = "\"aux\"";
static const char *const URCCFG_URCPORT_EMUX = "\"emux\"";

typedef enum BG770FlowControlType
{
    BG770_FLOW_CONTROL_TYPE_NONE = 0,       /**< No flow control. */
    BG770_FLOW_CONTROL_TYPE_HARDWARE = 2,   /**< RTS (DCE by DTE) or CTS (DTE by DCE). */
    BG770_FLOW_CONTROL_TYPE_UNKNOWN,        /**< Unknown/unsupported flow control type. */
} BG770FlowControlType_t;

static const char *const NO_FLOW_CONTROL_STRING = "0";
static const char *const HARDWARE_FLOW_CONTROL_STRING = "2";

typedef struct BG770FlowControlState
{
    /**< NOTE: Data terminal equipment (DTE) is microcontroller. */
    /**< NOTE: Data communications equipment (DCE) is Quectel BG770A cellular modem. */
    BG770FlowControlType_t dceByDTE;        /**< RTS if hardware flow control. */
    BG770FlowControlType_t dteByDCE;        /**< CTS if hardware flow control. */
} BG770FlowControlState_t;

static const TickType_t APP_READY_MAX_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 10000U );
static const TickType_t POST_APP_READY_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 5000U );

static const TickType_t SHORT_DELAY_ticks = pdMS_TO_TICKS( 10U );

/*-----------------------------------------------------------*/

static CellularError_t sendAtCommandWithRetryTimeout( CellularContext_t * pContext,
                                                      const CellularAtReq_t * pAtReq );

static CellularError_t _GetLwM2MEnabled( CellularHandle_t cellularHandle,
                                         bool * pLwM2MEnabled );

static CellularError_t _GetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t * pURCIndicationOptionType );

static CellularError_t _SetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t urcIndicationOptionType );

static CellularError_t _GetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t * pFlowControlState );

static CellularError_t _SetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t flowControlState );

/*-----------------------------------------------------------*/

static cellularModuleContext_t cellularBg770Context = { 0 };

static bool configSkipPostHWFlowControlSetupIfChanged = false;
static CellularModuleFullInitSkippedResult_t fullInitSkippedResult = CELLULAR_FULL_INIT_SKIPPED_RESULT_ERROR;

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
const char * CellularSrcTokenErrorTable[] =
{ "ERROR", "BUSY", "NO CARRIER", "NO ANSWER", "NO DIALTONE", "ABORTED", "+CMS ERROR", "+CME ERROR", "SEND FAIL" };
/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
uint32_t CellularSrcTokenErrorTableSize = sizeof( CellularSrcTokenErrorTable ) / sizeof( char * );

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
const char * CellularSrcTokenSuccessTable[] =
{ "OK", "CONNECT", "SEND OK", ">" };
/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
uint32_t CellularSrcTokenSuccessTableSize = sizeof( CellularSrcTokenSuccessTable ) / sizeof( char * );

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
const char * CellularUrcTokenWoPrefixTable[] =
{ "APP RDY", "NORMAL POWER DOWN", "POWERED DOWN", "PSM POWER DOWN",  "RDY" };
/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
uint32_t CellularUrcTokenWoPrefixTableSize = sizeof( CellularUrcTokenWoPrefixTable ) / sizeof( char * );

/*-----------------------------------------------------------*/

static CellularError_t sendAtCommandWithRetryTimeoutParams( CellularContext_t * pContext,
                                                            const CellularAtReq_t * pAtReq,
                                                            uint32_t commandTimeoutMS,
                                                            uint32_t exponentialBackoffInterCommandBaseMS )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    uint8_t tryCount = 0;

    if( pAtReq == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        for( ; tryCount < ENABLE_MODULE_UE_RETRY_COUNT; tryCount++ )
        {
            if (tryCount > 0) {
                // increasing backoff
                vTaskDelay( pdMS_TO_TICKS( exponentialBackoffInterCommandBaseMS * (uint32_t)tryCount * tryCount ) );
            }

            pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, *pAtReq, commandTimeoutMS );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );

            if( cellularStatus == CELLULAR_SUCCESS )
            {
                break;
            }
        }
    }

    return cellularStatus;
}

static CellularError_t sendAtCommandWithRetryTimeout( CellularContext_t * pContext,
                                                      const CellularAtReq_t * pAtReq )
{
    return sendAtCommandWithRetryTimeoutParams( pContext, pAtReq, ENABLE_MODULE_UE_RETRY_TIMEOUT, 1000UL );
}

/*-----------------------------------------------------------*/

static bool appendRatList( char * pRatList,
                           CellularRat_t cellularRat )
{
    bool retValue = true;

    /* Configure RAT Searching Sequence to default radio access technology. */
    switch( cellularRat )
    {
        case CELLULAR_RAT_LTE:
        case CELLULAR_RAT_CATM1:
            strcat( pRatList, "02" );
            break;

        case CELLULAR_RAT_NBIOT:
            strcat( pRatList, "03" );
            break;

        case CELLULAR_RAT_GSM:
            strcat( pRatList, "01" );
            break;

        default:
            /* Configure RAT Searching Sequence to automatic. */
            retValue = false;
            break;
    }

    return retValue;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_ModuleInit( const CellularContext_t * pContext,
                                     void ** ppModuleContext )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    bool mutexCreateStatus = false;

    if( pContext == NULL )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ppModuleContext == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Initialize the module context. */
        ( void ) memset( &cellularBg770Context, 0, sizeof( cellularModuleContext_t ) );

        /* Create the mutex for DNS. */
        mutexCreateStatus = PlatformMutex_Create( &cellularBg770Context.dnsQueryMutex, false );

        if( mutexCreateStatus == false )
        {
            cellularStatus = CELLULAR_NO_MEMORY;
        }

        if (cellularStatus == CELLULAR_SUCCESS)
        {
            /* Create the queue for DNS. */
            cellularBg770Context.pktDnsQueue = xQueueCreate( 1, sizeof( cellularDnsQueryResult_t ) );

            if( cellularBg770Context.pktDnsQueue == NULL )
            {
                cellularStatus = CELLULAR_NO_MEMORY;
            }
        }

        if (cellularStatus == CELLULAR_SUCCESS)
        {
            cellularBg770Context.pInitEvent = (PlatformEventGroupHandle_t) PlatformEventGroup_Create();

            if (cellularBg770Context.pInitEvent == NULL) {
                cellularStatus = CELLULAR_NO_MEMORY;
            }
            else
            {
                ( void ) PlatformEventGroup_ClearBits( ( PlatformEventGroupHandle_t ) cellularBg770Context.pInitEvent,
                                                       ( ( PlatformEventGroup_EventBits ) INIT_EVT_MASK_ALL_EVENTS ) );
            }
        }
    }

    if (cellularStatus == CELLULAR_SUCCESS)
    {
        *ppModuleContext = ( void * ) &cellularBg770Context;
    }
    else
    {
        /* Clean up any created entities */

        if (mutexCreateStatus)
        {
            PlatformMutex_Destroy( &cellularBg770Context.dnsQueryMutex );
        }

        if( cellularBg770Context.pktDnsQueue != NULL )
        {
            /* Delete DNS queue. */
            vQueueDelete( cellularBg770Context.pktDnsQueue );
        }

        if (cellularBg770Context.pInitEvent != NULL)
        {
            ( void ) PlatformEventGroup_Delete( cellularBg770Context.pInitEvent );
            cellularBg770Context.pInitEvent = ( PlatformEventGroupHandle_t ) ( uintptr_t ) ( uintptr_t * ) NULL;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_ModuleCleanUp( const CellularContext_t * pContext )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    if( pContext == NULL )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else
    {
        /* Delete DNS queue. */
        vQueueDelete( cellularBg770Context.pktDnsQueue );
        cellularBg770Context.pktDnsQueue = NULL;

        /* Delete the mutex for DNS. */
        PlatformMutex_Destroy( &cellularBg770Context.dnsQueryMutex );

        ( void ) PlatformEventGroup_Delete( cellularBg770Context.pInitEvent );
        cellularBg770Context.pInitEvent = ( PlatformEventGroupHandle_t ) ( uintptr_t ) ( uintptr_t * ) NULL;
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_ModuleEnableUE( CellularContext_t * pContext )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqGetNoResult =
    {
        NULL,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0
    };
    // NOTE: commands need to use this configuration until echo is turned off
    CellularAtReq_t atReqGetWithResult =
    {
        NULL,
        CELLULAR_AT_MULTI_WO_PREFIX,
        NULL,
        NULL,
        NULL,
        0
    };
    char ratSelectCmd[ BG770_NWSCANSEQ_CMD_MAX_SIZE ] = "AT+QCFG=\"nwscanseq\",";
    bool retAppendRat = true;

    if( pContext != NULL )
    {
        cellularModuleContext_t * pModuleContext = NULL;
        CellularError_t result = _Cellular_GetModuleContext( pContext, (void **)&pModuleContext );
        if( ( result == CELLULAR_SUCCESS ) && ( pModuleContext != NULL ) )
        {
            /* Wait events for abort thread or rx data available. */
            PlatformEventGroup_EventBits uxBits = ( PlatformEventGroup_EventBits ) PlatformEventGroup_WaitBits(
                    ( PlatformEventGroupHandle_t ) pModuleContext->pInitEvent,
                    ( ( PlatformEventGroup_EventBits ) INIT_EVT_MASK_APP_RDY_RECEIVED ),
                    pdTRUE,
                    pdFALSE,
                    APP_READY_MAX_WAIT_PERIOD_ticks );

            if( ( uxBits & INIT_EVT_MASK_APP_RDY_RECEIVED ) != 0 )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: 'APP_RDY' URC received." ) );
            }
            else
            {
                LogWarn( ( "Cellular_ModuleEnableUE: Init event flag 'APP_RDY received' timeout (after waiting %lu ticks).", APP_READY_MAX_WAIT_PERIOD_ticks ) );
            }
        }
        else
        {
            LogError( ( "Cellular_ModuleEnableUE: Failed to wait on Init event flag 'APP_RDY received', waiting %lu ticks.", APP_READY_MAX_WAIT_PERIOD_ticks ) );
            vTaskDelay( APP_READY_MAX_WAIT_PERIOD_ticks );
        }

        vTaskDelay( POST_APP_READY_WAIT_PERIOD_ticks );

        /* Empty command, looking for 'OK' to indicate presence of module. */
        atReqGetWithResult.pAtCmd = "AT";
        cellularStatus = sendAtCommandWithRetryTimeoutParams( pContext, &atReqGetWithResult, 1000UL, 100UL );
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success, found modem.", atReqGetWithResult.pAtCmd ) );
        }
        else
        {
            LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetWithResult.pAtCmd ) );
        }

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            /* Disable echo - command is not automatically saved to user profile, don't need to perform read before write. */
            atReqGetWithResult.pAtCmd = "ATE0";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetWithResult );
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetWithResult.pAtCmd ) );
            }
            else
            {
                LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetWithResult.pAtCmd ) );
            }
        }

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            /* Disable DTR function - command is not automatically saved to user profile, don't need to perform read before write. */
            atReqGetNoResult.pAtCmd = "AT&D0";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }

#ifndef CELLULAR_CONFIG_DISABLE_FLOW_CONTROL
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            static const BG770FlowControlState_t desiredFlowControlState =
            {
                .dceByDTE = BG770_FLOW_CONTROL_TYPE_HARDWARE,
                .dteByDCE = BG770_FLOW_CONTROL_TYPE_HARDWARE
            };
            BG770FlowControlState_t flowControlState =
            {
                .dceByDTE = BG770_FLOW_CONTROL_TYPE_UNKNOWN,
                .dteByDCE = BG770_FLOW_CONTROL_TYPE_UNKNOWN
            };
            cellularStatus = _GetFlowControlState( pContext, &flowControlState);
            if( cellularStatus != CELLULAR_SUCCESS ||
                desiredFlowControlState.dceByDTE != flowControlState.dceByDTE ||
                desiredFlowControlState.dteByDCE != flowControlState.dteByDCE )
            {
                if( cellularStatus != CELLULAR_SUCCESS )
                {
                    LogError( ( "Cellular_ModuleEnableUE: Could not get hardware flow control state, assuming not already set." ) );
                }

                cellularStatus = _SetFlowControlState( pContext, desiredFlowControlState );
                if( cellularStatus == CELLULAR_SUCCESS )
                {
                    LogInfo( ( "Cellular_ModuleEnableUE: Set hardware flow control command success." ) );
                    if( configSkipPostHWFlowControlSetupIfChanged )
                    {
                        LogInfo( ( "Cellular_ModuleEnableUE: Full initialization skipped based on flow control state change." ) );
                        fullInitSkippedResult = CELLULAR_FULL_INIT_SKIPPED_RESULT_YES;
                        return cellularStatus;
                    }
                }
                else
                {
                    LogError( ( "Cellular_ModuleEnableUE: Set hardware flow control command failure." ) );
                }
            }
            else
            {
                LogInfo( ( "Cellular_ModuleEnableUE: Set hardware flow control command skipped, already set." ) );
                fullInitSkippedResult = CELLULAR_FULL_INIT_SKIPPED_RESULT_NO;
            }
        }
#endif

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

#if defined( CELLULAR_BG770_URC_PORT_EMUX ) || defined( BG770_URC_PORT_EMUX )
            static const BG770URCIndicationOptionType_t desiredURCIndicationOptionType = BG770_URC_INDICATION_OPTION_EMUX;
#else
            static const BG770URCIndicationOptionType_t desiredURCIndicationOptionType = BG770_URC_INDICATION_OPTION_MAIN;
#endif

            BG770URCIndicationOptionType_t urcIndicationOptionType = BG770_URC_INDICATION_OPTION_UNKNOWN;
            CellularError_t getURCIndicationOptionStatus = _GetURCIndicationOption(pContext, &urcIndicationOptionType );
            if( getURCIndicationOptionStatus != CELLULAR_SUCCESS ||
                urcIndicationOptionType == BG770_URC_INDICATION_OPTION_UNKNOWN ||
                urcIndicationOptionType != desiredURCIndicationOptionType )
            {
                /* Setting URC output port. */
                cellularStatus = _SetURCIndicationOption( pContext, desiredURCIndicationOptionType );
                if( cellularStatus == CELLULAR_SUCCESS )
                {
                    LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
                }
                else
                {
                    LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
                }
            }
            else
            {
                LogInfo( ( "Cellular_ModuleEnableUE: 'AT+QURCCFG=\"urcport\"' command skipped, already set.", atReqGetNoResult.pAtCmd ) );
            }
        }

//#define CELLULAR_QUECTEL_ENABLE_DEBUG_UART
//#define CELLULAR_QUECTEL_DISABLE_DEBUG_UART
#if defined(CELLULAR_QUECTEL_ENABLE_DEBUG_UART) || defined(CELLULAR_QUECTEL_DISABLE_DEBUG_UART)
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Setting debug output enable. */
#ifdef CELLULAR_QUECTEL_ENABLE_DEBUG_UART
            atReqGetNoResult.pAtCmd = "AT+QCFGEXT=\"debug\",1";
#else
            atReqGetNoResult.pAtCmd = "AT+QCFGEXT=\"debug\",0";
#endif
            CellularError_t debugResult = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( debugResult == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogWarn( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Debug output enable skipped due to error." ) );
        }
#endif

//#define CELLULAR_QUECTEL_ENABLE_USB
//#define CELLULAR_QUECTEL_DISABLE_USB
#if defined(CELLULAR_QUECTEL_ENABLE_USB) || defined(CELLULAR_QUECTEL_DISABLE_USB)
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Setting debug output enable. */
#ifdef CELLULAR_QUECTEL_ENABLE_USB
            atReqGetNoResult.pAtCmd = "AT+QCFG=\"usb\",1";
#else
            atReqGetNoResult.pAtCmd = "AT+QCFG=\"usb\",0";
#endif
            CellularError_t debugResult = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( debugResult == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogWarn( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: USB enable skipped due to error." ) );
        }
#endif

        // TODO (MV): Implement read before write
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Configure Network Category to be Searched under LTE RAT to eMTC only. */
            /* FUTURE: From Quectel Support, this command "will force rescan bands and take more time to register.
             *         Actually, it is not necessary to send at each power cycle."; therefore,
                       should check if already set correctly and only send if changed. */
            atReqGetNoResult.pAtCmd = "AT+QCFG=\"iotopmode\",0,1";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: eMTC (LTE-M) only network category skipped due to error." ) );
        }

        // TODO (MV): Implement read before write
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            retAppendRat = appendRatList( ratSelectCmd, CELLULAR_CONFIG_DEFAULT_RAT );
            configASSERT( retAppendRat == true );

            #ifdef CELLULAR_CONFIG_DEFAULT_RAT_2
                retAppendRat = appendRatList( ratSelectCmd, CELLULAR_CONFIG_DEFAULT_RAT_2 );
                configASSERT( retAppendRat == true );
            #endif

            #ifdef CELLULAR_CONFIG_DEFAULT_RAT_3
                retAppendRat = appendRatList( ratSelectCmd, CELLULAR_CONFIG_DEFAULT_RAT_3 );
                configASSERT( retAppendRat == true );
            #endif

            strcat( ratSelectCmd, ",1" ); /* Take effect immediately. */
            atReqGetNoResult.pAtCmd = ratSelectCmd;
            /* FUTURE: From Quectel Support, this command "will force rescan bands and take more time to register.
             *         Actually, it is not necessary to send at each power cycle."; therefore,
                       should check if already set correctly and only send if changed. */
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Network scan RAT list skipped due to error." ) );
        }

        // TODO (MV): Implement read before write
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* SIM enabled, RF off. Need to set additional settings before modem tries to connect. */
            /* NOTE: Command may fail the first time a new different SIM card is inserted (eg. Soracom -> Verizon or Verizon -> Soracom),
             *       therefore, it is important that Cellular_Init() be tried more than once. */
            atReqGetNoResult.pAtCmd = "AT+CFUN=4";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
            }
            else
            {
                LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Skipped Set RF off / SIM enabled due to error." ) );
        }

        // TODO (MV): Implement read before write
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Disable LwM2M (automatically turned on with Verizon SIM, LwM2M can change APN and mess with normal DNS lookups) */
            /* MISRA Ref 21.6.1 [Use of snprintf] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
            /* coverity[misra_c_2012_rule_21_6_violation]. */
            ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+QCFG=\"lwm2m\",0" );

            atReqGetNoResult.pAtCmd = cmdBuf;

            bool isLwM2MEnabled = false;
            CellularError_t getLwM2MEnableStatus = _GetLwM2MEnabled(pContext, &isLwM2MEnabled );
            if( getLwM2MEnableStatus != CELLULAR_SUCCESS || isLwM2MEnabled )
            {
                vTaskDelay( SHORT_DELAY_ticks );

                cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
                if( cellularStatus == CELLULAR_SUCCESS )
                {
                    LogInfo( ( "Cellular_ModuleEnableUE: '%s' command success.", atReqGetNoResult.pAtCmd ) );
                }
                else
                {
                    LogError( ( "Cellular_ModuleEnableUE: '%s' command failed.", atReqGetNoResult.pAtCmd ) );
                }
            }
            else
            {
                LogInfo( ( "Cellular_ModuleEnableUE: '%s' command skipped, already set.", atReqGetNoResult.pAtCmd ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Disable LwM2M skipped due to error." ) );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_ModuleEnableUrc( CellularContext_t * pContext )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularAtReq_t atReqGetNoResult =
    {
        NULL,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0
    };

    if( configSkipPostHWFlowControlSetupIfChanged && fullInitSkippedResult == CELLULAR_FULL_INIT_SKIPPED_RESULT_YES )
    {
        LogInfo( ( "Cellular_ModuleEnableUrc: Commands skipped, re-init required for flow control change." ) );
        return cellularStatus;
    }

    /* Set numeric operator format. */
    atReqGetNoResult.pAtCmd = "AT+COPS=3,2";
    ( void ) _Cellular_AtcmdRequestWithCallback( pContext, atReqGetNoResult );

    /* Enable network registration and location information unsolicited result code:
        +CREG: <stat>[,[<lac>],[<ci>],[<AcT>]]
     */
    atReqGetNoResult.pAtCmd = "AT+CREG=2";
    ( void ) _Cellular_AtcmdRequestWithCallback( pContext, atReqGetNoResult );

    /* Enable LTE network registration and location information unsolicited result code:
        +CEREG: <stat>[,[<tac>],[<ci>],[<AcT>]]
     */
    atReqGetNoResult.pAtCmd = "AT+CEREG=2";
    ( void ) _Cellular_AtcmdRequestWithCallback( pContext, atReqGetNoResult );

    /* Enable time zone change event reporting by unsolicited result code +CTZV: <tz> */
    atReqGetNoResult.pAtCmd = "AT+CTZR=1";
    ( void ) _Cellular_AtcmdRequestWithCallback( pContext, atReqGetNoResult );

    /* Enable PSM URC reporting by unsolicited result code +QPSMTIMER: <TAU_timer>,<T3324_timer> */
    atReqGetNoResult.pAtCmd = "AT+QCFG=\"psm/urc\",1";
    ( void ) _Cellular_AtcmdRequestWithCallback( pContext, atReqGetNoResult );

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static bool _parseLwM2MEnable( char * pQcfgLwM2MPayload,
                               bool * pIsLwM2MEnabled )
{
    char * pToken = NULL, * pTmpQcfgLwM2MPayload = pQcfgLwM2MPayload;
    bool parseStatus = true;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    int32_t tempValue = 0;

    if( ( pIsLwM2MEnabled == NULL ) || ( pQcfgLwM2MPayload == NULL ) )
    {
        LogError( ( "_parseLwM2MEnable: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQcfgLwM2MPayload, &pToken ) != CELLULAR_AT_SUCCESS ||
            strcmp( pToken, "\"lwm2m\"" ) != 0 )
        {
            LogError( ( "_parseLwM2MEnable: Error, missing \"lwm2m\"" ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQcfgLwM2MPayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );
            if( atCoreStatus == CELLULAR_AT_SUCCESS)
            {
                if( ( tempValue >= 0 ) && ( tempValue <= ( int32_t ) 1 ) )
                {
                    *pIsLwM2MEnabled = ( tempValue == 1 );
                }
                else
                {
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }

            if( atCoreStatus != CELLULAR_AT_SUCCESS )
            {
                LogError( ( "_parseLwM2MEnable: Error in processing enable. Token %s", pToken ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseLwM2MEnable: enable not present" ) );
            *pIsLwM2MEnabled = false;
            parseStatus = false;
        }
    }
    else
    {
        *pIsLwM2MEnabled = false;
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetLwM2MEnable( CellularContext_t * pContext,
                                                    const CellularATCommandResponse_t * pAtResp,
                                                    void * pData,
                                                    uint16_t dataLen )
{
    char * pInputLine = NULL;
    bool * pIsLwM2MEnabled = ( bool * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pIsLwM2MEnabled == NULL ) || ( dataLen != sizeof( bool ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetLwM2MEnabled: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus != CELLULAR_AT_SUCCESS )
        {
            pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
        }
    }

    if( pktStatus == CELLULAR_PKT_STATUS_OK )
    {
        parseStatus = _parseLwM2MEnable( pInputLine, pIsLwM2MEnabled );

        if( parseStatus != true )
        {
            *pIsLwM2MEnabled = false;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetLwM2MEnabled( CellularHandle_t cellularHandle,
                                         bool * pIsLwM2MEnabled )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetLwM2MEnable =
    {
        "AT+QCFG=\"lwm2m\"",
        CELLULAR_AT_WITH_PREFIX,
        "+QCFG",
        _RecvFuncGetLwM2MEnable,
        pIsLwM2MEnabled,
        sizeof( bool ),
    };

    if( pIsLwM2MEnabled == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetLwM2MEnable );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetLwM2MEnabled: couldn't retrieve L2M2M enable" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static BG770URCIndicationOptionType_t _getURCIndictionOptionType( const char * pUrcIndicationOptionString )
{
    if( strcmp( pUrcIndicationOptionString, URCCFG_URCPORT_MAIN ) == 0 )
    {
        return BG770_URC_INDICATION_OPTION_MAIN;
    }
    else if ( strcmp( pUrcIndicationOptionString, URCCFG_URCPORT_AUX ) == 0 )
    {
        return BG770_URC_INDICATION_OPTION_AUX;
    }
    else if ( strcmp( pUrcIndicationOptionString, URCCFG_URCPORT_EMUX ) == 0 )
    {
        return BG770_URC_INDICATION_OPTION_EMUX;
    }
    else
    {
        return BG770_URC_INDICATION_OPTION_UNKNOWN;
    }
}

static const char * _getURCIndictionOptionString( const BG770URCIndicationOptionType_t urcIndicationOptionType )
{
    switch( urcIndicationOptionType )
    {
        case BG770_URC_INDICATION_OPTION_MAIN:
            return URCCFG_URCPORT_MAIN;

        case BG770_URC_INDICATION_OPTION_AUX:
            return URCCFG_URCPORT_AUX;

        case BG770_URC_INDICATION_OPTION_EMUX:
            return URCCFG_URCPORT_EMUX;

        default:
            return "unknown";
    }
}

static bool _parseURCIndicationOption( char * pQurccfgUrcportPayload,
                                       BG770URCIndicationOptionType_t * pURCIndicationOption )
{
    char * pToken = NULL, * pTmpQurccfgUrcportPayload = pQurccfgUrcportPayload;
    bool parseStatus = true;

    if( ( pURCIndicationOption == NULL ) || ( pQurccfgUrcportPayload == NULL ) )
    {
        LogError( ( "_parseURCIndicationOption: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQurccfgUrcportPayload, &pToken ) != CELLULAR_AT_SUCCESS ||
            strcmp( pToken, "\"urcport\"" ) != 0 )
        {
            LogError( ( "_parseURCIndicationOption: Error, missing \"urcport\"" ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQurccfgUrcportPayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            *pURCIndicationOption = _getURCIndictionOptionType( pToken );
            if( *pURCIndicationOption == BG770_URC_INDICATION_OPTION_UNKNOWN ) {
                LogError( ( "_parseURCIndicationOption: URC indication option string not valid" ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseURCIndicationOption: URC indication option string not present" ) );
            *pURCIndicationOption = BG770_URC_INDICATION_OPTION_UNKNOWN;
            parseStatus = false;
        }
    }
    else
    {
        *pURCIndicationOption = BG770_URC_INDICATION_OPTION_UNKNOWN;
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetURCIndicationOption( CellularContext_t * pContext,
                                                            const CellularATCommandResponse_t * pAtResp,
                                                            void * pData,
                                                            uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770URCIndicationOptionType_t * pURCIndicationOption = ( BG770URCIndicationOptionType_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pURCIndicationOption == NULL ) || ( dataLen != sizeof( BG770URCIndicationOptionType_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetURCIndicationOption: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus != CELLULAR_AT_SUCCESS )
        {
            pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
        }
    }

    if( pktStatus == CELLULAR_PKT_STATUS_OK )
    {
        parseStatus = _parseURCIndicationOption( pInputLine, pURCIndicationOption );

        if( parseStatus != true )
        {
            *pURCIndicationOption = BG770_URC_INDICATION_OPTION_UNKNOWN;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t * pURCIndicationOptionType )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetURCIndicationOption =
    {
        "AT+QURCCFG=\"urcport\"",
        CELLULAR_AT_WITH_PREFIX,
        "+QURCCFG",
        _RecvFuncGetURCIndicationOption,
        pURCIndicationOptionType,
        sizeof( BG770URCIndicationOptionType_t ),
    };

    if( pURCIndicationOptionType == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetURCIndicationOption );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetURCIndicationOption: couldn't retrieve URC indication option" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t _SetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t urcIndicationOptionType )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetURCIndicationOption =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( urcIndicationOptionType != BG770_URC_INDICATION_OPTION_MAIN &&
        urcIndicationOptionType != BG770_URC_INDICATION_OPTION_AUX &&
        urcIndicationOptionType != BG770_URC_INDICATION_OPTION_EMUX )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+QURCCFG=\"urcport\",%s",
                           _getURCIndictionOptionString( urcIndicationOptionType ) );

        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetURCIndicationOption );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_SetURCIndicationOption: couldn't set URC indication option" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

CellularError_t CellularModule_SkipInitializationPostHWFlowControlSetupIfChanged(
        const bool skipPostHWFlowControlSetupIfChanged )
{
    configSkipPostHWFlowControlSetupIfChanged = skipPostHWFlowControlSetupIfChanged;
    fullInitSkippedResult = CELLULAR_FULL_INIT_SKIPPED_RESULT_ERROR;    /**< Assume error until explicit yes/no. */
    return CELLULAR_SUCCESS;
}

/*-----------------------------------------------------------*/

CellularError_t CellularModule_TryGetDidSkipInitializationPostHWFlowControlSetup(
        CellularModuleFullInitSkippedResult_t *const pSkippedResult)
{
    if( pSkippedResult == NULL )
    {
        return CELLULAR_BAD_PARAMETER;
    }

    *pSkippedResult = fullInitSkippedResult;
    return CELLULAR_SUCCESS;
}

/*-----------------------------------------------------------*/

/**< NOTE: pFlowControlTypeString is expected to contain no whitespace. */
static BG770FlowControlType_t _getFlowControlType( const char * pFlowControlTypeString )
{
    if( strcmp( pFlowControlTypeString, NO_FLOW_CONTROL_STRING ) == 0 )
    {
        return BG770_FLOW_CONTROL_TYPE_NONE;
    }
    else if ( strcmp( pFlowControlTypeString, HARDWARE_FLOW_CONTROL_STRING ) == 0 )
    {
        return BG770_FLOW_CONTROL_TYPE_HARDWARE;
    }
    else
    {
        return BG770_FLOW_CONTROL_TYPE_UNKNOWN;
    }
}

static const char * _getFlowControlTypeString( const BG770FlowControlType_t flowControlType )
{
    switch( flowControlType )
    {
        case BG770_FLOW_CONTROL_TYPE_NONE:
            return NO_FLOW_CONTROL_STRING;

        case BG770_FLOW_CONTROL_TYPE_HARDWARE:
            return HARDWARE_FLOW_CONTROL_STRING;

        default:
            LogError( ( "_getFlowControlTypeString: Invalid BG770FlowControlType_t: %d", flowControlType ) );
            /**< Intentional fall-through */
        case BG770_FLOW_CONTROL_TYPE_UNKNOWN:
            return "unknown";
    }
}

static bool _parseFlowControlState( char * pQflowControlStatePayload,
                                    BG770FlowControlState_t *const pFlowControlState )
{
    char * pToken = NULL, * pTmpQflowControlStatePayload = pQflowControlStatePayload;
    bool parseStatus = true;

    if( ( pFlowControlState == NULL ) || ( pQflowControlStatePayload == NULL ) )
    {
        LogError( ( "_parseFlowControlType: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQflowControlStatePayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            pFlowControlState->dceByDTE = _getFlowControlType( pToken );
            if( pFlowControlState->dceByDTE == BG770_FLOW_CONTROL_TYPE_UNKNOWN ) {
                LogError( ( "_parseFlowControlType: DCE-by-DTE flow control type invalid, '%s'", pToken ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseFlowControlType: DCE-by-DTE flow control type string not present" ) );
            pFlowControlState->dceByDTE = BG770_FLOW_CONTROL_TYPE_UNKNOWN;
            pFlowControlState->dteByDCE = BG770_FLOW_CONTROL_TYPE_UNKNOWN;
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQflowControlStatePayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            pFlowControlState->dteByDCE = _getFlowControlType( pToken );
            if( pFlowControlState->dteByDCE == BG770_FLOW_CONTROL_TYPE_UNKNOWN ) {
                LogError( ( "_parseFlowControlType: DTE-by-DCE flow control type invalid, '%s'", pToken ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseFlowControlType: DTE-by-DCE flow control type string not present" ) );
            pFlowControlState->dteByDCE = BG770_FLOW_CONTROL_TYPE_UNKNOWN;
            parseStatus = false;
        }
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetFlowControlState( CellularContext_t * pContext,
                                                         const CellularATCommandResponse_t * pAtResp,
                                                         void * pData,
                                                         uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770FlowControlState_t * pFlowControlState = ( BG770FlowControlState_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pFlowControlState == NULL ) || ( dataLen != sizeof( BG770FlowControlState_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetFlowControlState: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus != CELLULAR_AT_SUCCESS )
        {
            pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
        }
    }

    if( pktStatus == CELLULAR_PKT_STATUS_OK )
    {
        parseStatus = _parseFlowControlState( pInputLine, pFlowControlState );

        if( parseStatus != true )
        {
            pFlowControlState->dceByDTE = BG770_FLOW_CONTROL_TYPE_UNKNOWN;
            pFlowControlState->dteByDCE = BG770_FLOW_CONTROL_TYPE_UNKNOWN;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t *const pFlowControlState )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetFlowControlState =
    {
        "AT+IFC?",
        CELLULAR_AT_WITH_PREFIX,
        "+IFC",
        _RecvFuncGetFlowControlState,
        pFlowControlState,
        sizeof( BG770FlowControlState_t ),
    };

    if( pFlowControlState == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetFlowControlState );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetFlowControlState: couldn't retrieve flow control state" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t _SetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t flowControlState )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetFlowControlState =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( ( flowControlState.dceByDTE != BG770_FLOW_CONTROL_TYPE_NONE &&
          flowControlState.dceByDTE != BG770_FLOW_CONTROL_TYPE_HARDWARE ) ||
        ( flowControlState.dteByDCE != BG770_FLOW_CONTROL_TYPE_NONE &&
          flowControlState.dteByDCE != BG770_FLOW_CONTROL_TYPE_HARDWARE ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+IFC=%s,%s",
                           _getFlowControlTypeString( flowControlState.dceByDTE ),
                           _getFlowControlTypeString( flowControlState.dteByDCE ) );

        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetFlowControlState );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_SetFlowControlState: couldn't set flow control state" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}
