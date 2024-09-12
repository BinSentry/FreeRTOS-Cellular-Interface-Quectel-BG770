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
#define ENABLE_MODULE_UE_RETRY_TIMEOUT_MS   ( 18000U )   /* observed at least 17113 ms */
#define ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS    ( 1000UL )
#define BG770_NWSCANSEQ_CMD_MAX_SIZE       ( 30U ) /* Need at least the length of AT+QCFG="nwscanseq",020301,1\0. */

#define BG770_MAX_SUPPORTED_LTE_BAND       ( 66U )
#define BG770_MAX_SUPPORTED_NB_IOT_BAND    ( 66U )

#define GET_BYTE_COUNT(maxBitsNeeded)       ( ( maxBitsNeeded + 7U ) / 8U )             // round up to next byte
#define GET_HEX_STRING_COUNT(maxBitsNeeded) ( GET_BYTE_COUNT( maxBitsNeeded ) * 2U )    // each nibble takes a character

// NOTE: +2 is for hex value prefix "0x"
#define BG770_LTE_BAND_HEX_STRING_MAX_LENGTH    ( GET_HEX_STRING_COUNT( BG770_MAX_SUPPORTED_LTE_BAND ) + 2 )
#define BG770_NB_IOT_BAND_HEX_STRING_MAX_LENGTH ( GET_HEX_STRING_COUNT( BG770_MAX_SUPPORTED_NB_IOT_BAND ) + 2 )

typedef CellularError_t (*FuncRetryableSet_t)( CellularHandle_t cellularHandle, uint32_t commandTimeoutMS );

typedef struct BG770FrequencyBands
{
    char lteBands_hexString[BG770_LTE_BAND_HEX_STRING_MAX_LENGTH + 1];
    char nbIotBands_hexString[BG770_NB_IOT_BAND_HEX_STRING_MAX_LENGTH + 1];
} BG770FrequencyBands_t;

typedef enum BG770URCIndicationOptionType
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

typedef enum BG770UEFunctionalityLevel
{
    BG770_UE_FUNCTIONALITY_LEVEL_MINIMUM = 0,     /**< RF front-end and SIM card disabled */
    BG770_UE_FUNCTIONALITY_LEVEL_FULL = 1,        /**< RF front-end and SIM card enabled */
    BG770_UE_FUNCTIONALITY_LEVEL_SIM_ONLY = 4,    /**< RF front-end disabled and SIM card enabled */
    BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN,         /**< Unknown/unsupported functionality type. */
} BG770UEFunctionalityLevel_t;

static const char *const UE_FUNC_LEVEL_MINIMUM_STRING = "0";
static const char *const UE_FUNC_LEVEL_FULL_STRING = "1";
static const char *const UE_FUNC_LEVEL_SIM_ONLY_STRING = "4";

/* SIM enabled, RF off. Need to set additional settings before modem tries to connect. */
static const BG770UEFunctionalityLevel_t DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL = BG770_UE_FUNCTIONALITY_LEVEL_SIM_ONLY;

typedef enum BG770NetworkCategorySearchMode
{
    BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC = 0,            /**< eMTC/LTE-M. */
    BG770_NETWORK_CATEGORY_SEARCH_MODE_NB_IoT = 1,          /**< NB-Iot. */
    BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC_AND_NB_IoT = 2, /**< eMTC/LTE-M and NB-IoT. */
    BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN              /**< Unknown network category search mode. */
} BG770NetworkCategorySearchMode_t;

static const char *const NET_CAT_SEARCH_MODE_eMTC_STRING = "0";
static const char *const NET_CAT_SEARCH_MODE_NB_IoT_STRING = "1";
static const char *const NET_CAT_SEARCH_MODE_eMTC_AND_NB_IoT_STRING = "2";

/* Configure Network Category to be Searched under LTE RAT to eMTC only. */
static const BG770NetworkCategorySearchMode_t DESIRED_NETWORK_CATEGORY_SEARCH_MODE = BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC;

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
                                             BG770FlowControlState_t * pFlowControlState,
                                             uint32_t commandTimeoutMS );

static CellularError_t _GetFlowControlStateWithRetryTimeout( CellularHandle_t cellularHandle,
                                                             BG770FlowControlState_t * pFlowControlState,
                                                             uint32_t commandTimeoutMS,
                                                             uint32_t exponentialBackoffInterCommandBaseMS );

static CellularError_t _SetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t flowControlState );

static CellularError_t _GetUEFunctionalityLevel( CellularHandle_t cellularHandle,
                                                 BG770UEFunctionalityLevel_t * pUEFunctionalityLevel,
                                                 uint32_t commandTimeoutMS );

static CellularError_t _GetUEFunctionalityLevelWithRetryTimeout( CellularHandle_t cellularHandle,
                                                                 BG770UEFunctionalityLevel_t * pUEFunctionalityLevel,
                                                                 uint32_t commandTimeoutMS,
                                                                 uint32_t exponentialBackoffInterCommandBaseMS );

static CellularError_t _SetUEFunctionalityLevel( CellularHandle_t cellularHandle,
                                                 BG770UEFunctionalityLevel_t ueFunctionalityLevel,
                                                 uint32_t commandTimeoutMS );

static CellularError_t _SetDesiredUEFunctionalityLevel( CellularHandle_t cellularHandle, uint32_t commandTimeoutMS );

static CellularError_t _GetNetworkCategorySearchMode( CellularHandle_t cellularHandle,
                                                      BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode,
                                                      uint32_t commandTimeoutMS );

static CellularError_t _GetNetworkCategorySearchModeWithRetryTimeout( CellularHandle_t cellularHandle,
                                                                      BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode,
                                                                      uint32_t commandTimeoutMS,
                                                                      uint32_t exponentialBackoffInterCommandBaseMS );

static CellularError_t _SetNetworkCategorySearchMode( CellularHandle_t cellularHandle,
                                                      BG770NetworkCategorySearchMode_t networkCategorySearchMode,
                                                      bool applyImmediately,
                                                      uint32_t commandTimeoutMS );

static CellularError_t _SetDesiredNetworkCategorySearchMode( CellularHandle_t cellularHandle, uint32_t commandTimeoutMS );

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
    return sendAtCommandWithRetryTimeoutParams( pContext, pAtReq, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                                                ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
}

static CellularError_t setRetryableSettingWithTimeoutParams( FuncRetryableSet_t retryableSetFunction,
                                                             CellularHandle_t cellularHandle,
                                                             uint32_t commandTimeoutMS,
                                                             uint32_t exponentialBackoffInterCommandBaseMS )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint8_t tryCount = 0;

    if( cellularHandle == NULL || retryableSetFunction == NULL )
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

            cellularStatus = ( * retryableSetFunction )( cellularHandle, commandTimeoutMS );

            if( cellularStatus == CELLULAR_SUCCESS )
            {
                break;
            }
        }
    }

    return cellularStatus;
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
            cellularStatus = _GetFlowControlStateWithRetryTimeout(
                    pContext, &flowControlState, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                    ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
            if( cellularStatus != CELLULAR_SUCCESS ||
                desiredFlowControlState.dceByDTE != flowControlState.dceByDTE ||
                desiredFlowControlState.dteByDCE != flowControlState.dteByDCE )
            {
                if( cellularStatus != CELLULAR_SUCCESS )
                {
                    LogError( ( "Cellular_ModuleEnableUE: Could not get hardware flow control state, assuming not already set." ) );
                }

                /**< No retry, even if miss 'OK' the flow control may have changed */
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
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: hardware flow control skipped due to error." ) );
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
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: URC indication port skipped due to error." ) );
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

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            BG770NetworkCategorySearchMode_t networkCategorySearchMode = BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN;
            cellularStatus = _GetNetworkCategorySearchModeWithRetryTimeout(
                    pContext, &networkCategorySearchMode, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                    ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
            if( cellularStatus != CELLULAR_SUCCESS ||
                DESIRED_NETWORK_CATEGORY_SEARCH_MODE != networkCategorySearchMode )
            {
                if( cellularStatus != CELLULAR_SUCCESS )
                {
                    LogError( ( "Cellular_ModuleEnableUE: Could not get network category search mode, assuming not already set." ) );
                }

                /* NOTE: From Quectel Support, this command "will force rescan bands and take more time to register.
                 *       Actually, it is not necessary to send at each power cycle."; therefore,
                 *       should check if already set correctly and only send if changed. */
                cellularStatus = setRetryableSettingWithTimeoutParams(
                        _SetDesiredNetworkCategorySearchMode, pContext, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                        ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
                if( cellularStatus == CELLULAR_SUCCESS )
                {
                    LogInfo( ( "Cellular_ModuleEnableUE: Set network category search mode (%d) command success.", DESIRED_NETWORK_CATEGORY_SEARCH_MODE ) );
                }
                else
                {
                    LogError( ( "Cellular_ModuleEnableUE: Set network category search mode (%d) command failure.", DESIRED_NETWORK_CATEGORY_SEARCH_MODE ) );
                }
            }
            else
            {
                LogInfo( ( "Cellular_ModuleEnableUE: Set network category search mode (%d) command skipped, already set.", DESIRED_NETWORK_CATEGORY_SEARCH_MODE ) );
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

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* NOTE: Command may fail the first time a new different SIM card is inserted (eg. Soracom -> Verizon or Verizon -> Soracom),
             *       therefore, it is important that Cellular_Init() be tried more than once. */
            BG770UEFunctionalityLevel_t ueFunctionalityLevel = BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN;
            cellularStatus = _GetUEFunctionalityLevelWithRetryTimeout(
                    pContext, &ueFunctionalityLevel, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                    ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
            if( cellularStatus != CELLULAR_SUCCESS ||
                DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL != ueFunctionalityLevel )
            {
                if( cellularStatus != CELLULAR_SUCCESS )
                {
                    LogError( ( "Cellular_ModuleEnableUE: Could not get UE functionality level, assuming not already set." ) );
                }

                cellularStatus = setRetryableSettingWithTimeoutParams(
                        _SetDesiredUEFunctionalityLevel, pContext, ENABLE_MODULE_UE_RETRY_TIMEOUT_MS,
                        ENABLE_MODULE_UE_RETRY_EXP_BACKOFF_INTER_COMMAND_BASE_MS );
                if( cellularStatus == CELLULAR_SUCCESS )
                {
                    LogInfo( ( "Cellular_ModuleEnableUE: Set UE functionality level (%d) command success.", DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL ) );
                }
                else
                {
                    LogError( ( "Cellular_ModuleEnableUE: Set UE functionality level (%d) command failure.", DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL ) );
                }
            }
            else
            {
                LogInfo( ( "Cellular_ModuleEnableUE: Set UE functionality level (%d) command skipped, already set.", DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Skipped Set RF off / SIM enabled due to error." ) );
        }

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
        if( pURCIndicationOption != NULL )
        {
            *pURCIndicationOption = BG770_URC_INDICATION_OPTION_UNKNOWN;
        }
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

static CellularError_t _GetFlowControlStateWithRetryTimeout( CellularHandle_t cellularHandle,
                                                             BG770FlowControlState_t *const pFlowControlState,
                                                             uint32_t commandTimeoutMS,
                                                             uint32_t exponentialBackoffInterCommandBaseMS )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint8_t tryCount = 0;

    if( cellularHandle == NULL || pFlowControlState == NULL )
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

            cellularStatus = _GetFlowControlState( cellularHandle, pFlowControlState, commandTimeoutMS );

            if( cellularStatus == CELLULAR_SUCCESS )
            {
                break;
            }
        }
    }

    return cellularStatus;
}

static CellularError_t _GetFlowControlState( CellularHandle_t cellularHandle,
                                             BG770FlowControlState_t *const pFlowControlState,
                                             uint32_t commandTimeoutMS )
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
        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqGetFlowControlState, commandTimeoutMS );

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

/*-----------------------------------------------------------*/

/**< NOTE: pFunctionalityLevelString is expected to contain no whitespace. */
static BG770UEFunctionalityLevel_t _getUEFunctionalityLevel( const char * pFunctionalityLevelString )
{
    if( strcmp( pFunctionalityLevelString, UE_FUNC_LEVEL_MINIMUM_STRING ) == 0 )
    {
        return BG770_UE_FUNCTIONALITY_LEVEL_MINIMUM;
    }
    else if ( strcmp( pFunctionalityLevelString, UE_FUNC_LEVEL_FULL_STRING ) == 0 )
    {
        return BG770_UE_FUNCTIONALITY_LEVEL_FULL;
    }
    else if ( strcmp( pFunctionalityLevelString, UE_FUNC_LEVEL_SIM_ONLY_STRING ) == 0 )
    {
        return BG770_UE_FUNCTIONALITY_LEVEL_SIM_ONLY;
    }
    else
    {
        return BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN;
    }
}

static const char * _getUEFunctionalityLevelString( const BG770UEFunctionalityLevel_t ueFunctionalityLevel )
{
    switch( ueFunctionalityLevel )
    {
        case BG770_UE_FUNCTIONALITY_LEVEL_MINIMUM:
            return UE_FUNC_LEVEL_MINIMUM_STRING;

        case BG770_UE_FUNCTIONALITY_LEVEL_FULL:
            return UE_FUNC_LEVEL_FULL_STRING;

        case BG770_UE_FUNCTIONALITY_LEVEL_SIM_ONLY:
            return UE_FUNC_LEVEL_SIM_ONLY_STRING;

        default:
            LogError( ( "_getUEFunctionalityLevelString: Invalid BG770UEFunctionalityLevel_t: %d", ueFunctionalityLevel ) );
            /**< Intentional fall-through */
        case BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN:
            return "unknown";
    }
}

static bool _parseUEFunctionalityLevel( char * pQUEFunctionalityLevelPayload,
                                        BG770UEFunctionalityLevel_t *const pUEFunctionalityLevel )
{
    char * pToken = NULL, * pTmpQUEFunctionalityLevelPayload = pQUEFunctionalityLevelPayload;
    bool parseStatus = true;

    if( ( pUEFunctionalityLevel == NULL ) || ( pQUEFunctionalityLevelPayload == NULL ) )
    {
        LogError( ( "_parseUEFunctionalityLevel: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQUEFunctionalityLevelPayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            *pUEFunctionalityLevel = _getUEFunctionalityLevel( pToken );
            if( *pUEFunctionalityLevel == BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN ) {
                LogError( ( "_parseUEFunctionalityLevel: UE functionality level invalid, '%s'", pToken ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseUEFunctionalityLevel: UE functionality level string not present" ) );
            *pUEFunctionalityLevel = BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN;
            parseStatus = false;
        }
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetUEFunctionalityLevel( CellularContext_t * pContext,
                                                             const CellularATCommandResponse_t * pAtResp,
                                                             void * pData,
                                                             uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770UEFunctionalityLevel_t * pUEFunctionalityLevel = ( BG770UEFunctionalityLevel_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pUEFunctionalityLevel == NULL ) || ( dataLen != sizeof( BG770UEFunctionalityLevel_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetUEFunctionalityLevel: Input Line passed is NULL" ) );
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
        parseStatus = _parseUEFunctionalityLevel( pInputLine, pUEFunctionalityLevel );

        if( parseStatus != true )
        {
            *pUEFunctionalityLevel = BG770_UE_FUNCTIONALITY_LEVEL_UNKNOWN;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetUEFunctionalityLevelWithRetryTimeout( CellularHandle_t cellularHandle,
                                                                 BG770UEFunctionalityLevel_t *const pUEFunctionalityLevel,
                                                                 uint32_t commandTimeoutMS,
                                                                 uint32_t exponentialBackoffInterCommandBaseMS )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint8_t tryCount = 0;

    if( cellularHandle == NULL || pUEFunctionalityLevel == NULL )
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

            cellularStatus = _GetUEFunctionalityLevel( cellularHandle, pUEFunctionalityLevel, commandTimeoutMS );

            if( cellularStatus == CELLULAR_SUCCESS )
            {
                break;
            }
        }
    }

    return cellularStatus;
}

static CellularError_t _GetUEFunctionalityLevel( CellularHandle_t cellularHandle,
                                                 BG770UEFunctionalityLevel_t *const pUEFunctionalityLevel,
                                                 uint32_t commandTimeoutMS )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetUEFunctionalityLevel =
    {
        "AT+CFUN?",
        CELLULAR_AT_WITH_PREFIX,
        "+CFUN",
        _RecvFuncGetUEFunctionalityLevel,
        pUEFunctionalityLevel,
        sizeof( BG770UEFunctionalityLevel_t ),
    };

    if( pUEFunctionalityLevel == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqGetUEFunctionalityLevel, commandTimeoutMS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetUEFunctionalityLevel: couldn't retrieve UE functionality level" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t _SetDesiredUEFunctionalityLevel( CellularHandle_t cellularHandle, uint32_t commandTimeoutMS )
{
    return _SetUEFunctionalityLevel( cellularHandle, DESIRED_UE_ENABLE_FUNCTIONALITY_LEVEL, commandTimeoutMS );
}

static CellularError_t _SetUEFunctionalityLevel( CellularHandle_t cellularHandle,
                                                 BG770UEFunctionalityLevel_t ueFunctionalityLevel,
                                                 uint32_t commandTimeoutMS )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetUEFunctionalityLevel =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( ueFunctionalityLevel != BG770_UE_FUNCTIONALITY_LEVEL_MINIMUM &&
        ueFunctionalityLevel != BG770_UE_FUNCTIONALITY_LEVEL_FULL &&
        ueFunctionalityLevel != BG770_UE_FUNCTIONALITY_LEVEL_SIM_ONLY )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+CFUN=%s",
                           _getUEFunctionalityLevelString( ueFunctionalityLevel ) );

        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqSetUEFunctionalityLevel, commandTimeoutMS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_SetUEFunctionalityLevel: couldn't set UE functionality level" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static BG770NetworkCategorySearchMode_t _getNetworkCategorySearchMode( const char *const pNetworkCategorySearchModeString )
{
    if( strcmp( pNetworkCategorySearchModeString, NET_CAT_SEARCH_MODE_eMTC_STRING ) == 0 )
    {
        return BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC;
    }
    else if ( strcmp( pNetworkCategorySearchModeString, NET_CAT_SEARCH_MODE_NB_IoT_STRING ) == 0 )
    {
        return BG770_NETWORK_CATEGORY_SEARCH_MODE_NB_IoT;
    }
    else if ( strcmp( pNetworkCategorySearchModeString, NET_CAT_SEARCH_MODE_eMTC_AND_NB_IoT_STRING ) == 0 )
    {
        return BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC_AND_NB_IoT;
    }
    else
    {
        return BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN;
    }
}

static const char * _getNetworkCategorySearchModeString( const BG770NetworkCategorySearchMode_t networkCategorySearchMode )
{
    switch( networkCategorySearchMode )
    {
        case BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC:
            return NET_CAT_SEARCH_MODE_eMTC_STRING;

        case BG770_NETWORK_CATEGORY_SEARCH_MODE_NB_IoT:
            return NET_CAT_SEARCH_MODE_NB_IoT_STRING;

        case BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC_AND_NB_IoT:
            return NET_CAT_SEARCH_MODE_eMTC_AND_NB_IoT_STRING;

        default:
            LogError( ( "_getNetworkCategorySearchModeString: Invalid BG770NetworkCategorySearchMode_t: %d",
                        networkCategorySearchMode ) );
            /**< Intentional fall-through */
        case BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN:
            return "unknown";
    }
}

static bool _parseNetworkCategorySearchMode( char * pQNetworkCategorySearchModePayload,
                                             BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode )
{
    char * pToken = NULL, * pTmpQNetworkCategorySearchModePayload = pQNetworkCategorySearchModePayload;
    bool parseStatus = true;

    if( ( pNetworkCategorySearchMode == NULL ) || ( pQNetworkCategorySearchModePayload == NULL ) )
    {
        LogError( ( "_parseNetworkCategorySearchMode: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQNetworkCategorySearchModePayload, &pToken ) != CELLULAR_AT_SUCCESS ||
            strcmp( pToken, "\"iotopmode\"" ) != 0 )
        {
            LogError( ( "_parseNetworkCategorySearchMode: Error, missing \"iotopmode\"" ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQNetworkCategorySearchModePayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            *pNetworkCategorySearchMode = _getNetworkCategorySearchMode( pToken );
            if( *pNetworkCategorySearchMode == BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN ) {
                LogError( ( "_parseNetworkCategorySearchMode: network category search mode string ('%s') not valid", pToken ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseNetworkCategorySearchMode: network category search mode string not present" ) );
            *pNetworkCategorySearchMode = BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN;
            parseStatus = false;
        }
    }
    else
    {
        if( pNetworkCategorySearchMode != NULL )
        {
            *pNetworkCategorySearchMode = BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN;
        }
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetNetworkCategorySearchMode( CellularContext_t * pContext,
                                                                  const CellularATCommandResponse_t * pAtResp,
                                                                  void * pData,
                                                                  uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode = ( BG770NetworkCategorySearchMode_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pNetworkCategorySearchMode == NULL ) || ( dataLen != sizeof( BG770NetworkCategorySearchMode_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetNetworkCategorySearchMode: Input Line passed is NULL" ) );
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
        parseStatus = _parseNetworkCategorySearchMode( pInputLine, pNetworkCategorySearchMode );

        if( parseStatus != true )
        {
            *pNetworkCategorySearchMode = BG770_NETWORK_CATEGORY_SEARCH_MODE_UNKNOWN;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetNetworkCategorySearchModeWithRetryTimeout( CellularHandle_t cellularHandle,
                                                                      BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode,
                                                                      uint32_t commandTimeoutMS,
                                                                      uint32_t exponentialBackoffInterCommandBaseMS )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    uint8_t tryCount = 0;

    if( cellularHandle == NULL || pNetworkCategorySearchMode == NULL )
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

            cellularStatus = _GetNetworkCategorySearchMode( cellularHandle, pNetworkCategorySearchMode, commandTimeoutMS );

            if( cellularStatus == CELLULAR_SUCCESS )
            {
                break;
            }
        }
    }

    return cellularStatus;
}

static CellularError_t _GetNetworkCategorySearchMode( CellularHandle_t cellularHandle,
                                                      BG770NetworkCategorySearchMode_t * pNetworkCategorySearchMode,
                                                      uint32_t commandTimeoutMS )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetNetworkCategorySearchMode =
    {
        "AT+QCFG=\"iotopmode\"",
        CELLULAR_AT_WITH_PREFIX,
        "+QCFG",
        _RecvFuncGetNetworkCategorySearchMode,
        pNetworkCategorySearchMode,
        sizeof( BG770NetworkCategorySearchMode_t ),
    };

    if( pNetworkCategorySearchMode == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqGetNetworkCategorySearchMode, commandTimeoutMS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetNetworkCategorySearchMode: couldn't retrieve network category search mode" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t _SetDesiredNetworkCategorySearchMode( CellularHandle_t cellularHandle, uint32_t commandTimeoutMS )
{
    return _SetNetworkCategorySearchMode( cellularHandle, DESIRED_NETWORK_CATEGORY_SEARCH_MODE,
                                          true /* applyImmediately= */, commandTimeoutMS );
}

static CellularError_t _SetNetworkCategorySearchMode( CellularHandle_t cellularHandle,
                                                      BG770NetworkCategorySearchMode_t networkCategorySearchMode,
                                                      bool applyImmediately,
                                                      uint32_t commandTimeoutMS )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetNetworkCategorySearchMode =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( networkCategorySearchMode != BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC &&
        networkCategorySearchMode != BG770_NETWORK_CATEGORY_SEARCH_MODE_NB_IoT &&
        networkCategorySearchMode != BG770_NETWORK_CATEGORY_SEARCH_MODE_eMTC_AND_NB_IoT )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+QCFG=\"iotopmode\",%s,%s",
                           _getNetworkCategorySearchModeString( networkCategorySearchMode),
                           ( applyImmediately ? "1" : "0" ) );

        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqSetNetworkCategorySearchMode,
                                                               commandTimeoutMS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_SetNetworkCategorySearchMode: couldn't set network category search mode" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}
