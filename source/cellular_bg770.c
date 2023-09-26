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
#define ENABLE_MODULE_UE_RETRY_TIMEOUT     ( 15000U )   /* observed at least 11000 */
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

typedef enum BG77URCIndicationOption
{
    BG770_URC_INDICATION_OPTION_MAIN = 0,  /**< URC output on main UART. */
    BG770_URC_INDICATION_OPTION_AUX,       /**< URC output on aux UART. */
    BG770_URC_INDICATION_OPTION_EMUX,      /**< URC output on emux UART. */
    BG770_URC_INDICATION_OPTION_UNKNOWN    /**< Unknown URC output. */
} BG770URCIndicationOptionType_t;

static const char * URCCFG_URCPORT_MAIN = "\"main\"";
static const char * URCCFG_URCPORT_AUX = "\"aux\"";
static const char * URCCFG_URCPORT_EMUX = "\"emux\"";

static const TickType_t APP_READY_MAX_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 10000U );
static const TickType_t POST_APP_READY_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 5000U );

static const TickType_t SHORT_DELAY_ticks = pdMS_TO_TICKS( 10U );

/*-----------------------------------------------------------*/

static CellularError_t sendAtCommandWithRetryTimeout( CellularContext_t * pContext,
                                                      const CellularAtReq_t * pAtReq );

static CellularError_t _GetFrequencyBands( CellularHandle_t cellularHandle,
                                           BG770FrequencyBands_t * pFrequencyBands );

static CellularError_t _GetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t * pURCIndicationOptionType );

static CellularError_t _SetURCIndicationOption( CellularHandle_t cellularHandle,
                                                BG770URCIndicationOptionType_t urcIndicationOptionType );

/*-----------------------------------------------------------*/

static cellularModuleContext_t cellularBg770Context = { 0 };

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

static void _removeHexValuePrefixIfPresent(const char **pString) {
    static const char *HEX_PREFIX_1 = "0x";
    static const char *HEX_PREFIX_2 = "0x";
    static const size_t HEX_PREFIX_LENGTH = 2;

    if( pString == NULL || *pString == NULL )
    {
        LogError( ( "NULL string in _removeHexValuePrefixIfPresent" ) );
        return;
    }

    if( strncmp( *pString, HEX_PREFIX_1, HEX_PREFIX_LENGTH ) == 0 ||
        strncmp( *pString, HEX_PREFIX_2, HEX_PREFIX_LENGTH ) == 0 )
    {
        *pString += HEX_PREFIX_LENGTH;
    }
}

// NOTE: strings expected to be free of whitespace and may contain
static bool _areHexStringChannelMasksEquivalent(const char *channelMaskHexString1, const char *channelMaskHexString2) {

    if( channelMaskHexString1 == NULL || channelMaskHexString2 == NULL )
    {
        return false;
    }

    _removeHexValuePrefixIfPresent( &channelMaskHexString1 );
    _removeHexValuePrefixIfPresent( &channelMaskHexString2 );

    const size_t channelMask1Size = strlen( channelMaskHexString1 );
    const size_t channelMask2Size = strlen( channelMaskHexString2 );

    if( channelMask1Size == channelMask2Size &&
        memcmp( channelMaskHexString1, channelMaskHexString2, channelMask1Size ) == 0 )
    {
        return true;    // check for strict equality first
    }

    // next check if channel masks are equivalent if initial zero bytes are removed
    if( channelMask1Size > 0 && channelMaskHexString2 > 0 )
    {
        int firstImportantByte1;
        for( firstImportantByte1 = 0; firstImportantByte1 < channelMask1Size; firstImportantByte1++ )
        {
            if( channelMaskHexString1[ firstImportantByte1 ] != '0' )
            {
                break;
            }
        }

        int firstImportantByte2;
        for( firstImportantByte2 = 0; firstImportantByte2 < channelMask2Size; firstImportantByte2++ )
        {
            if( channelMaskHexString2[ firstImportantByte2 ] != '0')
            {
                break;
            }
        }

        // confirm there is still something to compare after initial zeroes are removed
        if( firstImportantByte1 < channelMask1Size && firstImportantByte2 < channelMask2Size )
        {
            size_t importantChannelMask1Size = channelMask1Size - firstImportantByte1;
            size_t importantChannelMask2Size = channelMask2Size - firstImportantByte2;

            if( importantChannelMask1Size == importantChannelMask2Size
                && memcmp( &channelMaskHexString1[ firstImportantByte1 ], &channelMaskHexString2[ firstImportantByte2 ],
                           importantChannelMask1Size ) == 0 )
            {
                return true;
            }
        }
    }

    return false;
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
            /* Disable echo. */
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
            /* Disable DTR function. */
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

                /* Enable RTS/CTS hardware flow control. */
                atReqGetNoResult.pAtCmd = "AT+IFC=2,2";
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

#ifdef CELLULAR_QUECTEL_ENABLE_DEBUG_UART
        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Setting debug output enable. */
            atReqGetNoResult.pAtCmd = "AT+QCFGEXT=\"debug\",1";
            CellularError_t debugResult = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            if( debugResult != CELLULAR_SUCCESS )
            {
                LogWarn( ( "Cellular_ModuleEnableUE: Debug output enable failed" ) );
            }
        }
        else
        {
            LogWarn( ( "Cellular_ModuleEnableUE: Debug output enable skipped due to error" ) );
        }
#endif

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            static const char * LTE_BANDMASK_HEX_STRING = "0x2000000000f0e189f";       // FUTURE: Make this configurable

            /* Configure Band configuration to all bands.
             * NOTE: Order - GSM, LTE, NB-IoT.
             * GSM (bandmask: 'f') and NB-IoT (bandmask: '200000000090f189f') irrelevant at this point so send '0'
             * which means don't update */

            /* MISRA Ref 21.6.1 [Use of snprintf] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
            /* coverity[misra_c_2012_rule_21_6_violation]. */
            ( void ) snprintf( cmdBuf, sizeof ( cmdBuf ), "AT+QCFG=\"band\",0,%s,0", LTE_BANDMASK_HEX_STRING );

            atReqGetNoResult.pAtCmd = cmdBuf;

            BG770FrequencyBands_t frequencyBands = { 0 };
            CellularError_t getFreqBandsStatus = _GetFrequencyBands( pContext, &frequencyBands );
            if( getFreqBandsStatus != CELLULAR_SUCCESS ||
                !_areHexStringChannelMasksEquivalent( frequencyBands.lteBands_hexString, LTE_BANDMASK_HEX_STRING ) )
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

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Configure Network Category to be Searched under LTE RAT to eMTC only. */
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

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            atReqGetNoResult.pAtCmd = "AT+CFUN=1";
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

static bool _parseFrequencyBands( char * pQcfgBandsPayload,
                                  BG770FrequencyBands_t * pFrequencyBands )
{
    char * pToken = NULL, * pTmpQcfgBandsPayload = pQcfgBandsPayload;
    bool parseStatus = true;

    if( ( pFrequencyBands == NULL ) || ( pQcfgBandsPayload == NULL ) )
    {
        LogError( ( "_parseFrequencyBands: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQcfgBandsPayload, &pToken ) != CELLULAR_AT_SUCCESS ||
            strcmp( pToken, "\"band\"" ) != 0 )
        {
            LogError( ( "_parseFrequencyBands: Error, missing \"band\"" ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        // NOTE: expectation that token is present but don't care about the value since GSM is not supported
        if( Cellular_ATGetNextTok( &pTmpQcfgBandsPayload, &pToken ) != CELLULAR_AT_SUCCESS )
        {
            LogError( ( "_parseFrequencyBands: Error, missing GSM frequency bands" ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQcfgBandsPayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            (void) strncpy( pFrequencyBands->lteBands_hexString, pToken, sizeof ( pFrequencyBands->lteBands_hexString ) );
            if( pFrequencyBands->lteBands_hexString[ sizeof ( pFrequencyBands->lteBands_hexString ) - 1 ] != '\0' )
            {
                pFrequencyBands->lteBands_hexString[ sizeof ( pFrequencyBands->lteBands_hexString ) - 1 ] = '\0';
                LogError( ( "_parseFrequencyBands: lteBands string truncation. Token '%s' lteBands_hexString '%s'",
                            pToken, pFrequencyBands->lteBands_hexString ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseFrequencyBands: lteBands string not present" ) );
            pFrequencyBands->lteBands_hexString[ 0 ] = '\0';
            pFrequencyBands->nbIotBands_hexString[ 0 ] = '\0';
            parseStatus = false;
        }
    }
    else
    {
        pFrequencyBands->lteBands_hexString[ 0 ] = '\0';
        pFrequencyBands->nbIotBands_hexString[ 0 ] = '\0';
    }

    if( parseStatus == true )
    {
        if( Cellular_ATGetNextTok( &pTmpQcfgBandsPayload, &pToken ) == CELLULAR_AT_SUCCESS )
        {
            (void) strncpy( pFrequencyBands->nbIotBands_hexString, pToken, sizeof ( pFrequencyBands->nbIotBands_hexString ) );
            if( pFrequencyBands->nbIotBands_hexString[ sizeof ( pFrequencyBands->nbIotBands_hexString ) - 1 ] != '\0' )
            {
                pFrequencyBands->nbIotBands_hexString[ sizeof ( pFrequencyBands->nbIotBands_hexString ) - 1 ] = '\0';
                LogError( ( "_parseFrequencyBands: nbIotBands string truncation. Token '%s' nbIotBands_hexString '%s'",
                            pToken, pFrequencyBands->nbIotBands_hexString ) );
                parseStatus = false;
            }
        }
        else
        {
            LogError( ( "_parseFrequencyBands: nbIotBands string not present" ) );
            pFrequencyBands->nbIotBands_hexString[ 0 ] = '\0';
            parseStatus = false;
        }
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _RecvFuncGetFrequencyBands( CellularContext_t * pContext,
                                                       const CellularATCommandResponse_t * pAtResp,
                                                       void * pData,
                                                       uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770FrequencyBands_t * pFrequencyBands = ( BG770FrequencyBands_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pFrequencyBands == NULL ) || ( dataLen != sizeof( BG770FrequencyBands_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "_GetFrequencyBands: Input Line passed is NULL" ) );
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
        parseStatus = _parseFrequencyBands( pInputLine, pFrequencyBands );

        if( parseStatus != true )
        {
            pFrequencyBands->lteBands_hexString[0] = '\0';
            pFrequencyBands->nbIotBands_hexString[0] = '\0';
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularError_t _GetFrequencyBands( CellularHandle_t cellularHandle,
                                           BG770FrequencyBands_t * pFrequencyBands )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetFrequencyBands =
    {
        "AT+QCFG=\"band\"",
        CELLULAR_AT_WITH_PREFIX,
        "+QCFG",
        _RecvFuncGetFrequencyBands,
        pFrequencyBands,
        sizeof( BG770FrequencyBands_t ),
    };

    if( pFrequencyBands == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetFrequencyBands );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_GetFrequencyBands: couldn't retrieve frequency bands" ) );
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

