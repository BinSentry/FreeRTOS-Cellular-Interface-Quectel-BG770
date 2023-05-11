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
/* The config header is always included first. */
#ifndef CELLULAR_DO_NOT_USE_CUSTOM_CONFIG
#include "cellular_config.h"
#endif
#include "cellular_config_defaults.h"
#include "cellular_platform.h"
#include "cellular_common.h"
#include "cellular_common_portable.h"
#include "cellular_bg96.h"

/*-----------------------------------------------------------*/

#define ENABLE_MODULE_UE_RETRY_COUNT       ( 4U )
#define ENABLE_MODULE_UE_RETRY_TIMEOUT     ( 15000U )   /* observed at least 11000 */
#define BG770_NWSCANSEQ_CMD_MAX_SIZE       ( 30U ) /* Need at least the length of AT+QCFG="nwscanseq",020301,1\0. */

static const TickType_t APP_READY_MAX_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 10000U );
static const TickType_t POST_APP_READY_WAIT_PERIOD_ticks = pdMS_TO_TICKS( 5000U );

static const TickType_t SHORT_DELAY_ticks = pdMS_TO_TICKS( 10U );

/*-----------------------------------------------------------*/

static CellularError_t sendAtCommandWithRetryTimeout( CellularContext_t * pContext,
                                                      const CellularAtReq_t * pAtReq );

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
{ "APP RDY", "POWERED DOWN", "RDY" };
/* FreeRTOS Cellular Common Library porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
uint32_t CellularUrcTokenWoPrefixTableSize = sizeof( CellularUrcTokenWoPrefixTable ) / sizeof( char * );

/*-----------------------------------------------------------*/

static CellularError_t sendAtCommandWithRetryTimeoutParams( CellularContext_t * pContext,
                                                            const CellularAtReq_t * pAtReq,
                                                            uint32_t commandTimeoutMS,
                                                            uint32_t exponentialBackoffInterCommandBaseMS
                                                            )
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
    CellularAtReq_t atReqGetNoResult =
    {
        NULL,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0
    };
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
                LogInfo( ( "Cellular_ModuleEnableUE: 'APP_RDY' URC received" ) );
            }
            else
            {
                LogWarn( ( "Cellular_ModuleEnableUE: Init event flag 'APP_RDY received' timeout (after waiting %lu ticks)", APP_READY_MAX_WAIT_PERIOD_ticks ) );
            }
        }
        else
        {
            LogError( ( "Cellular_ModuleEnableUE: Failed to wait on Init event flag 'APP_RDY received', waiting %lu ticks", APP_READY_MAX_WAIT_PERIOD_ticks ) );
            vTaskDelay( APP_READY_MAX_WAIT_PERIOD_ticks );
        }

        vTaskDelay( POST_APP_READY_WAIT_PERIOD_ticks );

        /* Empty command, looking for 'OK' to indicate presence of module. */
        atReqGetWithResult.pAtCmd = "AT";
        cellularStatus = sendAtCommandWithRetryTimeoutParams( pContext, &atReqGetWithResult, 1000UL, 100UL );

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            /* Disable echo. */
            atReqGetWithResult.pAtCmd = "ATE0";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetWithResult );
        }

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            /* Disable DTR function. */
            atReqGetNoResult.pAtCmd = "AT&D0";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
        }

        #ifndef CELLULAR_CONFIG_DISABLE_FLOW_CONTROL
            if( cellularStatus == CELLULAR_SUCCESS )
            {
                vTaskDelay( SHORT_DELAY_ticks );

                /* Enable RTS/CTS hardware flow control. */
                atReqGetNoResult.pAtCmd = "AT+IFC=2,2";
                cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
            }
        #endif

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Setting URC output port. */
            #if defined( CELLULAR_BG770_URC_PORT_EMUX ) || defined( BG770_URC_PORT_EMUX )
                atReqGetNoResult.pAtCmd = "AT+QURCCFG=\"urcport\",\"emux\"";
            #else
                atReqGetNoResult.pAtCmd = "AT+QURCCFG=\"urcport\",\"main\"";
            #endif
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
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

            /* Configure Band configuration to all bands.
             * NOTE: Order - GSM, LTE, NB-IoT.
             * GSM (bandmask: 'f') and NB-IoT (bandmask: '200000000090f189f') irrelevant at this point so send '0'
             * which means don't update */
            atReqGetNoResult.pAtCmd = "AT+QCFG=\"band\",0,2000000000f0e189f,0";   // FUTURE: Make this configurable
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
        }

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            /* Configure Network Category to be Searched under LTE RAT to eMTC only. */
            atReqGetNoResult.pAtCmd = "AT+QCFG=\"iotopmode\",0,1";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
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
        }

        if( cellularStatus == CELLULAR_SUCCESS )
        {
            vTaskDelay( SHORT_DELAY_ticks );

            atReqGetNoResult.pAtCmd = "AT+CFUN=1";
            cellularStatus = sendAtCommandWithRetryTimeout( pContext, &atReqGetNoResult );
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

    return cellularStatus;  // TODO (MV): Why are these call's return code not used?
}

/*-----------------------------------------------------------*/
