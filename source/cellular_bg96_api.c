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

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>

#include "cellular_platform.h"
/* The config header is always included first. */
#ifndef CELLULAR_DO_NOT_USE_CUSTOM_CONFIG
#include "cellular_config.h"
#endif
#include "cellular_config_defaults.h"

#include "cellular_types.h"
#include "cellular_api.h"
#include "cellular_common_api.h"
#include "cellular_common.h"
#include "cellular_at_core.h"
#include "cellular_bg96.h"

/*-----------------------------------------------------------*/

#define CELLULAR_AT_CMD_TYPICAL_MAX_SIZE           ( 32U )
#define CELLULAR_AT_CMD_QUERY_DNS_MAX_SIZE         ( 280U )

#define SIGNAL_QUALITY_POS_SYSMODE                 ( 1U )
#define SIGNAL_QUALITY_POS_GSM_LTE_RSSI            ( 2U )
#define SIGNAL_QUALITY_POS_LTE_RSRP                ( 3U )
#define SIGNAL_QUALITY_POS_LTE_SINR                ( 4U )
#define SIGNAL_QUALITY_POS_LTE_RSRQ                ( 5U )
#define SIGNAL_QUALITY_SINR_MIN_VALUE              ( -20 )
#define SIGNAL_QUALITY_SINR_DIVISIBILITY_FACTOR    ( 5 )

#define COPS_POS_MODE                              ( 1U )
#define COPS_POS_FORMAT                            ( 2U )
#define COPS_POS_MCC_MNC_OPER_NAME                 ( 3U )
#define COPS_POS_RAT                               ( 4U )

/* AT command timeout for Get IP Address by Domain Name. */
#define DNS_QUERY_TIMEOUT_MS                       ( 60000UL )

/* Length of HPLMN including RAT. */
#define CRSM_HPLMN_RAT_LENGTH                      ( 9U )

/* Windows simulator implementation. */
#if defined( _WIN32 ) || defined( _WIN64 )
    #define strtok_r                  strtok_s
#endif

#define PRINTF_BINARY_PATTERN_INT4    "%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT4( i )            \
    ( ( ( ( i ) & 0x08UL ) != 0UL ) ? '1' : '0' ), \
    ( ( ( ( i ) & 0x04UL ) != 0UL ) ? '1' : '0' ), \
    ( ( ( ( i ) & 0x02UL ) != 0UL ) ? '1' : '0' ), \
    ( ( ( ( i ) & 0x01UL ) != 0UL ) ? '1' : '0' )

#define PRINTF_BINARY_PATTERN_INT8 \
    PRINTF_BINARY_PATTERN_INT4 PRINTF_BINARY_PATTERN_INT4
#define PRINTF_BYTE_TO_BINARY_INT8( i ) \
    PRINTF_BYTE_TO_BINARY_INT4( ( i ) >> 4 ), PRINTF_BYTE_TO_BINARY_INT4( i )

#define QPSMS_POS_MODE                           ( 0U )
#define QPSMS_POS_RAU                            ( 1U )
#define QPSMS_POS_RDY_TIMER                      ( 2U )
#define QPSMS_POS_TAU                            ( 3U )
#define QPSMS_POS_ACTIVE_TIME                    ( 4U )

#define CELLULAR_PDN_STATUS_POS_CONTEXT_ID       ( 0U )
#define CELLULAR_PDN_STATUS_POS_CONTEXT_STATE    ( 1U )
#define CELLULAR_PDN_STATUS_POS_CONTEXT_TYPE     ( 2U )
#define CELLULAR_PDN_STATUS_POS_IP_ADDRESS       ( 3U )

#define RAT_PRIORITY_STRING_LENGTH               ( 2U )
#define RAT_PRIORITY_LIST_LENGTH                 ( 3U )

#define INVALID_PDN_INDEX                        ( 0xFFU )

#define SOCKET_DATA_PREFIX_STRING                "+QIRD:"
#define SOCKET_DATA_PREFIX_STRING_LENGTH         ( 6U )
#define DATA_PREFIX_STRING_CHANGELINE_LENGTH     ( 2U )     /* The length of the change line "\r\n". */

#define SSL_SOCKET_DATA_PREFIX_STRING             "+QSSLRECV:"
#define SSL_SOCKET_DATA_PREFIX_STRING_LENGTH      ( 10U )

#define MAX_QIRD_STRING_PREFIX_STRING            ( 14U )    /* The max data prefix string is "+QIRD: 1500\r\n" */
#define MAX_QSSLRECV_STRING_PREFIX_STRING        ( 18U )    /* The max data prefix string is "+QSSLRECV: 1500\r\n" */

/*-----------------------------------------------------------*/

static const int FLOW_CONTROL_NONE = 0;
static const int RTS_FLOW_CONTROL_ENABLED = 2;
static const int CTS_FLOW_CONTROL_ENABLED = 2;

/*-----------------------------------------------------------*/

/**
 * @brief Parameters involved in receiving data through sockets
 */
typedef struct _socketDataRecv
{
    uint32_t * pDataLen;
    uint8_t * pData;
    CellularSocketAddress_t * pRemoteSocketAddress;
} _socketDataRecv_t;

/*-----------------------------------------------------------*/

static bool _parseSignalQuality( char * pQcsqPayload,
                                 CellularSignalInfo_t * pSignalInfo );
static CellularPktStatus_t _Cellular_RecvFuncGetSignalInfo( CellularContext_t * pContext,
                                                            const CellularATCommandResponse_t * pAtResp,
                                                            void * pData,
                                                            uint16_t dataLen );
static CellularError_t controlSignalStrengthIndication( CellularContext_t * pContext,
                                                        bool enable );
static CellularPktStatus_t _Cellular_RecvFuncGetIccid( CellularContext_t * pContext,
                                                       const CellularATCommandResponse_t * pAtResp,
                                                       void * pData,
                                                       uint16_t dataLen );
static CellularPktStatus_t _Cellular_RecvFuncGetImsi( CellularContext_t * pContext,
                                                      const CellularATCommandResponse_t * pAtResp,
                                                      void * pData,
                                                      uint16_t dataLen );
static bool _checkCrsmMemoryStatus( const char * pToken );
static bool _checkCrsmReadStatus( const char * pToken );
static bool _parseHplmn( char * pToken,
                         void * pData );
static CellularPktStatus_t _Cellular_RecvFuncGetHplmn( CellularContext_t * pContext,
                                                       const CellularATCommandResponse_t * pAtResp,
                                                       void * pData,
                                                       uint16_t dataLen );
static CellularPktStatus_t _Cellular_RecvFuncGetSimCardStatus( CellularContext_t * pContext,
                                                               const CellularATCommandResponse_t * pAtResp,
                                                               void * pData,
                                                               uint16_t dataLen );
static CellularSimCardLockState_t _getSimLockState( char * pToken );
static CellularPktStatus_t _Cellular_RecvFuncGetSimLockStatus( CellularContext_t * pContext,
                                                               const CellularATCommandResponse_t * pAtResp,
                                                               void * pData,
                                                               uint16_t dataLen );
static CellularATError_t parsePdnStatusContextId( char * pToken,
                                                  CellularPdnStatus_t * pPdnStatusBuffers );
static CellularATError_t parsePdnStatusContextState( char * pToken,
                                                     CellularPdnStatus_t * pPdnStatusBuffers );
static CellularATError_t parsePdnStatusContextType( char * pToken,
                                                    CellularPdnStatus_t * pPdnStatusBuffers );
static CellularATError_t getPdnStatusParseToken( char * pToken,
                                                 uint8_t tokenIndex,
                                                 CellularPdnStatus_t * pPdnStatusBuffers );
static CellularATError_t getPdnStatusParseLine( char * pRespLine,
                                                CellularPdnStatus_t * pPdnStatusBuffers );
static CellularPktStatus_t _Cellular_RecvFuncGetPdnStatus( CellularContext_t * pContext,
                                                           const CellularATCommandResponse_t * pAtResp,
                                                           void * pData,
                                                           uint16_t dataLen );
static CellularError_t buildSocketConnect( CellularSocketHandle_t socketHandle,
                                           char * pCmdBuf, size_t cmdBufLength );
static CellularATError_t getDataFromResp( const CellularATCommandResponse_t * pAtResp,
                                          const _socketDataRecv_t * pDataRecv,
                                          uint32_t outBufSize );
static CellularPktStatus_t _Cellular_RecvFuncData( CellularContext_t * pContext,
                                                   const CellularATCommandResponse_t * pAtResp,
                                                   void * pData,
                                                   uint16_t dataLen );
static CellularATError_t parseQpsmsMode( char * pToken,
                                         CellularPsmSettings_t * pPsmSettings );
static CellularATError_t parseQpsmsRau( char * pToken,
                                        CellularPsmSettings_t * pPsmSettings );
static CellularATError_t parseQpsmsRdyTimer( char * pToken,
                                             CellularPsmSettings_t * pPsmSettings );
static CellularATError_t parseQpsmsTau( char * pToken,
                                        CellularPsmSettings_t * pPsmSettings );
static CellularATError_t parseQpsmsActiveTime( char * pToken,
                                               CellularPsmSettings_t * pPsmSettings );
static CellularATError_t parseGetPsmToken( char * pToken,
                                           uint8_t tokenIndex,
                                           CellularPsmSettings_t * pPsmSettings );
static CellularRat_t convertRatPriority( char * pRatString );
static CellularPktStatus_t _Cellular_RecvFuncGetRatPriority( CellularContext_t * pContext,
                                                             const CellularATCommandResponse_t * pAtResp,
                                                             void * pData,
                                                             uint16_t dataLen );
static CellularPktStatus_t _Cellular_RecvFuncGetPsmSettings( CellularContext_t * pContext,
                                                             const CellularATCommandResponse_t * pAtResp,
                                                             void * pData,
                                                             uint16_t dataLen );
static CellularPktStatus_t socketRecvDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t lineLength,
                                                 char ** ppDataStart,
                                                 uint32_t * pDataLength );
static CellularPktStatus_t sslSocketRecvDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t lineLength,
                                                 char ** ppDataStart,
                                                 uint32_t * pDataLength );
static CellularError_t storeAccessModeAndAddress( CellularContext_t * pContext,
                                                  CellularSocketHandle_t socketHandle,
                                                  CellularSocketAccessMode_t dataAccessMode,
                                                  const CellularSocketAddress_t * pRemoteSocketAddress );
static CellularError_t registerDnsEventCallback( cellularModuleContext_t * pModuleContext,
                                                 CellularDnsResultEventCallback_t dnsEventCallback,
                                                 char * pDnsUsrData );
static void _dnsResultCallback( cellularModuleContext_t * pModuleContext,
                                char * pDnsResult,
                                char * pDnsUsrData );
static uint32_t appendBinaryPattern( char * cmdBuf,
                                     uint32_t cmdLen,
                                     uint32_t value,
                                     bool endOfString );
static CellularPktStatus_t socketSendDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t * pBytesRead );

/*-----------------------------------------------------------*/

static bool _parseSignalQuality( char * pQcsqPayload,
                                 CellularSignalInfo_t * pSignalInfo )
{
    char * pToken = NULL, * pTmpQcsqPayload = pQcsqPayload;
    int32_t tempValue = 0;
    bool parseStatus = true;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( ( pSignalInfo == NULL ) || ( pQcsqPayload == NULL ) )
    {
        LogError( ( "_parseSignalQuality: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( ( parseStatus == true ) && ( Cellular_ATGetNextTok( &pTmpQcsqPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        if( ( strcmp( pToken, "eMTC" ) != 0 ) &&
            ( strcmp( pToken, "NBIoT" ) != 0 ) )
        {
            parseStatus = false;
        }
    }
    else
    {
        LogDebug( ( "_parseSignalQuality: No Valid RAT in QCSQ Response" ) );
        parseStatus = false;
    }

    if( ( parseStatus == true ) && ( Cellular_ATGetNextTok( &pTmpQcsqPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSignalInfo->rssi = ( int16_t ) tempValue;  // NOTE: RSSI value does not need conversion, alread value between -113 and -51
        }
        else
        {
            LogError( ( "_parseSignalQuality: Error in processing RSSI. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && ( Cellular_ATGetNextTok( &pTmpQcsqPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSignalInfo->rsrp = ( int16_t ) tempValue;
        }
        else
        {
            LogError( ( "_parseSignalQuality: Error in processing RSRP. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && ( Cellular_ATGetNextTok( &pTmpQcsqPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* SINR is reported as an integer value ranging from 0 to 250 representing 1/5 of a dB.
             * Value 0 correspond to -20 dBm and 250 corresponds to +30 dBm. */
            pSignalInfo->sinr = ( int16_t ) ( SIGNAL_QUALITY_SINR_MIN_VALUE + ( ( tempValue ) / ( SIGNAL_QUALITY_SINR_DIVISIBILITY_FACTOR ) ) );
        }
        else
        {
            LogError( ( "_parseSignalQuality: Error in processing SINR. pToken %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && ( Cellular_ATGetNextTok( &pTmpQcsqPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSignalInfo->rsrq = ( int16_t ) tempValue;
        }
        else
        {
            LogError( ( "_parseSignalQuality: Error in processing RSRQ. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    return parseStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetSignalInfo( CellularContext_t * pContext,
                                                            const CellularATCommandResponse_t * pAtResp,
                                                            void * pData,
                                                            uint16_t dataLen )
{
    char * pInputLine = NULL;
    CellularSignalInfo_t * pSignalInfo = ( CellularSignalInfo_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pSignalInfo == NULL ) || ( dataLen != sizeof( CellularSignalInfo_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "GetSignalInfo: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pInputLine );
        }

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
        parseStatus = _parseSignalQuality( pInputLine, pSignalInfo );

        if( parseStatus != true )
        {
            pSignalInfo->rssi = CELLULAR_INVALID_SIGNAL_VALUE;
            pSignalInfo->rsrp = CELLULAR_INVALID_SIGNAL_VALUE;
            pSignalInfo->rsrq = CELLULAR_INVALID_SIGNAL_VALUE;
            pSignalInfo->ber = CELLULAR_INVALID_SIGNAL_VALUE;
            pSignalInfo->bars = CELLULAR_INVALID_SIGNAL_BAR_VALUE;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t controlSignalStrengthIndication( CellularContext_t * pContext,
                                                        bool enable )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };
    uint8_t enable_value = 0;
    CellularAtReq_t atReqControlSignalStrengthIndication =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( enable == true )
    {
        enable_value = 1;
    }
    else
    {
        enable_value = 0;
    }

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "AT+QINDCFG=\"csq\",%u", enable_value );
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqControlSignalStrengthIndication );
        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetIccid( CellularContext_t * pContext,
                                                       const CellularATCommandResponse_t * pAtResp,
                                                       void * pData,
                                                       uint16_t dataLen )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pRespLine = NULL;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) )
    {
        LogError( ( "getIccid: Response in invalid " ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pRespLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pRespLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* Removing QCCID Prefix in AT Response. */
            atCoreStatus = Cellular_ATRemovePrefix( &pRespLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* Storing the ICCID value in the AT Response. */
            if( strlen( pRespLine ) < ( ( size_t ) CELLULAR_ICCID_MAX_SIZE + 1U ) )
            {
                ( void ) strncpy( pData, pRespLine, dataLen );
            }
            else
            {
                atCoreStatus = CELLULAR_AT_BAD_PARAMETER;
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetImsi( CellularContext_t * pContext,
                                                      const CellularATCommandResponse_t * pAtResp,
                                                      void * pData,
                                                      uint16_t dataLen )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pRespLine = NULL;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) )
    {
        LogError( ( "getImsi: Response in invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pRespLine = pAtResp->pItm->pLine;

        /* Removing all the Spaces in the AT Response. */
        atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pRespLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            if( strlen( pRespLine ) < ( CELLULAR_IMSI_MAX_SIZE + 1U ) )
            {
                ( void ) strncpy( ( char * ) pData, pRespLine, dataLen );
            }
            else
            {
                atCoreStatus = CELLULAR_AT_ERROR;
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static bool _checkCrsmMemoryStatus( const char * pToken )
{
    bool memoryStatus = true;

    if( pToken == NULL )
    {
        LogError( ( "Input Parameter NULL" ) );
        memoryStatus = false;
    }

    if( memoryStatus )
    {
        /* checking the value sw2 in AT command response for memory problem during CRSM read.
         * Refer 3GPP Spec TS 51.011 Section 9.4. */
        if( strcmp( pToken, "64" ) == 0 )
        {
            LogError( ( "_checkCrsmMemoryStatus: Error in Processing HPLMN: CRSM Memory Error" ) );
            memoryStatus = false;
        }
    }

    return memoryStatus;
}

/*-----------------------------------------------------------*/

static bool _checkCrsmReadStatus( const char * pToken )
{
    bool readStatus = true;

    if( pToken == NULL )
    {
        LogError( ( "Input Parameter NULL" ) );
        readStatus = false;
    }

    if( readStatus )
    {
        /* checking the parameter sw1 in AT command response for successful CRSM read.
         * Refer 3GPP Spec TS 51.011 Section 9.4. */
        if( ( strcmp( pToken, "144" ) != 0 ) &&
            ( strcmp( pToken, "145" ) != 0 ) &&
            ( strcmp( pToken, "146" ) != 0 ) )
        {
            LogError( ( "_checkCrsmReadStatus: Error in Processing HPLMN: CRSM Read Error" ) );
            readStatus = false;
        }
    }

    return readStatus;
}

/*-----------------------------------------------------------*/

static bool _parseHplmn( char * pToken,
                         void * pData )
{
    bool parseStatus = true;
    CellularPlmnInfo_t * plmn = ( CellularPlmnInfo_t * ) pData;

    if( ( pToken == NULL ) || ( pData == NULL ) )
    {
        LogError( ( "Input Parameter NULL" ) );
        parseStatus = false;
    }

    if( parseStatus == true )
    {
        /* Checking if the very first HPLMN entry in AT command Response is valid*/
        if( ( strlen( pToken ) < ( CRSM_HPLMN_RAT_LENGTH ) ) || ( strncmp( pToken, "FFFFFF", 6 ) == 0 ) )
        {
            LogError( ( "_parseHplmn: Error in Processing HPLMN: Invalid Token %s", pToken ) );
            parseStatus = false;
        }
    }

    if( parseStatus == true )
    {
        /* Returning only the very first HPLMN present in EFHPLMNwACT in SIM.
         * EF-HPLMNwACT can contain a maximum of 10 HPLMN entries in decreasing order of priority.
         * In this implementation, returning the very first HPLMN is the PLMN priority list. */
        /* Refer TS 51.011 Section 10.3.37 for encoding. */
        /**< NOTE: Null termination of mcc relies on plmn being memset beforehand */
        plmn->mcc[ 0 ] = pToken[ 1 ];
        plmn->mcc[ 1 ] = pToken[ 0 ];
        plmn->mcc[ 2 ] = pToken[ 3 ];
        plmn->mnc[ 0 ] = pToken[ 5 ];
        plmn->mnc[ 1 ] = pToken[ 4 ];

        if( pToken[ 2 ] != 'F' )
        {
            plmn->mnc[ 2 ] = pToken[ 2 ];
            plmn->mnc[ 3 ] = '\0';
        }
        else
        {
            plmn->mnc[ 2 ] = '\0';
        }
    }

    return parseStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetHplmn( CellularContext_t * pContext,
                                                       const CellularATCommandResponse_t * pAtResp,
                                                       void * pData,
                                                       uint16_t dataLen )
{
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pCrsmResponse = NULL, * pToken = NULL;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) ||
             ( pData == NULL ) || ( dataLen != sizeof( CellularPlmnInfo_t ) ) )
    {
        LogError( ( "GetHplmn: Response is invalid " ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pCrsmResponse = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pCrsmResponse );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* Removing the CRSM prefix in AT Response. */
            atCoreStatus = Cellular_ATRemovePrefix( &pCrsmResponse );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* Removing All quotes in the AT Response. */
            atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pCrsmResponse );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            /* Getting the next token separated by comma in At Response*/
            atCoreStatus = Cellular_ATGetNextTok( &pCrsmResponse, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            parseStatus = _checkCrsmReadStatus( pToken );

            if( !parseStatus )
            {
                atCoreStatus = CELLULAR_AT_ERROR;
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pCrsmResponse, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            parseStatus = _checkCrsmMemoryStatus( pToken );

            if( !parseStatus )
            {
                atCoreStatus = CELLULAR_AT_ERROR;
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pCrsmResponse, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            parseStatus = _parseHplmn( pToken, pData );

            if( !parseStatus )
            {
                atCoreStatus = CELLULAR_AT_ERROR;
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetSimCardStatus( CellularContext_t * pContext,
                                                               const CellularATCommandResponse_t * pAtResp,
                                                               void * pData,
                                                               uint16_t dataLen )
{
    char * pInputLine = NULL;
    const char * pTokenPtr = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularSimCardState_t * pSimCardState = ( CellularSimCardState_t * ) pData;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "GetSimStatus: response is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pData == NULL ) || ( dataLen != sizeof( CellularSimCardState_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemoveLeadingWhiteSpaces( &pInputLine );
        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            /* remove the token prefix. */
            pTokenPtr = strtok_r( pInputLine, ":", &pInputLine );

            /* check the token prefix. */
            if( pTokenPtr == NULL )
            {
                pktStatus = CELLULAR_PKT_STATUS_BAD_RESPONSE;
            }
            else
            {
                pktStatus = _Cellular_ParseSimstat( pInputLine, pSimCardState );
            }
        }
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularSimCardLockState_t _getSimLockState( char * pToken )
{
    CellularSimCardLockState_t tempState = CELLULAR_SIM_CARD_LOCK_UNKNOWN;

    if( pToken != NULL )
    {
        if( strcmp( pToken, "READY" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_READY;
        }
        else if( strcmp( pToken, "SIM PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PIN;
        }
        else if( strcmp( pToken, "SIM PUK" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PUK;
        }
        else if( strcmp( pToken, "SIM PIN2" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PIN2;
        }
        else if( strcmp( pToken, "SIM PUK2" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PUK2;
        }
        else if( strcmp( pToken, "PH-SIM PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PH_SIM_PIN;
        }
        else if( strcmp( pToken, "PH-NET PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PH_NET_PIN;
        }
        else if( strcmp( pToken, "PH-NET PUK" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PH_NET_PUK;
        }
        else if( strcmp( pToken, "PH-NETSUB PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PH_NETSUB_PIN;
        }
        else if( strcmp( pToken, "PH-NETSUB PUK" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_PH_NETSUB_PUK;
        }
        else if( strcmp( pToken, "PH-SP PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_SP_PIN;
        }
        else if( strcmp( pToken, "PH-SP PUK" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_SP_PUK;
        }
        else if( strcmp( pToken, "PH-CORP PIN" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_CORP_PIN;
        }
        else if( strcmp( pToken, "PH-CORP PUK" ) == 0 )
        {
            tempState = CELLULAR_SIM_CARD_CORP_PUK;
        }
        else
        {
            LogError( ( "Unknown SIM Lock State %s", pToken ) );
        }
    }

    return tempState;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetSimLockStatus( CellularContext_t * pContext,
                                                               const CellularATCommandResponse_t * pAtResp,
                                                               void * pData,
                                                               uint16_t dataLen )
{
    char * pToken = NULL, * pInputStr = NULL;
    CellularSimCardLockState_t * pSimLockState = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) ||
             ( pData == NULL ) || ( dataLen != sizeof( CellularSimCardLockState_t ) ) )
    {
        LogError( ( " Get SIM lock State: Response data is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputStr = pAtResp->pItm->pLine;
        pSimLockState = ( CellularSimCardLockState_t * ) pData;

        if( strlen( pInputStr ) == 0U )
        {
            LogError( ( "Get SIM lock State: Input data is invalid" ) );
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    if( pktStatus == CELLULAR_PKT_STATUS_OK )
    {
        atCoreStatus = Cellular_ATRemovePrefix( &pInputStr );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveLeadingWhiteSpaces( &pInputStr );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveTrailingWhiteSpaces( pInputStr );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputStr, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            LogDebug( ( "SIM Lock State: %s", pToken ) );
            *pSimLockState = _getSimLockState( pToken );
        }

        if( atCoreStatus != CELLULAR_AT_SUCCESS )
        {
            pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
        }
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parsePdnStatusContextId( char * pToken,
                                                  CellularPdnStatus_t * pPdnStatusBuffers )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( ( tempValue >= ( int32_t ) CELLULAR_PDN_CONTEXT_ID_MIN ) &&
            ( tempValue <= ( int32_t ) CELLULAR_PDN_CONTEXT_ID_MAX ) )
        {
            pPdnStatusBuffers->contextId = ( uint8_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in Processing Context Id. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parsePdnStatusContextState( char * pToken,
                                                     CellularPdnStatus_t * pPdnStatusBuffers )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        // TODO (MV): Why is this state not checked more closely (according to documentation only 0 = Deactivated and 1 = Activated and all other valuds invalid
        if( ( tempValue >= 0 ) &&
            ( tempValue <= ( int32_t ) UINT8_MAX ) )
        {
            pPdnStatusBuffers->state = ( uint8_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing PDN Status Buffer state. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parsePdnStatusContextType( char * pToken,
                                                    CellularPdnStatus_t * pPdnStatusBuffers )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        // TODO (MV): IPV4V6 value (3) not supported, should have better validation here
        if( ( tempValue >= 0 ) && ( tempValue < ( int32_t ) CELLULAR_PDN_CONTEXT_TYPE_MAX ) )
        {
            /* Variable "tempValue" is ensured that it is valid and within
             * a valid range. Hence, assigning the value of the variable to
             * pdnContextType with a enum cast. */
            /* coverity[misra_c_2012_rule_10_5_violation] */
            pPdnStatusBuffers->pdnContextType = ( CellularPdnContextType_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing PDN State Buffer Status. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t getPdnStatusParseToken( char * pToken,
                                                 uint8_t tokenIndex,
                                                 CellularPdnStatus_t * pPdnStatusBuffers )
{
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    switch( tokenIndex )
    {
        case ( CELLULAR_PDN_STATUS_POS_CONTEXT_ID ):
            LogDebug( ( "Context Id: %s", pToken ) );
            atCoreStatus = parsePdnStatusContextId( pToken, pPdnStatusBuffers );
            break;

        case ( CELLULAR_PDN_STATUS_POS_CONTEXT_STATE ):
            LogDebug( ( "Context State: %s", pToken ) );
            atCoreStatus = parsePdnStatusContextState( pToken, pPdnStatusBuffers );
            break;

        case ( CELLULAR_PDN_STATUS_POS_CONTEXT_TYPE ):
            LogDebug( ( "Context Type: %s", pToken ) );
            atCoreStatus = parsePdnStatusContextType( pToken, pPdnStatusBuffers );
            break;

        case ( CELLULAR_PDN_STATUS_POS_IP_ADDRESS ):
            LogDebug( ( "IP address: %s", pToken ) );
            ( void ) strncpy( pPdnStatusBuffers->ipAddress.ipAddress,
                              pToken, CELLULAR_IP_ADDRESS_MAX_SIZE + 1U );
            pPdnStatusBuffers->ipAddress.ipAddress[CELLULAR_IP_ADDRESS_MAX_SIZE] = '\0';    // guarantee null terminated

            if( pPdnStatusBuffers->pdnContextType == CELLULAR_PDN_CONTEXT_IPV4 )
            {
                pPdnStatusBuffers->ipAddress.ipAddressType = CELLULAR_IP_ADDRESS_V4;
            }
            else if( pPdnStatusBuffers->pdnContextType == CELLULAR_PDN_CONTEXT_IPV6 )
            {
                pPdnStatusBuffers->ipAddress.ipAddressType = CELLULAR_IP_ADDRESS_V6;
            }
            else
            {
                LogError( ( "Unknown pdnContextType %d", pPdnStatusBuffers->pdnContextType ) );
                atCoreStatus = CELLULAR_AT_ERROR;
            }
            break;

        default:
            LogError( ( "Unknown token in getPdnStatusParseToken %s %d",
                        pToken, tokenIndex ) );
            atCoreStatus = CELLULAR_AT_ERROR;
            break;
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t getPdnStatusParseLine( char * pRespLine,
                                                CellularPdnStatus_t * pPdnStatusBuffer )
{
    char * pToken = NULL;
    char * pLocalRespLine = pRespLine;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    uint8_t tokenIndex = 0;

    atCoreStatus = Cellular_ATRemovePrefix( &pLocalRespLine );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pLocalRespLine );
    }

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        atCoreStatus = Cellular_ATGetNextTok( &pLocalRespLine, &pToken );
    }

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        tokenIndex = 0;

        while( ( pToken != NULL ) && ( atCoreStatus == CELLULAR_AT_SUCCESS ) )
        {
            atCoreStatus = getPdnStatusParseToken( pToken, tokenIndex, pPdnStatusBuffer );

            if( atCoreStatus != CELLULAR_AT_SUCCESS )
            {
                LogInfo( ( "getPdnStatusParseToken %s index %d failed", pToken, tokenIndex ) );
            }

            tokenIndex++;

            if( Cellular_ATGetNextTok( &pLocalRespLine, &pToken ) != CELLULAR_AT_SUCCESS )
            {
                break;
            }
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetPdnStatus( CellularContext_t * pContext,
                                                           const CellularATCommandResponse_t * pAtResp,
                                                           void * pData,
                                                           uint16_t dataLen )
{
    char * pRespLine = NULL;
    CellularPdnStatus_t * pPdnStatusBuffers = ( CellularPdnStatus_t * ) pData;
    uint8_t numStatusBuffers = ( uint8_t ) dataLen;
    uint8_t numRemainingBuffers = numStatusBuffers;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    const CellularATCommandLine_t * pCommandItem = NULL;

    if( pContext == NULL )
    {
        LogError( ( "GetPdnStatus: invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( pAtResp == NULL )
    {
        LogError( ( "GetPdnStatus: Response is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pPdnStatusBuffers == NULL ) || ( numStatusBuffers < 1U ) )
    {
        LogError( ( "GetPdnStatus: PDN Status bad parameters" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "GetPdnStatus: no activated PDN" ) );
        pPdnStatusBuffers[ 0 ].contextId = INVALID_PDN_INDEX;
        pktStatus = CELLULAR_PKT_STATUS_OK;
    }
    else
    {
        pCommandItem = pAtResp->pItm;

        while( ( numRemainingBuffers != 0U ) && ( pCommandItem != NULL ) )
        {
            pRespLine = pCommandItem->pLine;
            atCoreStatus = getPdnStatusParseLine( pRespLine, pPdnStatusBuffers );
            pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );

            if( pktStatus != CELLULAR_PKT_STATUS_OK )
            {
                LogError( ( "getPdnStatusParseLine parse %s failed", pRespLine ) );
                break;
            }

            pPdnStatusBuffers++;
            numRemainingBuffers--;
            pCommandItem = pCommandItem->pNext;
        }

        /* Mark first unused status buffer as invalid */
        if( ( pktStatus == CELLULAR_PKT_STATUS_OK ) &&
            ( numRemainingBuffers > 0 ) &&
            ( numRemainingBuffers < numStatusBuffers ) )
        {
            pPdnStatusBuffers[ numStatusBuffers - numRemainingBuffers - 1 ].contextId = INVALID_PDN_INDEX;
        }
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t buildSocketConnect( CellularSocketHandle_t socketHandle,
                                           char * pCmdBuf, size_t cmdBufLength )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    char protocol[ 15 ];
    int snprintfLength = -1;

    if( pCmdBuf == NULL )
    {
        LogError( ( "buildSocketConnect: Invalid command buffer" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        if( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP)
        {
            /* Form the AT command. */
            snprintfLength = snprintf( pCmdBuf, cmdBufLength,
                                       "%s%d,%d,%ld,\"%s\",%d,%d",
                                       "AT+QSSLOPEN=",
                                       socketHandle->contextId,
                                       socketHandle->sslContextId,
                                       socketHandle->socketId,
                                       socketHandle->remoteSocketAddress.ipAddress.ipAddress,
                                       socketHandle->remoteSocketAddress.port,
                                       socketHandle->dataMode);

            if (socketHandle->localPort != 0) {
                LogWarn( ( "buildSocketConnect: configured localPort ignored for SSL socket" ) );
            }
        }
        else
        {
            if (socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_TCP) {
                (void) strcpy(protocol, "TCP");
            } else {
                (void) strcpy(protocol, "UDP SERVICE");
            }

            /* Form the AT command. */
            snprintfLength = snprintf( pCmdBuf, CELLULAR_AT_CMD_MAX_SIZE,
                                       "%s%d,%ld,\"%s\",\"%s\",%d,%d,%d",
                                       "AT+QIOPEN=",
                                       socketHandle->contextId,
                                       socketHandle->socketId,
                                       protocol,
                                       socketHandle->remoteSocketAddress.ipAddress.ipAddress,
                                       socketHandle->remoteSocketAddress.port,
                                       socketHandle->localPort,
                                       socketHandle->dataMode);
        }

        if( ( snprintfLength <= 0) || ( snprintfLength >= cmdBufLength ) )
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t getDataFromResp( const CellularATCommandResponse_t * pAtResp,
                                          const _socketDataRecv_t * pDataRecv,
                                          uint32_t outBufSize )
{
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    const char * pInputLine = NULL;
    uint32_t dataLenToCopy = 0;

    /* Check if the received data size is greater than the output buffer size. */
    if( *pDataRecv->pDataLen > outBufSize )
    {
        LogError( ( "Data is truncated, received data length %d, out buffer size %d",
                    *pDataRecv->pDataLen, outBufSize ) );
        dataLenToCopy = outBufSize;
        *pDataRecv->pDataLen = outBufSize;
        // TODO (MV): MAJOR, How is this not an error condition? the socket data stream is now compromised!
    }
    else
    {
        dataLenToCopy = *pDataRecv->pDataLen;
    }

    /* Data is stored in the next intermediate response. */
    if( pAtResp->pItm->pNext != NULL )
    {
        pInputLine = pAtResp->pItm->pNext->pLine;

        if( ( pInputLine != NULL ) && ( dataLenToCopy > 0U ) )
        {
            /* Copy the data to the out buffer. */
            ( void ) memcpy( ( void * ) pDataRecv->pData, ( const void * ) pInputLine, dataLenToCopy );
        }
        else
        {
            LogError( ( "Receive Data: Data pointer NULL" ) );
            atCoreStatus = CELLULAR_AT_BAD_PARAMETER;
        }
    }
    else if( *pDataRecv->pDataLen == 0U )
    {
        /* Receive command success but no data. */
        LogDebug( ( "Receive Data: no data" ) );
    }
    else
    {
        LogError( ( "Receive Data: Intermediate response empty" ) );
        atCoreStatus = CELLULAR_AT_BAD_PARAMETER;
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncData( CellularContext_t * pContext,
                                                   const CellularATCommandResponse_t * pAtResp,
                                                   void * pData,
                                                   uint16_t dataLen )
{
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char * pInputLine = NULL, * pToken = NULL;
    const _socketDataRecv_t * pDataRecv = ( _socketDataRecv_t * ) pData;
    int32_t tempValue = 0;

    if( pContext == NULL )
    {
        LogError( ( "Receive Data: invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "Receive Data: response is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pDataRecv == NULL ) || ( pDataRecv->pData == NULL ) || ( pDataRecv->pDataLen == NULL ) )
    {
        LogError( ( "Receive Data: Bad param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        /* parse the datalen. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= ( int32_t ) 0 ) && ( tempValue < ( ( int32_t ) CELLULAR_MAX_RECV_DATA_LEN + 1 ) ) )
                {
                    *pDataRecv->pDataLen = ( uint32_t ) tempValue;
                }
                else
                {
                    LogError( ( "Error in Data Length Processing: No valid digit found. Token %s", pToken ) );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        /* Process the data buffer. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = getDataFromResp( pAtResp, pDataRecv, dataLen );
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseQpsmsMode( char * pToken,
                                         CellularPsmSettings_t * pPsmSettings )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( ( tempValue >= 0 ) && ( tempValue <= ( int32_t ) UINT8_MAX ) )  // TODO (MV): Only 0 and 1 are valid
        {
            pPsmSettings->mode = ( uint8_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing mode. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseQpsmsRau( char * pToken,
                                        CellularPsmSettings_t * pPsmSettings )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 2, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( tempValue >= 0 )
        {
            pPsmSettings->periodicRauValue = ( uint32_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing Periodic Processing RAU value. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseQpsmsRdyTimer( char * pToken,
                                             CellularPsmSettings_t * pPsmSettings )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 2, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( tempValue >= 0 )
        {
            pPsmSettings->gprsReadyTimer = ( uint32_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing Periodic Processing GPRS Ready Timer value. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseQpsmsTau( char * pToken,
                                        CellularPsmSettings_t * pPsmSettings )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 2, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( tempValue >= 0 )
        {
            pPsmSettings->periodicTauValue = ( uint32_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing Periodic TAU value value. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseQpsmsActiveTime( char * pToken,
                                               CellularPsmSettings_t * pPsmSettings )
{
    int32_t tempValue = 0;
    CellularATError_t atCoreStatus = Cellular_ATStrtoi( pToken, 2, &tempValue );

    if( atCoreStatus == CELLULAR_AT_SUCCESS )
    {
        if( tempValue >= 0 )
        {
            pPsmSettings->activeTimeValue = ( uint32_t ) tempValue;
        }
        else
        {
            LogError( ( "Error in processing Periodic Processing Active time value. Token %s", pToken ) );
            atCoreStatus = CELLULAR_AT_ERROR;
        }
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularATError_t parseGetPsmToken( char * pToken,
                                           uint8_t tokenIndex,
                                           CellularPsmSettings_t * pPsmSettings )
{
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    switch( tokenIndex )
    {
        case QPSMS_POS_MODE:
            atCoreStatus = parseQpsmsMode( pToken, pPsmSettings );
            break;

        case QPSMS_POS_RAU:
            atCoreStatus = parseQpsmsRau( pToken, pPsmSettings );
            break;

        case QPSMS_POS_RDY_TIMER:
            atCoreStatus = parseQpsmsRdyTimer( pToken, pPsmSettings );
            break;

        case QPSMS_POS_TAU:
            atCoreStatus = parseQpsmsTau( pToken, pPsmSettings );
            break;

        case QPSMS_POS_ACTIVE_TIME:
            atCoreStatus = parseQpsmsActiveTime( pToken, pPsmSettings );
            break;

        default:
            LogDebug( ( "Unknown Parameter Position in AT+QPSMS Response" ) );
            atCoreStatus = CELLULAR_AT_ERROR;
            break;
    }

    return atCoreStatus;
}

/*-----------------------------------------------------------*/

static CellularRat_t convertRatPriority( char * pRatString )
{
    CellularRat_t retRat = CELLULAR_RAT_INVALID;

    if( strncmp( pRatString, "01", RAT_PRIORITY_STRING_LENGTH ) == 0 )
    {
        retRat = CELLULAR_RAT_GSM;
    }
    else if( strncmp( pRatString, "02", RAT_PRIORITY_STRING_LENGTH ) == 0 )
    {
        retRat = CELLULAR_RAT_LTE;
    }
    else if( strncmp( pRatString, "03", RAT_PRIORITY_STRING_LENGTH ) == 0 )
    {
        retRat = CELLULAR_RAT_NBIOT;
    }
    else
    {
        LogDebug( ( "Invalid RAT string %s", pRatString ) );
    }

    return retRat;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetRatPriority( CellularContext_t * pContext,
                                                             const CellularATCommandResponse_t * pAtResp,
                                                             void * pData,
                                                             uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularRat_t * pRatPriorities = NULL;
    char pTempString[ RAT_PRIORITY_STRING_LENGTH + 1 ] = { "\0" }; /* The return RAT has two chars plus NULL char. */
    uint32_t ratIndex = 0;
    uint32_t maxRatPriorityLength = ( dataLen > RAT_PRIORITY_LIST_LENGTH ? RAT_PRIORITY_LIST_LENGTH : dataLen );

    if( pContext == NULL )
    {
        LogError( ( "GetRatPriority: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) || ( dataLen == 0U ) )
    {
        LogError( ( "GetRatPriority: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pRatPriorities = ( CellularRat_t * ) pData;

        /* Response string +QCFG:"nwscanseq",020301 => pToken : +QCFG:"nwscanseq", pInputLine : 020301. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        /* Response string 020301 => pToken : 020301, pInputLine : NULL. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        // TODO (MV): Is this actually guaranteed to always be 3 RATs, what if automatic is used or GSM and NB-IoT are non-existent?
        // TODO (MV): Change this to support up to 3 RATs and set all unused entries to invalid and support default of 00 [Automatic (eMTC → NB-IoT)]

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            if( strlen( pToken ) != ( RAT_PRIORITY_STRING_LENGTH * RAT_PRIORITY_LIST_LENGTH ) )
            {
                atCoreStatus = CELLULAR_AT_ERROR;
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            memset( pRatPriorities, CELLULAR_RAT_INVALID, dataLen );

            for( ratIndex = 0; ratIndex < maxRatPriorityLength; ratIndex++ )
            {
                memcpy( pTempString, &pToken[ ratIndex * RAT_PRIORITY_STRING_LENGTH ], RAT_PRIORITY_STRING_LENGTH );
                pRatPriorities[ ratIndex ] = convertRatPriority( pTempString );
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetPsmSettings( CellularContext_t * pContext,
                                                             const CellularATCommandResponse_t * pAtResp,
                                                             void * pData,
                                                             uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    uint8_t tokenIndex = 0;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularPsmSettings_t * pPsmSettings = NULL;

    if( pContext == NULL )
    {
        LogError( ( "GetPsmSettings: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) || ( dataLen != sizeof( CellularPsmSettings_t ) ) )
    {
        LogError( ( "GetPsmSettings: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pPsmSettings = ( CellularPsmSettings_t * ) pData;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pInputLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            tokenIndex = 0;

            while( pToken != NULL )
            {
                if( tokenIndex == 0 )
                {
                    atCoreStatus = parseGetPsmToken( pToken, tokenIndex, pPsmSettings );
                }
                else
                {
                    parseGetPsmToken( pToken, tokenIndex, pPsmSettings );
                }

                tokenIndex++;

                if( *pInputLine == ',' )
                {
                    *pInputLine = '\0';
                    pToken = pInputLine;
                    *pToken = '\0';
                    pInputLine = &pInputLine[ 1 ];
                }
                else if( Cellular_ATGetNextTok( &pInputLine, &pToken ) != CELLULAR_AT_SUCCESS )
                {
                    break;
                }
                else
                {
                    /* Empty Else MISRA 15.7 */
                }
            }
        }

        LogDebug( ( "PSM setting: mode: %d, RAU: %d, RDY_Timer: %d, TAU: %d, Active_time: %d",
                    pPsmSettings->mode,
                    pPsmSettings->periodicRauValue,
                    pPsmSettings->gprsReadyTimer,
                    pPsmSettings->periodicTauValue,
                    pPsmSettings->activeTimeValue ) );
        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetPsmConfigSettings( CellularContext_t * pContext,
                                                                   const CellularATCommandResponse_t * pAtResp,
                                                                   void * pData,
                                                                   uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    uint8_t tokenIndex = 0;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularPsmConfigSettings_t * pPsmConfigSettings = NULL;
    uint32_t tempUValue = 0;
    int32_t tempValue = 0;

    if( pContext == NULL )
    {
        LogError( ( "GetPsmConfigSettings: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) || ( dataLen != sizeof( CellularPsmConfigSettings_t ) ) )
    {
        LogError( ( "GetPsmConfigSettings: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pPsmConfigSettings = ( CellularPsmConfigSettings_t * ) pData;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoui(pToken, 10, &tempUValue);

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                pPsmConfigSettings->threshold = tempUValue;     // TODO (MV): Only value between 20 and UINT32_MAX
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi(pToken, 10, &tempValue);

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= 0 ) && (tempValue <= ( int32_t ) UINT8_MAX ) )  // TODO (MV): Only 0 - 15 are valid
                {
                    pPsmConfigSettings->psmVersion = ( uint8_t ) tempValue;
                }
                else
                {
                    LogError( ( "Error in processing PSM version. Token %s", pToken ) );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        LogDebug( ( "PSM config settings: threshold: %lu, version: %d",
                pPsmConfigSettings->threshold,
                pPsmConfigSettings->psmVersion ) );
        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularPktStatus_t socketRecvDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t lineLength,
                                                 char ** ppDataStart,
                                                 uint32_t * pDataLength )
{
    char * pDataStart = NULL;
    uint32_t prefixLineLength = 0U;
    int32_t tempValue = 0;
    CellularATError_t atResult = CELLULAR_AT_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    uint32_t i = 0;
    char pLocalLine[ MAX_QIRD_STRING_PREFIX_STRING + 1 ] = "\0";
    uint32_t localLineLength = MAX_QIRD_STRING_PREFIX_STRING > lineLength ? lineLength : MAX_QIRD_STRING_PREFIX_STRING;

    ( void ) pCallbackContext;

    if( ( pLine == NULL ) || ( ppDataStart == NULL ) || ( pDataLength == NULL ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        /* Check if the message is a data response. */
        if( strncmp( pLine, SOCKET_DATA_PREFIX_STRING, SOCKET_DATA_PREFIX_STRING_LENGTH ) == 0 )
        {
            (void) strncpy( pLocalLine, pLine, MAX_QIRD_STRING_PREFIX_STRING );
            pLocalLine[ MAX_QIRD_STRING_PREFIX_STRING ] = '\0';
            pDataStart = pLocalLine;

            /* Add a '\0' char at the end of the line. */
            for( i = 0; i < localLineLength; i++ )
            {
                if( ( pDataStart[ i ] == '\r' ) || ( pDataStart[ i ] == '\n' ) )    // TODO (MV): Strangeness here were line considered ended if only '\n' but later expect "\r\n"
                {
                    pDataStart[ i ] = '\0';
                    prefixLineLength = i;
                    break;
                }
            }

            if( i == localLineLength )
            {
                LogDebug( ( "Data prefix invalid line : %s", pLocalLine ) );
                pDataStart = NULL;
            }
        }

        if( pDataStart != NULL )
        {
            atResult = Cellular_ATStrtoi(&pDataStart[ SOCKET_DATA_PREFIX_STRING_LENGTH ], 10, &tempValue );

            if( ( atResult == CELLULAR_AT_SUCCESS ) && ( tempValue >= 0 ) &&
                ( tempValue <= ( int32_t ) CELLULAR_MAX_RECV_DATA_LEN ) )
            {
                if( ( prefixLineLength + DATA_PREFIX_STRING_CHANGELINE_LENGTH ) > lineLength )
                {
                    /* More data is required. */
                    *pDataLength = 0;
                    pDataStart = NULL;
                    pktStatus = CELLULAR_PKT_STATUS_SIZE_MISMATCH;
                }
                else
                {
                    pDataStart = &pLine[ prefixLineLength ];
                    pDataStart[ 0 ] = '\0';
                    pDataStart = &pDataStart[ DATA_PREFIX_STRING_CHANGELINE_LENGTH ];
                    *pDataLength = ( uint32_t ) tempValue;
                }

                LogDebug( ( "DataLength %p at pktIo = %d", pDataStart, *pDataLength ) );
            }
            else
            {
                *pDataLength = 0;
                pDataStart = NULL;
                LogError( ( "Data response received with wrong size" ) );
            }
        }

        *ppDataStart = pDataStart;
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularPktStatus_t sslSocketRecvDataPrefix( void * pCallbackContext,
                                                    char * pLine,
                                                    uint32_t lineLength,
                                                    char ** ppDataStart,
                                                    uint32_t * pDataLength )
{
    char * pDataStart = NULL;
    uint32_t prefixLineLength = 0U;
    int32_t tempValue = 0;
    CellularATError_t atResult = CELLULAR_AT_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    uint32_t i = 0;
    char pLocalLine[ MAX_QSSLRECV_STRING_PREFIX_STRING + 1 ] = "\0";
    uint32_t localLineLength = MAX_QSSLRECV_STRING_PREFIX_STRING > lineLength ? lineLength : MAX_QSSLRECV_STRING_PREFIX_STRING;

    ( void ) pCallbackContext;

    if( ( pLine == NULL ) || ( ppDataStart == NULL ) || ( pDataLength == NULL ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        /* Check if the message is a data response. */
        if( strncmp( pLine, SSL_SOCKET_DATA_PREFIX_STRING, SSL_SOCKET_DATA_PREFIX_STRING_LENGTH ) == 0 )
        {
            (void) strncpy( pLocalLine, pLine, MAX_QSSLRECV_STRING_PREFIX_STRING );
            pLocalLine[ MAX_QSSLRECV_STRING_PREFIX_STRING ] = '\0';
            pDataStart = pLocalLine;

            /* Add a '\0' char at the end of the line. */
            for( i = 0; i < localLineLength; i++ )
            {
                if( ( pDataStart[ i ] == '\r' ) || ( pDataStart[ i ] == '\n' ) )    // TODO (MV): Strangeness here were line considered ended if only '\n' but later expect "\r\n"
                {
                    pDataStart[ i ] = '\0';
                    prefixLineLength = i;
                    break;
                }
            }

            if( i == localLineLength )
            {
                LogDebug( ( "Data prefix invalid line : %s", pLocalLine ) );
                pDataStart = NULL;
            }
        }

        if( pDataStart != NULL )
        {
            atResult = Cellular_ATStrtoi(&pDataStart[ SSL_SOCKET_DATA_PREFIX_STRING_LENGTH ], 10, &tempValue );

            if( ( atResult == CELLULAR_AT_SUCCESS ) && ( tempValue >= 0 ) &&
                ( tempValue <= ( int32_t ) CELLULAR_MAX_RECV_DATA_LEN ) )
            {
                if( ( prefixLineLength + DATA_PREFIX_STRING_CHANGELINE_LENGTH ) > lineLength )
                {
                    /* More data is required. */
                    *pDataLength = 0;
                    pDataStart = NULL;
                    pktStatus = CELLULAR_PKT_STATUS_SIZE_MISMATCH;
                }
                else
                {
                    pDataStart = &pLine[ prefixLineLength ];
                    pDataStart[ 0 ] = '\0';
                    pDataStart = &pDataStart[ DATA_PREFIX_STRING_CHANGELINE_LENGTH ];
                    *pDataLength = ( uint32_t ) tempValue;
                }

                LogDebug( ( "DataLength %p at pktIo = %d", pDataStart, *pDataLength ) );
            }
            else
            {
                *pDataLength = 0;
                pDataStart = NULL;
                LogError( ( "Data response received with wrong size" ) );
            }
        }

        *ppDataStart = pDataStart;
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

static CellularError_t storeAccessModeAndAddress( CellularContext_t * pContext,
                                                  CellularSocketHandle_t socketHandle,
                                                  CellularSocketAccessMode_t dataAccessMode,
                                                  const CellularSocketAddress_t * pRemoteSocketAddress )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ( pRemoteSocketAddress == NULL ) || ( socketHandle == NULL ) )
    {
        LogError( ( "storeAccessModeAndAddress: Invalid socket address" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( socketHandle->socketState != SOCKETSTATE_ALLOCATED )
    {
        LogError( ( "storeAccessModeAndAddress, bad socket state %d",
                    socketHandle->socketState ) );
        cellularStatus = CELLULAR_INTERNAL_FAILURE;
    }
    else if( dataAccessMode != CELLULAR_ACCESSMODE_BUFFER )
    {
        LogError( ( "storeAccessModeAndAddress, Access mode not supported %d",
                    dataAccessMode ) );
        cellularStatus = CELLULAR_UNSUPPORTED;
    }
    else
    {
        socketHandle->remoteSocketAddress.port = pRemoteSocketAddress->port;
        socketHandle->dataMode = dataAccessMode;
        socketHandle->remoteSocketAddress.ipAddress.ipAddressType =
            pRemoteSocketAddress->ipAddress.ipAddressType;
        ( void ) strncpy( socketHandle->remoteSocketAddress.ipAddress.ipAddress,
                          pRemoteSocketAddress->ipAddress.ipAddress,
                          CELLULAR_IP_ADDRESS_MAX_SIZE + 1U );
    }

    return cellularStatus;
}


/*-----------------------------------------------------------*/

static CellularError_t registerDnsEventCallback( cellularModuleContext_t * pModuleContext,
                                                 CellularDnsResultEventCallback_t dnsEventCallback,
                                                 char * pDnsUsrData )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    if( pModuleContext == NULL )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else
    {
        pModuleContext->dnsEventCallback = dnsEventCallback;
        pModuleContext->pDnsUsrData = pDnsUsrData;
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static void _dnsResultCallback( cellularModuleContext_t * pModuleContext,
                                char * pDnsResult,
                                char * pDnsUsrData )
{
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pToken = NULL, * pDnsResultStr = pDnsResult;
    int32_t dnsResultNumber = 0;
    cellularDnsQueryResult_t dnsQueryResult = CELLULAR_DNS_QUERY_UNKNOWN;

    if( pModuleContext != NULL )
    {
        if( pModuleContext->dnsResultNumber == ( uint8_t ) 0 )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pDnsResultStr, &pToken );    // TODO (MV): Ignoring result code? This is so stupid, fix!
                                                                                //            Should be sending error code if result code is not 0 (success).
                                                                                //            Otherwise have a 60s timeout for no reason, so, so stupid! What a waste of power

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                atCoreStatus = Cellular_ATGetNextTok( &pDnsResultStr, &pToken );
            }

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                atCoreStatus = Cellular_ATStrtoi( pToken, 10, &dnsResultNumber );

                if( ( atCoreStatus == CELLULAR_AT_SUCCESS ) && ( dnsResultNumber >= 0 ) &&
                    ( dnsResultNumber <= ( int32_t ) UINT8_MAX ) )
                {
                    pModuleContext->dnsResultNumber = ( uint8_t ) dnsResultNumber;
                }
                else
                {
                    LogDebug( ( "_dnsResultCallback convert string failed %s", pToken ) );
                }
            }
        }
        else if( ( pModuleContext->dnsIndex < pModuleContext->dnsResultNumber ) && ( pDnsResultStr != NULL ) )
        {
            pModuleContext->dnsIndex = pModuleContext->dnsIndex + ( uint8_t ) 1;

            ( void ) strncpy( pDnsUsrData, pDnsResultStr, CELLULAR_IP_ADDRESS_MAX_SIZE );
            ( void ) registerDnsEventCallback( pModuleContext, NULL, NULL );
            dnsQueryResult = CELLULAR_DNS_QUERY_SUCCESS;

            if( xQueueSend( pModuleContext->pktDnsQueue, &dnsQueryResult, ( TickType_t ) 0 ) != pdPASS )
            {
                LogDebug( ( "_dnsResultCallback sends pktDnsQueue fail" ) );
            }
        }
        else
        {
            LogDebug( ( "_dnsResultCallback spurious DNS response" ) );
        }
    }
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SetRatPriority( CellularHandle_t cellularHandle,
                                         const CellularRat_t * pRatPriorities,
                                         uint8_t ratPrioritiesLength )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    uint8_t i = 0;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetRatPriority =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( pRatPriorities == NULL ) || ( ratPrioritiesLength == 0U ) ||
             ( ratPrioritiesLength > ( uint8_t ) CELLULAR_MAX_RAT_PRIORITY_COUNT ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /** Using AT+QCFG="nwscanseq",<scanseq>,<effect> to set the RAT priorities while searching.
         * <scanseq> can take value 01 for GSM, 02 for eMTC / LTE-M and 03 for NB-IoT.
         * <effect> can take value 0 for take effect after reboot and 1 for take effect immediately.
         *  If < effect > is omitted, RAT priority takes effect immediately.
         *  e.g. AT+QCFG="nwscanseq",020301,1 for decreasing RAT Priorities CAT M1, CAT NB1, GSM to take effect immediately. */
        ( void ) strcpy( cmdBuf, "AT+QCFG=\"nwscanseq\"," );

        while( i < ratPrioritiesLength )
        {
            if( pRatPriorities[ i ] == CELLULAR_RAT_GSM )
            {
                ( void ) strcat( cmdBuf, "01" );
            }
            else if( pRatPriorities[ i ] == CELLULAR_RAT_CATM1 || pRatPriorities[ i ] == CELLULAR_RAT_LTE )
            {
                ( void ) strcat( cmdBuf, "02" );
            }
            else if( pRatPriorities[ i ] == CELLULAR_RAT_NBIOT )
            {
                ( void ) strcat( cmdBuf, "03" );
            }
            else
            {
                cellularStatus = CELLULAR_BAD_PARAMETER;
                break;
            }

            i++;
        }
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetRatPriority );
        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetRatPriority( CellularHandle_t cellularHandle,
                                         CellularRat_t * pRatPriorities,
                                         uint8_t ratPrioritiesLength,
                                         uint8_t * pReceiveRatPrioritesLength )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    uint32_t ratIndex = 0;

    CellularAtReq_t atReqSetRatPriority =
    {
        "AT+QCFG=\"nwscanseq\"",
        CELLULAR_AT_WITH_PREFIX,
        "+QCFG",
        _Cellular_RecvFuncGetRatPriority,
        pRatPriorities,
        ( uint16_t ) ratPrioritiesLength,
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( pRatPriorities == NULL ) || ( ratPrioritiesLength == 0U ) ||
             ( ratPrioritiesLength > ( uint8_t ) CELLULAR_MAX_RAT_PRIORITY_COUNT ) ||
             ( pReceiveRatPrioritesLength == NULL ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetRatPriority );

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            for( ratIndex = 0; ratIndex < ratPrioritiesLength; ratIndex++ )
            {
                if( pRatPriorities[ ratIndex ] == CELLULAR_RAT_INVALID )
                {
                    break;
                }
            }

            *pReceiveRatPrioritesLength = ratIndex;
        }

        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SetDns( CellularHandle_t cellularHandle,
                                 uint8_t contextId,
                                 const char * pPrimaryDnsServerAddress,
                                 const char * pSecondaryDnsServerAddress )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    int snprintfLength = -1;
    CellularAtReq_t atReqSetDns =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pPrimaryDnsServerAddress == NULL )
    {
        LogError( ( "Cellular_SetDns: Invalid parameter" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        cellularStatus = _Cellular_IsValidPdn( contextId );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Form the AT command. */
        if( pSecondaryDnsServerAddress != NULL )
        {
            snprintfLength = snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s%d,\"%s\",\"%s\"",
                                       "AT+QIDNSCFG=", contextId,
                                       pPrimaryDnsServerAddress, pSecondaryDnsServerAddress );
        }
        else
        {
            snprintfLength = snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s%d,\"%s\"",
                                       "AT+QIDNSCFG=", contextId, pPrimaryDnsServerAddress );
        }

        if( ( snprintfLength <= 0 ) || ( snprintfLength >= CELLULAR_AT_CMD_MAX_SIZE ) )
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetDns );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SetDns: couldn't set the DNS, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetPsmSettings( CellularHandle_t cellularHandle,
                                         CellularPsmSettings_t * pPsmSettings )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetPsm =
    {
        "AT+QPSMS?",
        CELLULAR_AT_WITH_PREFIX,
        "+QPSMS",
        _Cellular_RecvFuncGetPsmSettings,
        pPsmSettings,
        sizeof( CellularPsmSettings_t ),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pPsmSettings == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* initialize the data. */
        ( void ) memset( pPsmSettings, 0, sizeof( CellularPsmSettings_t ) );
        pPsmSettings->mode = 0xFF;

        /* we should always query the PSMsettings from the network. */
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetPsm );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetPsmSettings: couldn't retrieve PSM settings" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetPsmConfigSettings( CellularHandle_t cellularHandle,
                                               CellularPsmConfigSettings_t * pPsmConfigSettings )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetPsm =
    {
        "AT+QPSMCFG?",
        CELLULAR_AT_WITH_PREFIX,
        "+QPSMCFG",
        _Cellular_RecvFuncGetPsmConfigSettings,
        pPsmConfigSettings,
        sizeof( CellularPsmConfigSettings_t ),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pPsmConfigSettings == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* initialize the data. */
        ( void ) memset( pPsmConfigSettings, 0, sizeof( CellularPsmConfigSettings_t ) );
        pPsmConfigSettings->psmVersion = 0xFF;

        /* we should always query the PSMsettings from the network. */
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetPsm );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetPsmSettings: couldn't retrieve PSM settings" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static uint32_t appendBinaryPattern( char * cmdBuf,
                                     uint32_t cmdLen,
                                     uint32_t value,
                                     bool endOfString )
{
    uint32_t retLen = 0;

    if( cmdBuf != NULL )
    {
        if( value != 0U )
        {
            /* The return value of snprintf is not used.
             * The max length of the string is fixed and checked offline. */
            /* coverity[misra_c_2012_rule_21_6_violation]. */
            ( void ) snprintf( cmdBuf, cmdLen, "\"" PRINTF_BINARY_PATTERN_INT8 "\"%c",
                               PRINTF_BYTE_TO_BINARY_INT8( value ), endOfString ? '\0' : ',' );
        }
        else
        {
            /* The return value of snprintf is not used.
             * The max length of the string is fixed and checked offline. */
            /* coverity[misra_c_2012_rule_21_6_violation]. */
            ( void ) snprintf( cmdBuf, cmdLen, "%c", endOfString ? '\0' : ',' );
        }

        retLen = strlen( cmdBuf );
    }

    return retLen;
}

/*-----------------------------------------------------------*/

static CellularPktStatus_t socketSendDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t * pBytesRead )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;

    if( ( pLine == NULL ) || ( pBytesRead == NULL ) )
    {
        LogError( ( "socketSendDataPrefix: pLine is invalid or pBytesRead is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( pCallbackContext != NULL )
    {
        LogError( ( "socketSendDataPrefix: pCallbackContext is not NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( *pBytesRead != 2U )        // TODO (MV): Is this actually 2 characters, expecting just one '>'
    {
        LogDebug( ( "socketSendDataPrefix: pBytesRead %u %s is not 1", *pBytesRead, pLine ) );
    }
    else
    {
        /* After the data prefix, there should not be any data in stream.
         * Cellular common processes AT command in lines. Add a '\0' after '>'. */
        if( strcmp( pLine, "> " ) == 0 )        // TODO (MV): MAJOR, Is this actually 2 characters, expecting just one '>'
        {
            pLine[ 1 ] = '\n';
        }
    }

    return pktStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SetPsmSettings( CellularHandle_t cellularHandle,
                                         const CellularPsmSettings_t * pPsmSettings )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    uint32_t cmdBufLen = 0;
    CellularAtReq_t atReqSetPsm =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pPsmSettings == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Form the AT command. */

        if (pPsmSettings->periodicRauValue != 0) {
            LogWarn( ( "Cellular_SetPsmSettings: periodicRauValue not supported" ) );
        }

        if (pPsmSettings->gprsReadyTimer != 0) {
            LogWarn( ( "Cellular_SetPsmSettings: gprsReadyTimer not supported" ) );
        }

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "AT+QPSMS=%d", pPsmSettings->mode );
        cmdBufLen = strlen( cmdBuf );
        if (pPsmSettings->periodicTauValue != 0 || pPsmSettings->activeTimeValue != 0) {
            (void) strcat(cmdBuf, ",");     // TODO (MV): Improve robustness of this
            cmdBufLen++;
            cmdBufLen += appendBinaryPattern(&cmdBuf[cmdBufLen], (CELLULAR_AT_CMD_MAX_SIZE - cmdBufLen), 0, false);    // NOTE: BG770 doesn't support this parameter
            cmdBufLen += appendBinaryPattern(&cmdBuf[cmdBufLen], (CELLULAR_AT_CMD_MAX_SIZE - cmdBufLen), 0, false);    // NOTE: BG770 doesn't support this parameter
            cmdBufLen += appendBinaryPattern(&cmdBuf[cmdBufLen], (CELLULAR_AT_CMD_MAX_SIZE - cmdBufLen), pPsmSettings->periodicTauValue, false);
            cmdBufLen += appendBinaryPattern(&cmdBuf[cmdBufLen], (CELLULAR_AT_CMD_MAX_SIZE - cmdBufLen), pPsmSettings->activeTimeValue, true);
        }

        LogDebug( ( "PSM setting: %s ", cmdBuf ) );

        if( cmdBufLen < CELLULAR_AT_CMD_MAX_SIZE )
        {
            pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetPsm );

            if( pktStatus != CELLULAR_PKT_STATUS_OK )
            {
                LogError( ( "Cellular_SetPsmSettings: couldn't set PSM settings" ) );
                cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
            }
        }
        else
        {
            cellularStatus = CELLULAR_NO_MEMORY;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SetPsmConfigSettings( CellularHandle_t cellularHandle,
                                               const CellularPsmConfigSettings_t * pPsmConfigSettings )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetPsmConfig =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( pPsmConfigSettings == NULL )
             || ( ( pPsmConfigSettings->psmVersion & PSM_VERSION_BIT_MASK) != pPsmConfigSettings->psmVersion ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        int len = snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "AT+QPSMCFG=%lu,%d", pPsmConfigSettings->threshold, pPsmConfigSettings->psmVersion );

        if( len < CELLULAR_AT_CMD_MAX_SIZE )
        {
            LogDebug( ( "PSM config settings: %s ", cmdBuf ) );

            pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqSetPsmConfig );

            if( pktStatus != CELLULAR_PKT_STATUS_OK )
            {
                LogError( ( "Cellular_SetPsmSettings: couldn't set PSM settings" ) );
                cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
            }
        }
        else
        {
            cellularStatus = CELLULAR_NO_MEMORY;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_DeactivatePdn( CellularHandle_t cellularHandle,
                                        uint8_t contextId )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqDeactPdn =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    cellularStatus = _Cellular_IsValidPdn( contextId );

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Make sure the library is open. */
        cellularStatus = _Cellular_CheckLibraryStatus( pContext );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "%s%d", "AT+QIDEACT=", contextId );
        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqDeactPdn, PDN_DEACTIVATION_PACKET_REQ_TIMEOUT_MS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_DeactivatePdn: can't deactivate PDN, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_ActivatePdn( CellularHandle_t cellularHandle,
                                      uint8_t contextId )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };

    CellularAtReq_t atReqActPdn =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    cellularStatus = _Cellular_IsValidPdn( contextId );

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Make sure the library is open. */
        cellularStatus = _Cellular_CheckLibraryStatus( pContext );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "%s%d", "AT+QIACT=", contextId );
        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqActPdn, PDN_ACTIVATION_PACKET_REQ_TIMEOUT_MS );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_ActivatePdn: can't activate PDN, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SetPdnConfig( CellularHandle_t cellularHandle,
                                       uint8_t contextId,
                                       const CellularPdnConfig_t * pPdnConfig )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetPdn =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    if( pPdnConfig == NULL )
    {
        LogError( ( "Cellular_ATCommandRaw: Input parameter is NULL" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        cellularStatus = _Cellular_IsValidPdn( contextId );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Make sure the library is open. */
        cellularStatus = _Cellular_CheckLibraryStatus( pContext );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s%d,%d,\"%s\",\"%s\",\"%s\",%d",
                           "AT+QICSGP=",
                           contextId,
                           pPdnConfig->pdnContextType,
                           pPdnConfig->apnName,
                           pPdnConfig->username,
                           pPdnConfig->password,
                           pPdnConfig->pdnAuthType );
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSetPdn );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SetPdnConfig: can't set PDN, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetSignalInfo( CellularHandle_t cellularHandle,
                                        CellularSignalInfo_t * pSignalInfo )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularRat_t rat = CELLULAR_RAT_INVALID;
    CellularAtReq_t atReqQuerySignalInfo =
    {
        "AT+QCSQ",
        CELLULAR_AT_WITH_PREFIX,
        "+QCSQ",
        _Cellular_RecvFuncGetSignalInfo,
        pSignalInfo,
        sizeof( CellularSignalInfo_t ),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pSignalInfo == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        cellularStatus = _Cellular_GetCurrentRat( pContext, &rat );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqQuerySignalInfo );

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            /* If the convert failed, the API will return CELLULAR_INVALID_SIGNAL_BAR_VALUE in bars field. */
            ( void ) _Cellular_ComputeSignalBars( rat, pSignalInfo );
        }

        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
/* coverity[misra_c_2012_rule_8_13_violation] */
CellularError_t Cellular_SocketRecv( CellularHandle_t cellularHandle,
                                     CellularSocketHandle_t socketHandle,
                                     /* coverity[misra_c_2012_rule_8_13_violation] */
                                     uint8_t * pBuffer,
                                     uint32_t bufferLength,
                                     /* coverity[misra_c_2012_rule_8_13_violation] */
                                     uint32_t * pReceivedDataLength )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };
    uint32_t recvTimeout = DATA_READ_TIMEOUT_MS;
    uint32_t recvLen = bufferLength;
    _socketDataRecv_t dataRecv =
    {
        pReceivedDataLength,
        pBuffer,
        NULL
    };
    CellularAtReq_t atReqSocketRecv =
    {
        cmdBuf,
        CELLULAR_AT_MULTI_DATA_WO_PREFIX,
        ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ? "+QSSLRECV" : "+QIRD" ),
        _Cellular_RecvFuncData,
        ( void * ) &dataRecv,
        bufferLength,
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "_Cellular_CheckLibraryStatus failed." ) );
    }
    else if( socketHandle == NULL )
    {
        LogError( ( "Cellular_SocketRecv: Invalid socket handle." ) );
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ( pBuffer == NULL ) || ( pReceivedDataLength == NULL ) || ( bufferLength == 0U ) )
    {
        LogError( ( "Cellular_SocketRecv: Bad input Param." ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( socketHandle->socketState != SOCKETSTATE_CONNECTED )
    {
        /* Check the socket connection state. */
        LogInfo( ( "Cellular_SocketRecv: socket state %d is not connected.", socketHandle->socketState ) );

        if( ( socketHandle->socketState == SOCKETSTATE_ALLOCATED ) || ( socketHandle->socketState == SOCKETSTATE_CONNECTING ) )
        {
            cellularStatus = CELLULAR_SOCKET_NOT_CONNECTED;
        }
        else
        {
            cellularStatus = CELLULAR_SOCKET_CLOSED;
        }
    }
    else
    {
        /* Update recvLen to maximum module length. */
        if( CELLULAR_MAX_RECV_DATA_LEN <= bufferLength )
        {
            recvLen = ( uint32_t ) CELLULAR_MAX_RECV_DATA_LEN;
        }

        /* Update receive timeout to default timeout if not set with setsocketopt. */
        if( socketHandle->recvTimeoutMs != 0U )
        {
            recvTimeout = socketHandle->recvTimeoutMs;
        }

        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "%s%ld,%ld",
                           ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ?
                                    "AT+QSSLRECV=" : "AT+QIRD=" ),
                           socketHandle->socketId, recvLen );
        pktStatus = _Cellular_TimeoutAtcmdDataRecvRequestWithCallback(
                pContext, atReqSocketRecv, recvTimeout,
                ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ?
                        sslSocketRecvDataPrefix : socketRecvDataPrefix ),
                NULL );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            /* Reset data handling parameters. */
            LogError( ( "_Cellular_RecvData: Data Receive fail, pktStatus: %d", pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
/* coverity[misra_c_2012_rule_8_13_violation] */
CellularError_t Cellular_SocketSend( CellularHandle_t cellularHandle,
                                     CellularSocketHandle_t socketHandle,
                                     const uint8_t * pData,
                                     uint32_t dataLength,
                                     /* coverity[misra_c_2012_rule_8_13_violation] */
                                     uint32_t * pSentDataLength )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    uint32_t sendTimeout = DATA_SEND_TIMEOUT_MS;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSocketSend =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };
    CellularAtDataReq_t atDataReqSocketSend =
    {
        pData,
        dataLength,
        pSentDataLength,
        NULL,
        0,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "_Cellular_CheckLibraryStatus failed." ) );
    }
    else if( socketHandle == NULL )
    {
        LogError( ( "Cellular_SocketSend: Invalid socket handle." ) );
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ( pData == NULL ) || ( pSentDataLength == NULL ) || ( dataLength == 0U ) )
    {
        LogError( ( "Cellular_SocketSend: Invalid parameter." ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( socketHandle->socketState != SOCKETSTATE_CONNECTED )
    {
        /* Check the socket connection state. */
        LogInfo( ( "Cellular_SocketSend: socket state %d is not connected.", socketHandle->socketState ) );

        if( ( socketHandle->socketState == SOCKETSTATE_ALLOCATED ) || ( socketHandle->socketState == SOCKETSTATE_CONNECTING ) )
        {
            cellularStatus = CELLULAR_SOCKET_NOT_CONNECTED;
        }
        else
        {
            cellularStatus = CELLULAR_SOCKET_CLOSED;
        }
    }
    else
    {
        /* Send data length check. */
        if( dataLength > ( uint32_t ) CELLULAR_MAX_SEND_DATA_LEN )
        {
            atDataReqSocketSend.dataLen = ( uint32_t ) CELLULAR_MAX_SEND_DATA_LEN;
        }

        /* Check send timeout. If not set by setsockopt, use default value. */
        if( socketHandle->sendTimeoutMs != 0U )
        {
            sendTimeout = socketHandle->sendTimeoutMs;
        }

        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "%s%ld,%ld",
                           ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ?
                                    "AT+QSSLSEND=" : "AT+QISEND=" ),
                           socketHandle->socketId, atDataReqSocketSend.dataLen );

        pktStatus = _Cellular_AtcmdDataSend( pContext, atReqSocketSend, atDataReqSocketSend,
                                             socketSendDataPrefix, NULL,
                                             PACKET_REQ_TIMEOUT_MS, sendTimeout, 0U );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SocketSend: Data send fail, PktRet: %d", pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SocketClose( CellularHandle_t cellularHandle,
                                      CellularSocketHandle_t socketHandle )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_TYPICAL_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSockClose =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    /* Make sure the library is open. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( socketHandle == NULL )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else
    {
        if( socketHandle->socketState == SOCKETSTATE_CONNECTING )
        {
            LogWarn( ( "Cellular_SocketClose: Socket state is SOCKETSTATE_CONNECTING." ) );
        }

        if( ( socketHandle->socketState == SOCKETSTATE_CONNECTING ) ||
            ( socketHandle->socketState == SOCKETSTATE_CONNECTED ) ||
            ( socketHandle->socketState == SOCKETSTATE_DISCONNECTED ) )
        {
            /* Form the AT command. */

            /* The return value of snprintf is not used.
             * The max length of the string is fixed and checked offline. */
            /* coverity[misra_c_2012_rule_21_6_violation]. */
            ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_TYPICAL_MAX_SIZE, "%s%ld",
                               ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ?
                                        "AT+QSSLCLOSE=" : "AT+QICLOSE=" ),
                               socketHandle->socketId );
            pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback( pContext, atReqSockClose,
                                                                   SOCKET_DISCONNECT_PACKET_REQ_TIMEOUT_MS );

            if( pktStatus != CELLULAR_PKT_STATUS_OK )
            {
                // TODO (MV): Why is this being ignored? This results in potentially orphaning the Quectel socket.
                //            Should handle the cases where the modem didn't receive the command successfully by retrying
                LogError( ( "Cellular_SocketClose: Socket close failed, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            }
        }

        /* Ignore the result from the info, and force to remove the socket. */
        cellularStatus = _Cellular_RemoveSocketData( pContext, socketHandle );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_SocketConnect( CellularHandle_t cellularHandle,
                                        CellularSocketHandle_t socketHandle,
                                        CellularSocketAccessMode_t dataAccessMode,
                                        const CellularSocketAddress_t * pRemoteSocketAddress )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSocketConnect =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    /* Make sure the library is open. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "Cellular_SocketConnect: _Cellular_CheckLibraryStatus failed." ) );
    }
    else if( pRemoteSocketAddress == NULL )
    {
        LogError( ( "Cellular_SocketConnect: Invalid socket address." ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( socketHandle == NULL )
    {
        LogError( ( "Cellular_SocketConnect: Invalid socket handle." ) );
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ( socketHandle->socketState == SOCKETSTATE_CONNECTED ) || ( socketHandle->socketState == SOCKETSTATE_CONNECTING ) )
    {
        LogError( ( "Cellular_SocketConnect: Not allowed in state %d.", socketHandle->socketState ) );
        cellularStatus = CELLULAR_NOT_ALLOWED;
    }
    else
    {
        cellularStatus = storeAccessModeAndAddress( pContext, socketHandle, dataAccessMode, pRemoteSocketAddress );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Builds the Socket connect command. */
        cellularStatus = buildSocketConnect( socketHandle, cmdBuf, sizeof(cmdBuf) );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Set the socket state to connecting state. If cellular modem returns error,
         * revert the state to allocated state. */
        socketHandle->socketState = SOCKETSTATE_CONNECTING;

        pktStatus = _Cellular_TimeoutAtcmdRequestWithCallback(
                pContext, atReqSocketConnect,
                ( socketHandle->socketProtocol == CELLULAR_SOCKET_PROTOCOL_SSL_OVER_TCP ?
                        SSL_SOCKET_CONNECT_PACKET_REQ_TIMEOUT_MS : SOCKET_CONNECT_PACKET_REQ_TIMEOUT_MS ) );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SocketConnect: Socket connect failed, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );

            /* Revert the state to allocated state. */
            socketHandle->socketState = SOCKETSTATE_ALLOCATED;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
/* coverity[misra_c_2012_rule_8_13_violation] */
CellularError_t Cellular_GetPdnStatus( CellularHandle_t cellularHandle,
                                       CellularPdnStatus_t * pPdnStatusBuffers,
                                       uint8_t numStatusBuffers,
                                       uint8_t * pNumStatus )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    const CellularPdnStatus_t * pTempPdnStatusBuffer = pPdnStatusBuffers;
    uint8_t numBuffers = 0;
    CellularAtReq_t atReqGetPdnStatus =
    {
        "AT+QIACT?",
        CELLULAR_AT_WITH_PREFIX,
        "+QIACT",
        _Cellular_RecvFuncGetPdnStatus,
        pPdnStatusBuffers,
        numStatusBuffers,
    };

    if( ( pTempPdnStatusBuffer == NULL ) || ( pNumStatus == NULL ) || ( numStatusBuffers < 1u ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
        LogWarn( ( "_Cellular_GetPdnStatus: Bad input Parameter " ) );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Make sure the library is open. */
        cellularStatus = _Cellular_CheckLibraryStatus( pContext );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetPdnStatus );
        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* Populate the Valid number of statuses. */
        *pNumStatus = 0;
        numBuffers = numStatusBuffers;

        while( numBuffers != 0U )
        {
            /* Check if the PDN state is valid. The context ID of the first
             * invalid PDN status is set to FF. */
            if( ( pTempPdnStatusBuffer->contextId != INVALID_PDN_INDEX ) &&
                ( pTempPdnStatusBuffer->contextId >= CELLULAR_PDN_CONTEXT_ID_MIN ) &&
                ( pTempPdnStatusBuffer->contextId <= CELLULAR_PDN_CONTEXT_ID_MAX ) )
            {
                ( *pNumStatus ) += 1U;
            }
            else
            {
                break;
            }

            numBuffers--;
            pTempPdnStatusBuffer++;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetSimCardStatus( CellularHandle_t cellularHandle,
                                           CellularSimCardStatus_t * pSimCardStatus )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetSimCardStatus =
    {
        "AT+QSIMSTAT?",
        CELLULAR_AT_WITH_PREFIX,
        "+QSIMSTAT",
        _Cellular_RecvFuncGetSimCardStatus,
        &pSimCardStatus->simCardState,
        sizeof( CellularSimCardState_t ),
    };
    CellularAtReq_t atReqGetSimLockStatus =
    {
        "AT+CPIN?",
        CELLULAR_AT_WITH_PREFIX,
        "+CPIN",
        _Cellular_RecvFuncGetSimLockStatus,
        &pSimCardStatus->simCardLockState,
        sizeof( CellularSimCardLockState_t ),
    };

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pSimCardStatus == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Initialize the sim state and the sim lock state. */
        pSimCardStatus->simCardState = CELLULAR_SIM_CARD_UNKNOWN;
        pSimCardStatus->simCardLockState = CELLULAR_SIM_CARD_LOCK_UNKNOWN;

        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetSimCardStatus );

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetSimLockStatus );
        }

        cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        LogDebug( ( "_Cellular_GetSimStatus, Sim Insert State[%d], Lock State[%d]",
                    pSimCardStatus->simCardState, pSimCardStatus->simCardLockState ) );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetSimCardInfo( CellularHandle_t cellularHandle,
                                         CellularSimCardInfo_t * pSimCardInfo )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;

    CellularAtReq_t atReqGetIccid =
    {
        "AT+QCCID",
        CELLULAR_AT_WITH_PREFIX,
        "+QCCID",
        _Cellular_RecvFuncGetIccid,
        pSimCardInfo->iccid,
        CELLULAR_ICCID_MAX_SIZE + 1U,
    };
    CellularAtReq_t atReqGetImsi =
    {
        "AT+CIMI",
        CELLULAR_AT_WO_PREFIX,
        NULL,
        _Cellular_RecvFuncGetImsi,
        pSimCardInfo->imsi,
        CELLULAR_IMSI_MAX_SIZE + 1U,
    };
    CellularAtReq_t atReqGetHplmn =
    {
        "AT+CRSM=176,28514,0,0,0",
        CELLULAR_AT_WITH_PREFIX,
        "+CRSM",
        _Cellular_RecvFuncGetHplmn,
        &pSimCardInfo->plmn,
        sizeof( CellularPlmnInfo_t ),
    };

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pSimCardInfo == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        ( void ) memset( pSimCardInfo, 0, sizeof( CellularSimCardInfo_t ) );
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetImsi );

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetHplmn );
        }

        if( pktStatus == CELLULAR_PKT_STATUS_OK )
        {
            pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetIccid );
        }

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
        else
        {
            LogDebug( ( "SimInfo updated: IMSI:%s, Hplmn:%s%s, ICCID:%s",
                        pSimCardInfo->imsi, pSimCardInfo->plmn.mcc, pSimCardInfo->plmn.mnc,
                        pSimCardInfo->iccid ) );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_RegisterUrcSignalStrengthChangedCallback( CellularHandle_t cellularHandle,
                                                                   CellularUrcSignalStrengthChangedCallback_t signalStrengthChangedCallback,
                                                                   void * pCallbackContext )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;

    /* pContext is checked in the common library. */
    cellularStatus = Cellular_CommonRegisterUrcSignalStrengthChangedCallback(
        cellularHandle, signalStrengthChangedCallback, pCallbackContext );

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        if( signalStrengthChangedCallback != NULL )
        {
            cellularStatus = controlSignalStrengthIndication( pContext, true );
        }
        else
        {
            cellularStatus = controlSignalStrengthIndication( pContext, false );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library API. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularError_t Cellular_GetHostByName( CellularHandle_t cellularHandle,
                                        uint8_t contextId,
                                        const char * pcHostName,
                                        char * pResolvedAddress )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_QUERY_DNS_MAX_SIZE ];
    cellularDnsQueryResult_t dnsQueryResult = CELLULAR_DNS_QUERY_UNKNOWN;
    cellularModuleContext_t * pModuleContext = NULL;
    CellularAtReq_t atReqQueryDns =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( pcHostName == NULL ) || ( pResolvedAddress == NULL ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        cellularStatus = _Cellular_IsValidPdn( contextId );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        cellularStatus = _Cellular_GetModuleContext( pContext, ( void ** ) &pModuleContext );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        PlatformMutex_Lock( &pModuleContext->dnsQueryMutex );
        pModuleContext->dnsResultNumber = 0;
        pModuleContext->dnsIndex = 0;
        ( void ) xQueueReset( pModuleContext->pktDnsQueue );
        cellularStatus = registerDnsEventCallback( pModuleContext, _dnsResultCallback, pResolvedAddress );
    }

    /* Send the AT command and wait the URC result. */
    if( cellularStatus == CELLULAR_SUCCESS )
    {
        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_QUERY_DNS_MAX_SIZE,
                           "AT+QIDNSGIP=%u,\"%s\"", contextId, pcHostName );
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqQueryDns );  // TODO (MV): Is the default 5s timeout valid here, datasheet doesn't specify time to get OK, probably OK, ask Quectel?

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetHostByName: couldn't resolve host name" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
            PlatformMutex_Unlock( &pModuleContext->dnsQueryMutex );
        }
    }

    /* URC handler calls the callback to unblock this function. */
    if( cellularStatus == CELLULAR_SUCCESS )
    {
        if( xQueueReceive( pModuleContext->pktDnsQueue, &dnsQueryResult,
                           pdMS_TO_TICKS( DNS_QUERY_TIMEOUT_MS ) ) == pdTRUE )
        {
            if( dnsQueryResult != CELLULAR_DNS_QUERY_SUCCESS )
            {
                cellularStatus = CELLULAR_UNKNOWN;
            }
        }
        else
        {
            ( void ) registerDnsEventCallback( pModuleContext, NULL, NULL );
            cellularStatus = CELLULAR_TIMEOUT;
        }

        PlatformMutex_Unlock( &pModuleContext->dnsQueryMutex );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

CellularError_t Cellular_Init( CellularHandle_t * pCellularHandle,
                               const CellularCommInterface_t * pCommInterface )
{
    CellularTokenTable_t cellularTokenTable =
    {
        .pCellularUrcHandlerTable              = CellularUrcHandlerTable,
        .cellularPrefixToParserMapSize         = CellularUrcHandlerTableSize,
        .pCellularSrcTokenErrorTable           = CellularSrcTokenErrorTable,
        .cellularSrcTokenErrorTableSize        = CellularSrcTokenErrorTableSize,
        .pCellularSrcTokenSuccessTable         = CellularSrcTokenSuccessTable,
        .cellularSrcTokenSuccessTableSize      = CellularSrcTokenSuccessTableSize,
        .pCellularUrcTokenWoPrefixTable        = CellularUrcTokenWoPrefixTable,
        .cellularUrcTokenWoPrefixTableSize     = CellularUrcTokenWoPrefixTableSize,
        .pCellularSrcExtraTokenSuccessTable    = NULL,
        .cellularSrcExtraTokenSuccessTableSize = 0
    };

    return Cellular_CommonInit( pCellularHandle, pCommInterface, &cellularTokenTable );
}

/*-----------------------------------------------------------*/

static bool _parseFileUploadResult(char * pQfuplPayload,
                                   CellularFileUploadResult_t * pFileUploadResult )
{
    char * pToken = NULL, * pTmpQfuplPayload = pQfuplPayload;
    int32_t tempValue = 0;
    uint32_t tempUValue = 0;
    bool parseStatus = true;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( ( pFileUploadResult == NULL ) || (pQfuplPayload == NULL ) )
    {
        LogError( ( "_parseSignalQuality: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( ( parseStatus == true ) && (Cellular_ATGetNextTok(&pTmpQfuplPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoui( pToken, 10, &tempUValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pFileUploadResult->uploadedFileLength = tempUValue;
        }
        else
        {
            LogError( ( "_parseFileUploadResult: Error in processing upload size. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && (Cellular_ATGetNextTok(&pTmpQfuplPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 16, &tempValue );     // NOTE: File AT command v1.0 documentation states integer value (pg. 15) but actually hex string (with no leading 0X)

        if( atCoreStatus == CELLULAR_AT_SUCCESS &&
            tempValue >= 0 && tempValue <= UINT16_MAX )
        {
            pFileUploadResult->xorChecksum = ( uint16_t ) tempValue;
        }
        else
        {
            LogError( ( "_parseFileUploadResult: Error in processing XOR checksum. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFileUploadResult( CellularContext_t * pContext,
                                                           const CellularATCommandResponse_t * pAtResp,
                                                           void * pData,
                                                           uint16_t dataLen )
{
    char * pInputLine = NULL;
    CellularFileUploadResult_t * pFileUploadResult = ( CellularFileUploadResult_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pFileUploadResult == NULL ) || ( dataLen != sizeof( CellularFileUploadResult_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "GetSignalInfo: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pInputLine );
        }

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
        parseStatus = _parseFileUploadResult(pInputLine, pFileUploadResult );

        if( parseStatus != true )
        {
            pFileUploadResult->uploadedFileLength = 0;
            pFileUploadResult->xorChecksum = 0;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

static CellularPktStatus_t fileUploadDataPrefix( void * pCallbackContext,
                                                 char * pLine,
                                                 uint32_t * pBytesRead )
{
    static const int FILE_UPLOAD_DATA_PREFIX_LENGTH = 9U;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;

    if( ( pLine == NULL ) || ( pBytesRead == NULL ) )
    {
        LogError( ( "fileUploadDataPrefix: pLine is invalid or pBytesRead is invalid" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( pCallbackContext != NULL )
    {
        LogError( ( "fileUploadDataPrefix: pCallbackContext is not NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( *pBytesRead != FILE_UPLOAD_DATA_PREFIX_LENGTH )    // TODO (MV): Should this be 8? ie. 7 + 1 like in socketSendDataPrefix?
    {
        LogDebug( ( "fileUploadDataPrefix: pBytesRead %u %s is not %d", *pBytesRead, pLine, FILE_UPLOAD_DATA_PREFIX_LENGTH ) );
    }
    else
    {
        /* After the data prefix, there should not be any data in stream.
         * Cellular common processes AT command in lines. Add a '\0' after 'CONNECT'. */
        if( strcmp( pLine, "CONNECT\r\n" ) == 0 )
        {
            pLine[ FILE_UPLOAD_DATA_PREFIX_LENGTH ] = '\n';
        }
    }

    return pktStatus;
}

// TODO (MV): Confirm post data response still works
CellularError_t Cellular_UploadFileToModem( CellularHandle_t cellularHandle,
                                            const char * pcFilename,
                                            const uint8_t * pFile,
                                            uint32_t fileLength,
                                            CellularFileUploadResult_t * fileUploadResult )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    uint32_t sentFileLength = 0;
    CellularAtReq_t atReqSocketSend =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };
    CellularAtDataReq_t atDataReqSocketSend =
    {
        pFile,
        fileLength,
        &sentFileLength,
        NULL,
        0,
        CELLULAR_AT_WITH_PREFIX,
        "+QFUPL",
        _Cellular_RecvFileUploadResult,
        fileUploadResult,
        sizeof(CellularFileUploadResult_t),
    };

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "_Cellular_CheckLibraryStatus failed." ) );
    }
    else if((pFile == NULL ) || (fileLength == 0U ) || (fileLength > CELLULAR_CONFIG_FILE_UPLOAD_MAX_SIZE) || (fileUploadResult == NULL ) )
    {
        LogError( ( "Cellular_UploadFileToModem: Invalid parameter." ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Send data length check. */
        if(fileLength > ( uint32_t ) CELLULAR_CONFIG_FILE_UPLOAD_MAX_SIZE )
        {
            atDataReqSocketSend.dataLen = ( uint32_t ) CELLULAR_CONFIG_FILE_UPLOAD_MAX_SIZE;
        }

        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s\"%s\",%lu",
                           "AT+QFUPL=", pcFilename, atDataReqSocketSend.dataLen );

        pktStatus = _Cellular_AtcmdDataSend( pContext, atReqSocketSend, atDataReqSocketSend,
                                             fileUploadDataPrefix, NULL,
                                             PACKET_REQ_TIMEOUT_MS, PACKET_REQ_TIMEOUT_MS, 0U );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_UploadFileToModem: Data send fail, PktRet: %d", pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        } else if (sentFileLength != fileLength) {
            LogError( ( "Cellular_UploadFileToModem: File send fail, len: %lu, sentLen: %lu", fileLength, sentFileLength ) );
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

CellularError_t Cellular_DeleteFileOnModem( CellularHandle_t cellularHandle,
                                            const char * pcFilename )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqDeleteFile =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( pcFilename == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        (void) snprintf(cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s\"%s\"", "AT+QFDEL=", pcFilename);
        pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqDeleteFile);

        if (pktStatus != CELLULAR_PKT_STATUS_OK) {
            LogError(("Cellular_DeleteFileOnModem: couldn't delete the file, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus));
            cellularStatus = _Cellular_TranslatePktStatus(pktStatus);
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

typedef struct BG770FileCRCs
{
    uint32_t crc32;
    uint16_t crc16;
    uint16_t crc16_ccitt;
} BG770FileCRCs_t;

static bool _parseFileCRCs( char * pQfcrcPayload,
                            BG770FileCRCs_t * pFileCRCs )
{
    char * pToken = NULL, * pTmpQfcrcPayload = pQfcrcPayload;
    int32_t tempValue = 0;
    uint32_t tempUValue = 0;
    bool parseStatus = true;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( ( pFileCRCs == NULL ) || ( pQfcrcPayload == NULL ) )
    {
        LogError( ( "_parseSignalQuality: Invalid Input Parameters" ) );
        parseStatus = false;
    }

    if( ( parseStatus == true ) && (Cellular_ATGetNextTok(&pTmpQfcrcPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoui( pToken, 16, &tempUValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pFileCRCs->crc32 = tempUValue;
        }
        else
        {
            LogError( ( "_parseFileCRCs: Error in processing CRC32. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && (Cellular_ATGetNextTok(&pTmpQfcrcPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 16, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS &&
            tempValue >= 0 && tempValue <= UINT16_MAX )
        {
            pFileCRCs->crc16 = ( uint16_t ) tempValue;
        }
        else
        {
            LogError( ( "_parseFileCRCs: Error in processing CRC16. Token %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    if( ( parseStatus == true ) && (Cellular_ATGetNextTok(&pTmpQfcrcPayload, &pToken ) == CELLULAR_AT_SUCCESS ) )
    {
        atCoreStatus = Cellular_ATStrtoi( pToken, 16, &tempValue );

        if( atCoreStatus == CELLULAR_AT_SUCCESS &&
            tempValue >= 0 && tempValue <= UINT16_MAX )
        {
            pFileCRCs->crc16_ccitt = ( uint16_t ) tempValue;
        }
        else
        {
            LogError( ( "_parseFileCRCs: Error in processing CCITT CRC16. pToken %s", pToken ) );
            parseStatus = false;
        }
    }
    else
    {
        parseStatus = false;
    }

    return parseStatus;
}

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFileCRCs( CellularContext_t * pContext,
                                                   const CellularATCommandResponse_t * pAtResp,
                                                   void * pData,
                                                   uint16_t dataLen )
{
    char * pInputLine = NULL;
    BG770FileCRCs_t * pFileCRCs = ( BG770FileCRCs_t * ) pData;
    bool parseStatus = true;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;

    if( pContext == NULL )
    {
        pktStatus = CELLULAR_PKT_STATUS_INVALID_HANDLE;
    }
    else if( ( pFileCRCs == NULL ) || ( dataLen != sizeof( BG770FileCRCs_t ) ) )
    {
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) || ( pAtResp->pItm->pLine == NULL ) )
    {
        LogError( ( "GetSignalInfo: Input Line passed is NULL" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllDoubleQuote( pInputLine );
        }

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
        parseStatus = _parseFileCRCs( pInputLine, pFileCRCs );

        if( parseStatus != true )
        {
            pFileCRCs->crc32 = 0;
            pFileCRCs->crc16 = 0;
            pFileCRCs->crc16_ccitt = 0;
            pktStatus = CELLULAR_PKT_STATUS_FAILURE;
        }
    }

    return pktStatus;
}

CellularError_t Cellular_GetModemFileCRC32( CellularHandle_t cellularHandle,
                                            const char * pcFilename,
                                            uint32_t * crc32 )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    BG770FileCRCs_t fileCRCs = { 0 };
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqGetFileCRCs =
    {
        cmdBuf,
        CELLULAR_AT_WITH_PREFIX,
        "+QFCRC",
        _Cellular_RecvFileCRCs,
        &fileCRCs,
        sizeof(BG770FileCRCs_t),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( pcFilename == NULL ) || ( crc32 == NULL ) )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Form the AT command. */

        /* The return value of snprintf is not used.
         * The max length of the string is fixed and checked offline. */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s\"%s\"", "AT+QFCRC=", pcFilename );
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetFileCRCs );

        *crc32 = fileCRCs.crc32;

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetModemFileCRC32: couldn't get the file CRC32, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

static bool _tryTranslateSSLVersionEnumToBG770SSLVersionValue(
        const CellularSSLVersion_t sslVersion,
        int *const out_sslVersionValue)
{
    if (out_sslVersionValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslVersion) {
        case CELLULAR_SSL_VERSION_SSL_3_0:
            *out_sslVersionValue = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_VERSION_TLS_1_0:
            *out_sslVersionValue = 1;
            retVal = true;
            break;
        case CELLULAR_SSL_VERSION_TLS_1_1:
            *out_sslVersionValue = 2;
            retVal = true;
            break;
        case CELLULAR_SSL_VERSION_TLS_1_2:
            *out_sslVersionValue = 3;
            retVal = true;
            break;
        case CELLULAR_SSL_VERSION_ALL:
            *out_sslVersionValue = 4;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

#define CIPHER_SUITE_TLS_RSA_WITH_AES_256_CBC_SHA            0X0035  // TLS_RSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_RSA_WITH_AES_128_CBC_SHA            0X002F  // TLS_RSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_RSA_WITH_RC4_128_SHA                0X0005  // TLS_RSA_WITH_RC4_128_SHA
#define CIPHER_SUITE_TLS_RSA_WITH_RC4_128_MD5                0X0004  // TLS_RSA_WITH_RC4_128_MD5
#define CIPHER_SUITE_TLS_RSA_WITH_3DES_EDE_CBC_SHA           0X000A  // TLS_RSA_WITH_3DES_EDE_CBC_SHA
#define CIPHER_SUITE_TLS_RSA_WITH_AES_256_CBC_SHA256         0X003D  // TLS_RSA_WITH_AES_256_CBC_SHA256
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_RC4_128_SHA         0XC002  // TLS_ECDH_ECDSA_WITH_RC4_128_SHA
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA    0XC003  // TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA     0XC004  // TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA     0XC005  // TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_RC4_128_SHA        0XC007  // TLS_ECDHE_ECDSA_WITH_RC4_128_SHA
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA   0XC008  // TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA    0XC009  // TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA    0XC00A  // TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_RC4_128_SHA          0XC011  // TLS_ECDHE_RSA_WITH_RC4_128_SHA
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA     0XC012  // TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA      0XC013  // TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA      0XC014  // TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_RC4_128_SHA           0XC00C  // TLS_ECDH_RSA_WITH_RC4_128_SHA
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA      0XC00D  // TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA       0XC00E  // TLS_ECDH_RSA_WITH_AES_128_CBC_SHA
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA       0XC00F  // TLS_ECDH_RSA_WITH_AES_256_CBC_SHA
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 0XC023  // TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384 0XC024  // TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256  0XC025  // TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256
#define CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384  0XC026  // TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256   0XC027  // TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384   0XC028  // TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256    0XC029  // TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256
#define CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384    0XC02A  // TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384
#define CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256 0XC02B  // TLS-ECDHE-ECDSA-WITH-AES-128-GCM-SHA256
#define CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256   0XC02F  // TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
#define CIPHER_SUITE_TLS_PSK_WITH_AES_128_CCM_8              0XC0A8  // TLS_PSK_WITH_AES_128_CCM_8
#define CIPHER_SUITE_TLS_PSK_WITH_AES_128_CBC_SHA256         0X00AE  // TLS_PSK_WITH_AES_128_CBC_SHA256
#define CIPHER_SUITE_ALL                                     0XFFFF  // Support all cipher suites

static const uint32_t _sslCipherMapping[] = {
        CIPHER_SUITE_TLS_RSA_WITH_AES_256_CBC_SHA,
        CIPHER_SUITE_TLS_RSA_WITH_AES_128_CBC_SHA,
        CIPHER_SUITE_TLS_RSA_WITH_RC4_128_SHA,
        CIPHER_SUITE_TLS_RSA_WITH_RC4_128_MD5,
        CIPHER_SUITE_TLS_RSA_WITH_3DES_EDE_CBC_SHA,
        CIPHER_SUITE_TLS_RSA_WITH_AES_256_CBC_SHA256,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_RC4_128_SHA,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_3DES_EDE_CBC_SHA,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_RC4_128_SHA,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_3DES_EDE_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_RC4_128_SHA,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_3DES_EDE_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_RC4_128_SHA,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_3DES_EDE_CBC_SHA,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256,
        CIPHER_SUITE_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256,
        CIPHER_SUITE_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384,
        CIPHER_SUITE_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256,
        CIPHER_SUITE_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,
        CIPHER_SUITE_TLS_PSK_WITH_AES_128_CCM_8,
        CIPHER_SUITE_TLS_PSK_WITH_AES_128_CBC_SHA256
};
static const size_t SSL_CIPHER_MAPPING_LENGTH = sizeof( _sslCipherMapping ) / sizeof( _sslCipherMapping[ 0 ] );

static bool _tryTranslateSSLCipherSuiteBitmaskToBG770SSLCipherSuites(
        const CellularSSLCipherSuite_t sslCipherSuites,
        uint32_t *const out_bg770CipherSuites)
{
    if (out_bg770CipherSuites == NULL) {
        return false;
    }

    bool retVal = false;
    uint32_t bg770CipherSuite = 0;
    if (sslCipherSuites == CELLULAR_SSL_CIPHER_SUITE_SUPPORT_ALL)
    {
        bg770CipherSuite = CIPHER_SUITE_ALL;
        retVal = true;
    } else
    {
        for (unsigned int i = 0; i < sizeof(CellularSSLCipherSuite_t) * 8; i++)
        {
            uint64_t curBitMask = (1ULL << i);
            if ((sslCipherSuites & curBitMask) != 0)
            {
                if (bg770CipherSuite == 0) {   // no cipher assigned yet
                    if (i < SSL_CIPHER_MAPPING_LENGTH) {
                        bg770CipherSuite = _sslCipherMapping[i];
                        LogDebug( ( "BG770 SSL cipher suite 0X%lX set", bg770CipherSuite ) );
                        retVal = true;
                    } else {
                        LogError( ( "BG770 does not support selected cipher suite (0X%llX)", curBitMask ) );
                        retVal = false;
                    }
                } else {
                    LogError( ( "BG770 does not support multiple cipher suites, cipher 0X%lX already set", bg770CipherSuite ) );
                    retVal = false;
                }
            }
        }
    }

    *out_bg770CipherSuites = bg770CipherSuite;
    return retVal;
}

static bool _tryTranslateSSLAuthModeEnumToBG770SSLAuthModeValue(
        const CellularSSLAuthMode_t sslAuthMode,
        int *const out_sslAuthModeValue)
{
    if (out_sslAuthModeValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslAuthMode) {
        case CELLULAR_SSL_AUTH_MODE_NONE:
            *out_sslAuthModeValue = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_AUTH_MODE_SERVER:
            *out_sslAuthModeValue = 1;
            retVal = true;
            break;
        case CELLULAR_SSL_AUTH_MODE_SERVER_AND_CLIENT:
            *out_sslAuthModeValue = 2;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateSSLSessionResumptionEnumToBG770SSLSessionResumptionValue(
        const CellularSSLSessionResumption_t sslSessionResumption,
        int *const out_sslSessionResumptionValue)
{
    if (out_sslSessionResumptionValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslSessionResumption) {
        case CELLULAR_SSL_SESSION_RESUME_DISABLE:
            *out_sslSessionResumptionValue = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_SESSION_RESUME_ENABLE:
            *out_sslSessionResumptionValue = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateSSLSNIEnumToBG770SSLSNIValue(
        const CellularSSLSNI_t sslSNI,
        int *const out_sslSNIValue)
{
    if (out_sslSNIValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslSNI) {
        case CELLULAR_SSL_SNI_DISABLE:
            *out_sslSNIValue = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_SNI_ENABLE:
            *out_sslSNIValue = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateSSLCheckHostEnumToBG770SSLCheckHostValue(
        const CellularSSLCheckHost_t sslCheckHost,
        int *const out_sslCheckHost)
{
    if (out_sslCheckHost == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslCheckHost) {
        case CELLULAR_SSL_CHECK_HOST_DISABLE:
            *out_sslCheckHost = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_CHECK_HOST_ENABLE:
            *out_sslCheckHost = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateSSLIgnoreLocaltimeEnumToBG770SSLIgnoreLocaltimeValue(
        const CellularSSLIgnoreLocaltime_t sslIgnoreLocaltime,
        int *const out_sslIgnoreLocaltime)
{
    if (out_sslIgnoreLocaltime == NULL) {
        return false;
    }

    bool retVal = false;
    switch (sslIgnoreLocaltime) {
        case CELLULAR_SSL_IGNORE_LOCALTIME_OFF:
            *out_sslIgnoreLocaltime = 0;
            retVal = true;
            break;
        case CELLULAR_SSL_IGNORE_LOCALTIME_ON:
            *out_sslIgnoreLocaltime = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateTLSRenegotiationEnumToBG770TLSRenegotiationValue(
        const CellularTLSRenegotiation_t tlsRenegotiation,
        int *const out_tlsRenegotiationValue)
{
    if (out_tlsRenegotiationValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (tlsRenegotiation) {
        case CELLULAR_TLS_RENEGOTIATION_DISABLE:
            *out_tlsRenegotiationValue = 0;
            retVal = true;
            break;
        case CELLULAR_TLS_RENEGOTIATION_ENABLE:
            *out_tlsRenegotiationValue = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateDTLSEnableEnumToBG770DTLSEnableValue(
        const CellularDTLSEnable_t dtlsEnable,
        int *const out_dtlsEnableValue)
{
    if (out_dtlsEnableValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (dtlsEnable) {
        case CELLULAR_DTLS_DISABLE:
            *out_dtlsEnableValue = 0;
            retVal = true;
            break;
        case CELLULAR_DTLS_ENABLE:
            *out_dtlsEnableValue = 1;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

static bool _tryTranslateDTLSVersionEnumToBG770DTLSVersionValue(
        const CellularDTLSVersion_t dtlsVersion,
        int *const out_dtlsVersionValue)
{
    if (out_dtlsVersionValue == NULL) {
        return false;
    }

    bool retVal = false;
    switch (dtlsVersion) {
        case CELLULAR_DTLS_VERSION_DTLS_1_0:
            *out_dtlsVersionValue = 0;
            retVal = true;
            break;
        case CELLULAR_DTLS_VERSION_DTLS_1_2:
            *out_dtlsVersionValue = 1;
            retVal = true;
            break;
        case CELLULAR_DTLS_VERSION_BOTH:
            *out_dtlsVersionValue = 2;
            retVal = true;
            break;
        // intentionally no default case to cause compile error if not handled
    }

    return retVal;
}

typedef enum SSLConfigSettingType
{
    SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
    SSL_CONFIG_SETTING_TYPE_HEX_FORMAT,
    SSL_CONFIG_SETTING_TYPE_STRING,
} SSLConfigSettingType_t;

typedef struct SSLConfigDescription
{
    const char * paramDescription;
    SSLConfigSettingType_t type;
    union
    {
        int numeric;            // use if type == SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE
        uint32_t hex;           // use if type == SSL_CONFIG_SETTING_TYPE_HEX_FORMAT
        const char * string;    // use if type == SSL_CONFIG_SETTING_TYPE_STRING
    } param;
} SSLConfigDescription_t;

/* Internal function of Cellular_SocketSetSockOpt to reduce complexity. */
static CellularError_t _buildSetSSLOptDescription( CellularSSLContextOption_t option,
                                                   const uint8_t * pOptionValue,
                                                   uint32_t optionValueLength,
                                                   SSLConfigDescription_t *const out_sslConfigDescription)
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    if( option == CELLULAR_SSL_CONTEXT_OPTION_SSL_VERSION )
    {
        if( optionValueLength == sizeof( CellularSSLVersion_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLVersion_t * pSSLVersion = ( const CellularSSLVersion_t * ) pOptionValue;
            int sslVersionValue = -1;
            bool success = _tryTranslateSSLVersionEnumToBG770SSLVersionValue(*pSSLVersion, &sslVersionValue);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                    .paramDescription = "sslversion",
                    .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                    .param = { .numeric = sslVersionValue }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_CIPHER_SUITE )
    {
        if( optionValueLength == sizeof( CellularSSLCipherSuite_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLCipherSuite_t * pSSLCipherSuite = ( const CellularSSLCipherSuite_t * ) pOptionValue;
            uint32_t sslCipherSuite = 0;
            bool success = _tryTranslateSSLCipherSuiteBitmaskToBG770SSLCipherSuites(*pSSLCipherSuite, &sslCipherSuite);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "ciphersuite",
                        .type = SSL_CONFIG_SETTING_TYPE_HEX_FORMAT,
                        .param = { .hex = sslCipherSuite }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_CA_CERT )
    {
        if( optionValueLength == sizeof( const char * ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            *out_sslConfigDescription = (SSLConfigDescription_t){
                    .paramDescription = "cacert",
                    .type = SSL_CONFIG_SETTING_TYPE_STRING,
                    .param = { .string = ( const char * ) pOptionValue }
                };
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_CLIENT_CERT )
    {
        if( optionValueLength == sizeof( const char * ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            *out_sslConfigDescription = (SSLConfigDescription_t){
                    .paramDescription = "clientcert",
                    .type = SSL_CONFIG_SETTING_TYPE_STRING,
                    .param = { .string = ( const char * ) pOptionValue }
            };
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_CLIENT_KEY )
    {
        if( optionValueLength == sizeof( const char * ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            *out_sslConfigDescription = (SSLConfigDescription_t){
                    .paramDescription = "clientkey",
                    .type = SSL_CONFIG_SETTING_TYPE_STRING,
                    .param = { .string = ( const char * ) pOptionValue }
            };
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_AUTH_MODE )
    {
        if( optionValueLength == sizeof( CellularSSLAuthMode_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLAuthMode_t * pSSLAuthMode = ( const CellularSSLAuthMode_t * ) pOptionValue;
            int sslAuthMode = -1;
            bool success = _tryTranslateSSLAuthModeEnumToBG770SSLAuthModeValue(*pSSLAuthMode, &sslAuthMode);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "seclevel",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = sslAuthMode }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_SSL_RESUMPTION )
    {
        if( optionValueLength == sizeof( CellularSSLSessionResumption_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLSessionResumption_t * pSSLSessionResumption = ( const CellularSSLSessionResumption_t * ) pOptionValue;
            int sslSessionResumption = -1;
            bool success = _tryTranslateSSLSessionResumptionEnumToBG770SSLSessionResumptionValue(*pSSLSessionResumption, &sslSessionResumption);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "session",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = sslSessionResumption }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_SNI )
    {
        if( optionValueLength == sizeof( CellularSSLSNI_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLSNI_t * pSSLSNI = ( const CellularSSLSNI_t * ) pOptionValue;
            int sslSNI = -1;
            bool success = _tryTranslateSSLSNIEnumToBG770SSLSNIValue(*pSSLSNI, &sslSNI);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "sni",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = sslSNI }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_CHECK_HOST )
    {
        if( optionValueLength == sizeof( CellularSSLCheckHost_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLCheckHost_t * pSSLCheckHost = ( const CellularSSLCheckHost_t * ) pOptionValue;
            int sslCheckHost = -1;
            bool success = _tryTranslateSSLCheckHostEnumToBG770SSLCheckHostValue(*pSSLCheckHost, &sslCheckHost);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "checkhost",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = sslCheckHost }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_IGNORE_LOCAL_TIME )
    {
        if( optionValueLength == sizeof( CellularSSLIgnoreLocaltime_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLIgnoreLocaltime_t * pSSLIgnoreLocaltime = ( const CellularSSLIgnoreLocaltime_t * ) pOptionValue;
            int sslIgnoreLocaltime = -1;
            bool success = _tryTranslateSSLIgnoreLocaltimeEnumToBG770SSLIgnoreLocaltimeValue(*pSSLIgnoreLocaltime, &sslIgnoreLocaltime);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "ignorelocaltime",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = sslIgnoreLocaltime }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_NEGOTIATE_TIME )
    {
        if( optionValueLength == sizeof( CellularSSLNegotiateTime_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularSSLNegotiateTime_t * pSSLNegotiateTime = ( const CellularSSLNegotiateTime_t * ) pOptionValue;
            static const CellularSSLNegotiateTime_t SSL_NEGOTIATE_TIME_MIN_sec = 10;
            static const CellularSSLNegotiateTime_t SSL_NEGOTIATE_TIME_MAX_sec = 300;
            CellularSSLNegotiateTime_t sslNegotiateTime = *pSSLNegotiateTime;
            if( ( sslNegotiateTime >= SSL_NEGOTIATE_TIME_MIN_sec ) && ( sslNegotiateTime <= SSL_NEGOTIATE_TIME_MAX_sec ) )
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "negotiatetime",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = ( int ) sslNegotiateTime }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_TLS_RENEGOTIATION )
    {
        if( optionValueLength == sizeof( CellularTLSRenegotiation_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularTLSRenegotiation_t * pTLSRenegotiation = ( const CellularTLSRenegotiation_t * ) pOptionValue;
            int tlsRenegotiation = -1;
            bool success = _tryTranslateTLSRenegotiationEnumToBG770TLSRenegotiationValue(*pTLSRenegotiation, &tlsRenegotiation);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "renegotiation",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = tlsRenegotiation }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_DTLS_ENABLE )
    {
        if( optionValueLength == sizeof( CellularDTLSEnable_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularDTLSEnable_t * pDTLSEnable = ( const CellularDTLSEnable_t * ) pOptionValue;
            int dtlsEnable = -1;
            bool success = _tryTranslateDTLSEnableEnumToBG770DTLSEnableValue(*pDTLSEnable, &dtlsEnable);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "dtls",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = dtlsEnable }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else if( option == CELLULAR_SSL_CONTEXT_OPTION_DTLS_VERSION )
    {
        if( optionValueLength == sizeof( CellularDTLSVersion_t ) )
        {
            /* MISRA Ref 11.3 [Misaligned access] */
            /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-113 */
            /* coverity[misra_c_2012_rule_11_3_violation] */
            const CellularDTLSVersion_t * pDTLSVersion = ( const CellularDTLSVersion_t * ) pOptionValue;
            int dtlsVersion = -1;
            bool success = _tryTranslateDTLSVersionEnumToBG770DTLSVersionValue(*pDTLSVersion, &dtlsVersion);
            if (success)
            {
                *out_sslConfigDescription = (SSLConfigDescription_t){
                        .paramDescription = "dtlsversion",
                        .type = SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE,
                        .param = { .numeric = dtlsVersion }
                };
            } else {
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
            }
        }
        else
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }
    else
    {
        LogError( ( "_buildSetSSLOptDescription: SSL option (%d) not supported", option ) );
        cellularStatus = CELLULAR_UNSUPPORTED;
    }

    return cellularStatus;
}


static CellularError_t buildSetSSLOption( uint8_t sslContextId,
                                          char * pCmdBuf, size_t cmdBufLength,
                                          const SSLConfigDescription_t *const sslConfigDescription )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    char sslOptionValueString[ CELLULAR_AT_CMD_MAX_SIZE ];

    if( ( pCmdBuf == NULL ) || ( cmdBufLength == 0 ) )
    {
        LogError( ( "buildSetSSLOption: Invalid command buffer" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( _Cellular_IsValidSSLContext(sslContextId) != CELLULAR_SUCCESS )
    {
        LogError( ( "buildSetSSLOption: Invalid SSL context id" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else if( sslConfigDescription == NULL )
    {
        LogError( ( "buildSetSSLOption: Invalid SSL config description" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        static const size_t SSL_OPTION_VALUE_STRING_LENGTH = sizeof( sslOptionValueString );
        int len = -1;
        switch (sslConfigDescription->type) {
            case SSL_CONFIG_SETTING_TYPE_NUMERIC_VALUE:
                len = snprintf( sslOptionValueString, SSL_OPTION_VALUE_STRING_LENGTH, "%d", sslConfigDescription->param.numeric );
                break;

            case SSL_CONFIG_SETTING_TYPE_HEX_FORMAT:
                len = snprintf( sslOptionValueString, SSL_OPTION_VALUE_STRING_LENGTH, "0X%04lX", sslConfigDescription->param.hex );
                break;

            case SSL_CONFIG_SETTING_TYPE_STRING:
                len = snprintf( sslOptionValueString, SSL_OPTION_VALUE_STRING_LENGTH, "\"%s\"", sslConfigDescription->param.string );
                break;

            default:
                cellularStatus = CELLULAR_INTERNAL_FAILURE;
                break;
        }

        if ( ( len <= 0 ) || ( (size_t)len >= SSL_OPTION_VALUE_STRING_LENGTH) )
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }

    if (cellularStatus == CELLULAR_SUCCESS)
    {
        /* Form the AT command. */
        int len = snprintf( pCmdBuf, cmdBufLength,
                           "%s\"%s\",%d,%s",
                           "AT+QSSLCFG=",
                           sslConfigDescription->paramDescription,
                           sslContextId,
                           sslOptionValueString );
        if ( ( len <= 0 ) || ( (size_t)len >= cmdBufLength) )
        {
            cellularStatus = CELLULAR_INTERNAL_FAILURE;
        }
    }

    return cellularStatus;
}

static CellularError_t _Cellular_SocketSetSSLOpt( CellularContext_t * pContext,
                                                  uint8_t sslContextId,
                                                  CellularSSLContextOption_t option,
                                                  const uint8_t * pOptionValue,
                                                  uint32_t optionValueLength )
{
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSSLOpt =
    {
        cmdBuf,
        CELLULAR_AT_NO_RESULT,
        NULL,
        NULL,
        NULL,
        0,
    };

    SSLConfigDescription_t sslConfigDescription = { 0 };
    cellularStatus = _buildSetSSLOptDescription( option, pOptionValue, optionValueLength, &sslConfigDescription );
    if (cellularStatus != CELLULAR_SUCCESS)
    {
        LogError( ( "_Cellular_SocketSetSSLOpt: can't build SSL option description" ) );
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        cellularStatus = buildSetSSLOption( sslContextId, cmdBuf, sizeof( cmdBuf ), &sslConfigDescription );
        if (cellularStatus != CELLULAR_SUCCESS)
        {
            LogError( ( "_Cellular_SocketSetSSLOpt: can't build SSL option command" ) );
        }
    }

    if( cellularStatus == CELLULAR_SUCCESS )
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqSSLOpt );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "_Cellular_SocketSetSSLOpt: can't set SSL option, cmdBuf:%s, PktRet: %d", cmdBuf, pktStatus ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

CellularError_t Cellular_SocketSetSSLOpt( CellularHandle_t cellularHandle,
                                          uint8_t sslContextId,
                                          CellularSSLContextOption_t option,
                                          const uint8_t * pOptionValue,
                                          uint32_t optionValueLength )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;

    /* pContext is checked in _Cellular_CheckLibraryStatus function. */
    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( _Cellular_IsValidSSLContext(sslContextId) != CELLULAR_SUCCESS )
    {
        cellularStatus = CELLULAR_INVALID_HANDLE;
    }
    else if( ( pOptionValue == NULL ) || ( optionValueLength == 0U ) )
    {
        LogError( ( "Cellular_SocketSetSSLOpt: Invalid parameter" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        cellularStatus = _Cellular_SocketSetSSLOpt( pContext, sslContextId, option, pOptionValue, optionValueLength );
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/


/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetSocketLastResultCode( CellularContext_t * pContext,
                                                                      const CellularATCommandResponse_t * pAtResp,
                                                                      void * pData,
                                                                      uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    uint32_t * pLastResultCode = NULL;

    if( pContext == NULL )
    {
        LogError( ( "Cellular_GetSocketLastResultCode: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) || ( pData == NULL ) || ( dataLen == 0 ) )
    {
        LogError( ( "Cellular_GetSocketLastResultCode: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pLastResultCode = ( uint32_t * ) pData;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            uint32_t tempValue = 0;
            atCoreStatus = Cellular_ATStrtoui( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                *pLastResultCode = tempValue;
                LogDebug( ( "Socket last result code: %lu", *pLastResultCode ) );
            }
            else
            {
                LogError( ( "Error in processing last result code. Token %s", pToken ) );
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

CellularError_t Cellular_GetSocketLastResultCode( CellularHandle_t cellularHandle,
                                                  uint32_t * lastResultCode)
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetLastResultCode =
    {
            "AT+QIGETERROR",
            CELLULAR_AT_WITH_PREFIX,
            "+QIGETERROR",
            _Cellular_RecvFuncGetSocketLastResultCode,
            lastResultCode,
            sizeof( *lastResultCode ),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( lastResultCode == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback( pContext, atReqGetLastResultCode );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetSocketLastResultCode: couldn't retrieve last result code" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetFlowControlSetting( CellularContext_t * pContext,
                                                                    const CellularATCommandResponse_t * pAtResp,
                                                                    void * pData,
                                                                    uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    CellularModuleCommFlowControl_t * pFlowControlSetting = NULL;
    int32_t rtsFlowControl = -1;
    int32_t ctsFlowControl = -1;

    if( pContext == NULL )
    {
        LogError( ( "Cellular_GetModuleFlowControlSetting: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) ||
             ( pData == NULL ) || ( dataLen == sizeof(CellularModuleCommFlowControl_t) ) )
    {
        LogError( ( "Cellular_GetModuleFlowControlSetting: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pFlowControlSetting = ( CellularModuleCommFlowControl_t * ) pData;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &rtsFlowControl );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                LogDebug( ( "RTS flow control setting: %ld", rtsFlowControl ) );
            }
            else
            {
                LogError( ( "Error in processing RTS flow control setting. Token %s", pToken ) );
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &ctsFlowControl );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                LogDebug( ( "CTS flow control setting: %ld", ctsFlowControl ) );
            }
            else
            {
                LogError( ( "Error in processing CTS flow control setting. Token %s", pToken ) );
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            if( ( rtsFlowControl == RTS_FLOW_CONTROL_ENABLED ) && ( ctsFlowControl == CTS_FLOW_CONTROL_ENABLED ) )
            {
                *pFlowControlSetting = CELLULAR_MODULE_COMM_FLOW_CONTROL_RTS_CTS;
            }
            else if( ( rtsFlowControl == RTS_FLOW_CONTROL_ENABLED ) && ( ctsFlowControl == FLOW_CONTROL_NONE ) )
            {
                *pFlowControlSetting = CELLULAR_MODULE_COMM_FLOW_CONTROL_RTS;
            }
            else if( ( rtsFlowControl != FLOW_CONTROL_NONE ) && ( ctsFlowControl == CTS_FLOW_CONTROL_ENABLED ) )
            {
                *pFlowControlSetting = CELLULAR_MODULE_COMM_FLOW_CONTROL_CTS;
            }
            else if( ( rtsFlowControl == FLOW_CONTROL_NONE ) && ( ctsFlowControl == FLOW_CONTROL_NONE ) )
            {
                *pFlowControlSetting = CELLULAR_MODULE_COMM_FLOW_CONTROL_NONE;
            }
            else
            {
                LogError( ( "Cellular_GetModuleFlowControlSetting: unexpected RTS and/or CTS setting, RTS: %ld, CTS: %ld",
                            rtsFlowControl, ctsFlowControl ) );
                *pFlowControlSetting = CELLULAR_MODULE_COMM_FLOW_CONTROL_UNKNOWN;
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

CellularError_t Cellular_GetModuleFlowControlSetting( CellularHandle_t cellularHandle,
                                                      CellularModuleCommFlowControl_t * flowControl )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetFlowControl =
    {
        "AT+IFC?",
        CELLULAR_AT_WITH_PREFIX,
        "+IFC",
        _Cellular_RecvFuncGetFlowControlSetting,
        flowControl,
        sizeof( *flowControl )
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( flowControl == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqGetFlowControl );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetModuleFlowControlSetting: couldn't retrieve flow control setting" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

CellularError_t Cellular_SetModuleFlowControlSetting( CellularHandle_t cellularHandle,
                                                      CellularModuleCommFlowControl_t * flowControl )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetFlowControl = {0};
    int rtsSetting = FLOW_CONTROL_NONE;
    int ctsSetting = FLOW_CONTROL_NONE;

    atReqSetFlowControl.pAtCmd = cmdBuf;
    atReqSetFlowControl.atCmdType = CELLULAR_AT_NO_RESULT;
    atReqSetFlowControl.pAtRspPrefix = NULL;
    atReqSetFlowControl.respCallback = NULL;
    atReqSetFlowControl.pData = NULL;
    atReqSetFlowControl.dataLen = 0;

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( ( flowControl == NULL ) || ( *flowControl == CELLULAR_MODULE_COMM_FLOW_CONTROL_UNKNOWN ) )
    {
        LogError( ( "Cellular_SetModuleFlowControlSetting : Bad parameter" ) );
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        /* Form the AT command. */

        if( ( *flowControl == CELLULAR_MODULE_COMM_FLOW_CONTROL_RTS ) ||
            ( *flowControl == CELLULAR_MODULE_COMM_FLOW_CONTROL_RTS_CTS ) )
        {
            rtsSetting = RTS_FLOW_CONTROL_ENABLED;
        }
        else
        {
            rtsSetting = FLOW_CONTROL_NONE;
        }

        if( ( *flowControl == CELLULAR_MODULE_COMM_FLOW_CONTROL_CTS ) ||
            ( *flowControl == CELLULAR_MODULE_COMM_FLOW_CONTROL_RTS_CTS ) )
        {
            ctsSetting = CTS_FLOW_CONTROL_ENABLED;
        }
        else
        {
            ctsSetting = FLOW_CONTROL_NONE;
        }

        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s%d,%d",
                           "AT+IFC=",
                           rtsSetting,
                           ctsSetting
                           );
        LogDebug( ( "Baud rate setting: %s ", cmdBuf ) );
        pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqSetFlowControl );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SetModuleFlowControlSetting: couldn't set flow control settings" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

/* FreeRTOS Cellular Library types. */
/* coverity[misra_c_2012_rule_8_13_violation] */
static CellularPktStatus_t _Cellular_RecvFuncGetBaudRateSetting( CellularContext_t * pContext,
                                                                 const CellularATCommandResponse_t * pAtResp,
                                                                 void * pData,
                                                                 uint16_t dataLen )
{
    char * pInputLine = NULL, * pToken = NULL;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    uint32_t * pBaudRateSetting = NULL;
    uint32_t tempValue = 0;

    if( pContext == NULL )
    {
        LogError( ( "Cellular_GetModuleBaudRateSetting: Invalid context" ) );
        pktStatus = CELLULAR_PKT_STATUS_FAILURE;
    }
    else if( ( pAtResp == NULL ) || ( pAtResp->pItm == NULL ) ||
             ( pAtResp->pItm->pLine == NULL ) ||
             ( pData == NULL ) || ( dataLen == sizeof(uint32_t) ) )
    {
        LogError( ( "Cellular_GetModuleBaudRateSetting: Invalid param" ) );
        pktStatus = CELLULAR_PKT_STATUS_BAD_PARAM;
    }
    else
    {
        pInputLine = pAtResp->pItm->pLine;
        pBaudRateSetting = ( uint32_t * ) pData;
        atCoreStatus = Cellular_ATRemovePrefix( &pInputLine );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATRemoveAllWhiteSpaces( pInputLine );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoui( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                LogDebug( ( "Baud rate setting: %lu", tempValue ) );
                *pBaudRateSetting = tempValue;
            }
            else
            {
                LogError( ( "Error in processing baud rate setting. Token %s", pToken ) );
            }
        }

        pktStatus = _Cellular_TranslateAtCoreStatus( atCoreStatus );
    }

    return pktStatus;
}

CellularError_t Cellular_GetModuleBaudRateSetting( CellularHandle_t cellularHandle,
                                                   uint32_t * baudRate )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularAtReq_t atReqGetBaudRate =
    {
        "AT+IPR?",
        CELLULAR_AT_WITH_PREFIX,
        "+IPR",
        _Cellular_RecvFuncGetBaudRateSetting,
        baudRate,
        sizeof( *baudRate ),
    };

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogDebug( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else if( baudRate == NULL )
    {
        cellularStatus = CELLULAR_BAD_PARAMETER;
    }
    else
    {
        pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqGetBaudRate );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_GetModuleBaudRateSetting: couldn't retrieve baud rate setting" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}

/*-----------------------------------------------------------*/

CellularError_t Cellular_SetModuleBaudRateSetting( CellularHandle_t cellularHandle,
                                                   uint32_t baudRate )
{
    CellularContext_t * pContext = ( CellularContext_t * ) cellularHandle;
    CellularError_t cellularStatus = CELLULAR_SUCCESS;
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    char cmdBuf[ CELLULAR_AT_CMD_MAX_SIZE ] = { '\0' };
    CellularAtReq_t atReqSetBaudRate = {0 };

    atReqSetBaudRate.pAtCmd = cmdBuf;
    atReqSetBaudRate.atCmdType = CELLULAR_AT_NO_RESULT;
    atReqSetBaudRate.pAtRspPrefix = NULL;
    atReqSetBaudRate.respCallback = NULL;
    atReqSetBaudRate.pData = NULL;
    atReqSetBaudRate.dataLen = 0;

    cellularStatus = _Cellular_CheckLibraryStatus( pContext );

    if( cellularStatus != CELLULAR_SUCCESS )
    {
        LogError( ( "_Cellular_CheckLibraryStatus failed" ) );
    }
    else
    {
        /* Form the AT command. */

        /* MISRA Ref 21.6.1 [Use of snprintf] */
        /* More details at: https://github.com/FreeRTOS/FreeRTOS-Cellular-Interface/blob/main/MISRA.md#rule-216 */
        /* coverity[misra_c_2012_rule_21_6_violation]. */
        ( void ) snprintf( cmdBuf, CELLULAR_AT_CMD_MAX_SIZE, "%s%lu",
                           "AT+IPR=",
                           baudRate );
        LogDebug( ( "Baud rate setting: %s ", cmdBuf ) );
        pktStatus = _Cellular_AtcmdRequestWithCallback(pContext, atReqSetBaudRate );

        if( pktStatus != CELLULAR_PKT_STATUS_OK )
        {
            LogError( ( "Cellular_SetModuleBaudRateSetting: couldn't set baud rate" ) );
            cellularStatus = _Cellular_TranslatePktStatus( pktStatus );
        }
    }

    return cellularStatus;
}
