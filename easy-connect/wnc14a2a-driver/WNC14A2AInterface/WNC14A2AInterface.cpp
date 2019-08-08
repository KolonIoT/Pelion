/**
* copyright (c) 2017-2018, James Flynn
* SPDX-License-Identifier: Apache-2.0
*/

/* 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
 
/**
*   @file   WNC14A2AInterface.cpp
*   @brief  WNC14A2A implementation of NetworkInterfaceAPI for Mbed OS
*
*   @author James Flynn
*
*   @date   1-Feb-2018
*/

#include "WNC14A2AInterface.h"
#include <Thread.h>
#include "mbed_events.h"
#include "WNCIO.h"

#include <string> 
#include <ctype.h>

#define WNC_DEBUG   0           //1=enable the WNC startup debug output
                                //0=disable the WNC startup debug output
#define STOP_ON_FE  1           //1=hang forever if a fatal error occurs
                                //0=simply return failed response for all socket calls
#define DISPLAY_FE  1           //1 to display the fatal error when it occurs
                                //0 to NOT display the fatal error
#define RESETON_FE  0           //1 to cause the MCU to reset on fatal error
                                //0 to NOT reset the MCU

/** Error Handling macros & data
*   @brief  The macros CHK_WNCFE is used to check if a fatal error has occured. If it has
*           then execute the action specified: fail, void, null, resume
*
*    CHK_WNCFE( condition-to-check, fail|void|null|resume )
*
*     'fail'   if you want FATAL_WNC_ERROR to be called.  
*     'void'   if you want to execute a void return
*     'null'   if you want to execute a null return
*     'resume' if you simply want to resume program execution
*
*  There are several settings that control how FATAL_WNC_ERROR behaves:
*      1) RESETON_FE determines if the system will reset or hang.
*      2) DISPLAY_FE determine if an error message is generated or not
*
*  The DISPLAY_FE setting determines if a failure message is displayed. 
*  If set to 1, user sees this messageo:
*
*      WNC FAILED @ source-file-name:source-file-line-number
*
*  if not set, nothing is displayed.
*/

#define FATAL_FLAG  WncController::WNC_NO_RESPONSE
#define WNC_GOOD    WncController::WNC_ON

#define RETfail return -1
#define RETvoid return
#define RETnull return NULL
#define RETresume   

#define DORET(x) RET##x

#define TOSTR(x) #x
#define INTSTR(x) TOSTR(x)
#define FATAL_STR (char*)(__FILE__ ":" INTSTR(__LINE__))

#if RESETON_FE == 1   //reset on fatal error
#define MCURESET     ((*((volatile unsigned long *)0xE000ED0CU))=(unsigned long)((0x5fa<<16) | 0x04L))
#define RSTMSG       "RESET MCU! "
#else
#define MCURESET
#define RSTMSG       ""
#endif

#if DISPLAY_FE == 1  //display fatal error message
#define PFE     {if(_debugUart)_debugUart->printf((char*)RSTMSG "\r\n>>WNC FAILED @ %s\r\n", FATAL_STR);}
#else
#define PFE
#endif

#if STOP_ON_FE == 1  //halt cpu on fatal error
#define FATAL_WNC_ERROR(v)  {_fatal_err_loc=FATAL_STR;PFE;MCURESET;while(1);}
#else
#define FATAL_WNC_ERROR(v)  {_fatal_err_loc=FATAL_STR;PFE;DORET(v);}
#endif

#define CHK_WNCFE(x,y)    if( x ){FATAL_WNC_ERROR(y);}

//
// Define different levels of debug output
//
#define DBGMSG_DRV	               0x04                     //driver enter/exit info
#define DBGMSG_EQ	               0x08                     //driver event queue info
#define DBGMSG_SMS	               0x10                     //driver SMS info
#define DBGMSG_ARRY                    0x20                     //dump driver arrays

#define WNC14A2A_READ_TIMEOUTMS        4000                     //duration to read no data to receive in MS
#define WNC14A2A_COMMUNICATION_TIMEOUT 100                      //how long (ms) to wait for a WNC14A2A connect response
#define WNC_BUFF_SIZE                  1500                     //total number of bytes in a single WNC call
#define UART_BUFF_SIZE                 4096                     //size of our internal uart buffer.. define in *.json file

#define EQ_FREQ                        250                      //frequency in ms to check for Tx/Rx data
#define EQ_FREQ_SLOW                   2000                     //frequency in ms to check when in slow monitor mode

//
// The WNC device does not generate interrutps on received data, so this software polls 
// for data availablility.  To implement a non-blocking mode, simulate interrupts using
// mbed OS Event Queues.  These Constants are used to manage the Rx/Tx states.
//
#define READ_INIT                      10
#define READ_START                     11
#define READ_ACTIVE                    12
#define DATA_AVAILABLE                 13
#define TX_IDLE                        20
#define TX_STARTING                    21
#define TX_ACTIVE                      22
#define TX_COMPLETE                    23

#if MBED_CONF_APP_WNC_DEBUG == true
#define debugOutput(...)      WNC14A2AInterface::_dbOut(__VA_ARGS__)
#define debugDump_arry(...)   WNC14A2AInterface::_dbDump_arry(__VA_ARGS__)
#else
#define debugOutput(...)      {/* __VA_ARGS__ */}
#define debugDump_arry(...)   {/* __VA_ARGS__ */}
#endif

/*   Constructor
*
*  @brief    May be invoked with or without the debug pointer.
*  @note     After the constructor has completed, call check 
*  m_errors to determine if any errors occured. Possible values:
*           NSAPI_ERROR_UNSUPPORTED 
*           NSAPI_ERROR_DEVICE_ERROR
*/
WNC14A2AInterface::WNC14A2AInterface(WNCDebug *dbg) : 
 m_wncpoweredup(0),
 m_debug(0),
 m_pwnc(NULL),
 m_errors(NSAPI_ERROR_OK),
 m_smsmoning(0),
 _active_socket(0),
 mdmUart(MBED_CONF_WNC14A2A_LIBRARY_WNC_TXD,MBED_CONF_WNC14A2A_LIBRARY_WNC_RXD,115200),
 wnc_io(&mdmUart)
{
    _debugUart = dbg;           
    memset(_mac_address,0x00,sizeof(_mac_address));
    memset(_socTxS,0x00,sizeof(_socTxS));
    memset(_socRxS,0x00,sizeof(_socRxS));
    for( unsigned int i=0; i<WNC14A2A_SOCKET_COUNT; i++ ) {
        _sockets[i].socket = i;
        _sockets[i].addr = NULL;
        _sockets[i].opened=false;

        _sockets[i].connected=false;
        _sockets[i].proto=1;
        _socRxS[i].m_rx_socket=i;
        _socTxS[i].m_tx_socket=i;
        }
}

//! Standard destructor
WNC14A2AInterface::~WNC14A2AInterface()
{
    if( m_pwnc )
        delete m_pwnc;  //free the existing WncControllerK64F object
}

// - - - - - - - 
// SMS Functions
// - - - - - - - 

char* WNC14A2AInterface::getSMSnbr( void ) 
{
    char * ret=NULL;
    string iccid_str;
    static string msisdn_str;

    if( !m_pwnc ) {
        m_errors=NSAPI_ERROR_DEVICE_ERROR;
        return NULL;
        }
    CHK_WNCFE(( m_pwnc->getWncStatus() == FATAL_FLAG ), null);

    _pwnc_mutex.lock();
    if( !m_pwnc->getICCID(&iccid_str) ) {
        _pwnc_mutex.unlock();
        return ret;
        }
 
    if( m_pwnc->convertICCIDtoMSISDN(iccid_str, &msisdn_str) )
         ret = (char*)msisdn_str.c_str();    
    _pwnc_mutex.unlock();
    return ret;
}

void WNC14A2AInterface::sms_attach(void (*callback)(IOTSMS *))
{
    debugOutput("ENTER/EXIT sms_attach()");
    _sms_cb = callback;
}

void WNC14A2AInterface::sms_start(void)
{
    _pwnc_mutex.lock();                     
    m_pwnc->deleteSMSTextFromMem('*');       
    _pwnc_mutex.unlock();
}

void WNC14A2AInterface::sms_listen(uint16_t pp)
{
    debugOutput("ENTER sms_listen(%d)",pp);

    if( m_smsmoning )
        m_smsmoning = false;
    if( pp < 1)
        pp = 30;

    debugOutput("setup sms_listen event queue");
    _smsThread.start(callback(&sms_queue,&EventQueue::dispatch_forever));

    sms_start();
    sms_queue.call_every(pp*1000, mbed::Callback<void()>((WNC14A2AInterface*)this,&WNC14A2AInterface::handle_sms_event));

    m_smsmoning = true;
    debugOutput("EXIT sms_listen()");
}

void WNC14A2AInterface::handle_sms_event()
{
    int msgs_available;
    debugOutput("ENTER handle_sms_event()");

    if ( _sms_cb && m_smsmoning ) {
        _pwnc_mutex.lock();
        msgs_available = m_pwnc->readUnreadSMSText(&m_smsmsgs, true);
        _pwnc_mutex.unlock();
        if( msgs_available ) {
            debugOutput("Have %d unread texts present",m_smsmsgs.msgCount);
            for( int i=0; i< m_smsmsgs.msgCount; i++ ) {
                m_MsgText.number = m_smsmsgs.e[i].number;
                m_MsgText.date = m_smsmsgs.e[i].date;
                m_MsgText.time = m_smsmsgs.e[i].time;
                m_MsgText.msg = m_smsmsgs.e[i].msg;
                _sms_cb(&m_MsgText);
                }
            }
        }
    debugOutput("EXIT handle_sms_event");
}

int WNC14A2AInterface::getSMS(IOTSMS **pmsg) 
{
    int msgs_available=0;

    debugOutput("ENTER getSMS()");
    if( !m_pwnc ) 
        m_errors=NSAPI_ERROR_DEVICE_ERROR;
    else{
        CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);
        _pwnc_mutex.lock();
        msgs_available = m_pwnc->readUnreadSMSText(&m_smsmsgs, true);
        _pwnc_mutex.unlock();
        }

    if( msgs_available ) {
        debugOutput("Have %d unread texts present",m_smsmsgs.msgCount);
        for( int i=0; i< m_smsmsgs.msgCount; i++ ) {
            m_MsgText_array[i].number = m_smsmsgs.e[i].number;
            m_MsgText_array[i].date   = m_smsmsgs.e[i].date;
            m_MsgText_array[i].time   = m_smsmsgs.e[i].time;
            m_MsgText_array[i].msg    = m_smsmsgs.e[i].msg;
            pmsg[i] = (IOTSMS*)&m_MsgText_array[i];
            }
        msgs_available = m_smsmsgs.msgCount;
        }
    debugOutput("EXIT getSMS");
    return msgs_available;
}


int WNC14A2AInterface::sendIOTSms(const string& number, const string& message) 
{
    debugOutput("ENTER sendIOTSms(%s,%s)",number.c_str(), message.c_str());

    if( !m_pwnc ) 
        return (m_errors=NSAPI_ERROR_DEVICE_ERROR);
    CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);

    _pwnc_mutex.lock();
    int i =  m_pwnc->sendSMSText((char*)number.c_str(), message.c_str());
    _pwnc_mutex.unlock();

    debugOutput("EXIT sendIOTSms(%s,%s)",number.c_str(), message.c_str());
    return i;
}


// - - - - - - - - - - -
// WNC Control Functions
// - - - - - - - - - - -

nsapi_error_t WNC14A2AInterface::connect()   //can be called with no arguments or with arguments
{
    debugOutput("ENTER connect(void)");
    return connect(NULL,NULL,NULL);
}

nsapi_error_t WNC14A2AInterface::connect(const char *apn, const char *username, const char *password) 
{
    //
    // GPIO Pins used to initialize the WNC parts on the Avnet WNC Shield
    //
    // on powerup, 0 = boot mode, 1 = normal boot
    // 0=let modem sleep, 1=keep modem awake -- Note: pulled high on shield
    // active high
    // 0 = disabled (all signals high impedence, 1 = translation active
    // WNC doesn't utilize RTS/CTS but the pin is connected

    static DigitalOut  mdm_uart2_rx_boot_mode_sel(MBED_CONF_WNC14A2A_LIBRARY_WNC_RX_BOOT_SEL);
    static DigitalOut  mdm_power_on(MBED_CONF_WNC14A2A_LIBRARY_WNC_POWER_ON);
    static DigitalOut  mdm_wakeup_in(MBED_CONF_WNC14A2A_LIBRARY_WNC_WAKEUP);
    static DigitalOut  mdm_reset(MBED_CONF_WNC14A2A_LIBRARY_WNC_RESET);
    static DigitalOut  shield_3v3_1v8_sig_trans_ena(MBED_CONF_WNC14A2A_LIBRARY_WNC_LVLTRANSLATOR);
    static DigitalOut  mdm_uart1_cts(MBED_CONF_WNC14A2A_LIBRARY_WNC_CTS);

    //! associations for the controller class to use. Order of pins is critical.
    static WncControllerK64F_fk::WncGpioPinListK64F wncPinList = { 
        &mdm_uart2_rx_boot_mode_sel,
        &mdm_power_on,
        &mdm_wakeup_in,
        &mdm_reset,
        &shield_3v3_1v8_sig_trans_ena,
        &mdm_uart1_cts
    };

    debugOutput("ENTER connect(apn,user,pass)");

    if( m_pwnc == NULL ) {
        m_pwnc = new WncControllerK64F_fk::WncControllerK64F(&wncPinList, &wnc_io, _debugUart);
        if( !m_pwnc ) {
            debugOutput("FAILED to open WncControllerK64!");
            m_errors = NSAPI_ERROR_DEVICE_ERROR;
            return NSAPI_ERROR_NO_MEMORY;
            }
        CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);
        #if MBED_CONF_APP_WNC_DEBUG == true
        m_pwnc->enableDebug( (MBED_CONF_APP_WNC_DEBUG_SETTING&1), (MBED_CONF_APP_WNC_DEBUG_SETTING&2) );
        #endif
        }

    _eqThread.start(callback(&wnc_queue,&EventQueue::dispatch_forever));

    if (!apn)
        apn = "m2m.com.attz";

    _pwnc_mutex.lock();
    if (!m_wncpoweredup) {
        debugOutput("call powerWncOn(%s,40)",apn);
        m_wncpoweredup=m_pwnc->powerWncOn(apn,40);
        m_errors = m_wncpoweredup? 1:0;
        }
    else {          //powerWncOn already called, set a new APN
        debugOutput("set APN=%s",apn);
        m_errors = m_pwnc->setApnName(apn)? 1:0;
        }

    m_errors |= m_pwnc->getWncNetworkingStats(&myNetStats)? 2:0;
    _pwnc_mutex.unlock();

    debugOutput("EXIT connect (%02X)",m_errors);
    return (!m_errors)? NSAPI_ERROR_NO_CONNECTION : NSAPI_ERROR_OK;
}

const char* WNC14A2AInterface::getWNCRev(void)
{
    if( m_pwnc ) {
        const char * str = m_pwnc->getFirmRev();
        return &str[12];
        }
    else
        return NULL;
}


const char *WNC14A2AInterface::get_ip_address()
{
    const char *ptr=NULL; 

    if( !m_pwnc ) {
        m_errors=NSAPI_ERROR_DEVICE_ERROR;
        return ptr;
        }
    CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), null);

    _pwnc_mutex.lock();
    if ( m_pwnc->getWncNetworkingStats(&myNetStats) ) {
        _pwnc_mutex.unlock();
        ptr = &myNetStats.ip[0];
        }
    else{
        _pwnc_mutex.unlock();
        m_errors=NSAPI_ERROR_NO_CONNECTION;
        }
    return ptr;
}
 
const char *WNC14A2AInterface::get_mac_address()
{
    string mac, str;
    debugOutput("ENTER get_mac_address()");

    if( m_pwnc ) {
        CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), null);
        _pwnc_mutex.lock();
        if( m_pwnc->getICCID(&str) ) {
            _pwnc_mutex.unlock();
            mac = str.substr(3,20);
            mac[2]=mac[5]=mac[8]=mac[11]=mac[14]=':';
            strncpy(_mac_address, mac.c_str(), mac.length());
            debugOutput("EXIT get_mac_address() - %s",_mac_address);
            return _mac_address;
            }
        _pwnc_mutex.unlock();
        }
    debugOutput("EXIT get_mac_address() - NULL");
    return NULL;
}

NetworkStack *WNC14A2AInterface::get_stack() {
    debugOutput("ENTER/EXIT get_stack()");
    return this;
}

nsapi_error_t WNC14A2AInterface::disconnect() 
{
    debugOutput("ENTER/EXIT disconnect()");
    return NSAPI_ERROR_OK;
}

nsapi_error_t WNC14A2AInterface::set_credentials(const char *apn, const char *username, const char *password) 
{

    m_errors=NSAPI_ERROR_OK;
    debugOutput("ENTER set_credentials()");

    if( !m_pwnc ) 
        return (m_errors=NSAPI_ERROR_DEVICE_ERROR);
    CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);
        
    if( !apn )
        return (m_errors=NSAPI_ERROR_PARAMETER);

    _pwnc_mutex.lock();
    if( !m_pwnc->setApnName(apn) )
        m_errors=NSAPI_ERROR_DEVICE_ERROR;
    _pwnc_mutex.unlock();
    debugOutput("EXIT set_credentials()");
    return m_errors;
}

bool WNC14A2AInterface::registered()
{
    debugOutput("ENTER registered()");
    m_errors=NSAPI_ERROR_OK;

    if( !m_pwnc ) {
        return (m_errors=NSAPI_ERROR_DEVICE_ERROR);
        }
    CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);

    _pwnc_mutex.lock();
    if ( m_pwnc->getWncStatus() != WNC_GOOD )
        m_errors=NSAPI_ERROR_NO_CONNECTION;
    _pwnc_mutex.unlock();

    debugOutput("EXIT registered()");
    return (m_errors==NSAPI_ERROR_OK);
}


void WNC14A2AInterface::doDebug( int v )
{
    #if MBED_CONF_APP_WNC_DEBUG == true
    m_debug= v;
    debugOutput("SET debug flag to 0x%02X",v);
    #endif
}

/** function to dump a user provided array.
*
* @author James Flynn
* @param  data    pointer to the data array to dump
* @param  size    number of bytes to dump
* @return void
* @date 1-Feb-2018
*/
void WNC14A2AInterface::_dbDump_arry( const uint8_t* data, unsigned int size )
{
    #if MBED_CONF_APP_WNC_DEBUG == true
    char buffer[256];
    unsigned int i, k;

    if( _debugUart != NULL && (m_debug & DBGMSG_ARRY) ) {
        for (i=0; i<size; i+=16) {
            sprintf(buffer,"[WNC Driver]:0x%04X: ",i);
            _debugUart->puts(buffer);
            for (k=0; k<16; k++) {
                sprintf(buffer, "%02X ", data[i+k]);
                _debugUart->puts(buffer);
                }
            _debugUart->puts(" -- ");
            for (k=0; k<16; k++) {
                sprintf(buffer, "%2c", isprint(data[i+k])? data[i+k]:'.');
                _debugUart->puts(buffer);
                }
            _debugUart->puts("\n\r");
            }
        }
    #endif
}

void WNC14A2AInterface::_dbOut(const char *format, ...) 
{
    #if MBED_CONF_APP_WNC_DEBUG == true
    char buffer[256];

    sprintf(buffer,"[WNC Driver]: ");
    if( _debugUart != NULL && (m_debug & (DBGMSG_DRV|DBGMSG_EQ|DBGMSG_SMS)) ) {
        va_list args;
        va_start (args, format);
        _debugUart->puts(buffer);
        if( m_debug & DBGMSG_DRV )
            vsnprintf(buffer, sizeof(buffer), format, args);
        if( m_debug & DBGMSG_EQ )
            vsnprintf(buffer, sizeof(buffer), format, args);
        if( m_debug & DBGMSG_SMS )
            vsnprintf(buffer, sizeof(buffer), format, args);
        _debugUart->puts(buffer);
        _debugUart->putc('\n');
        va_end (args);
        }
    #endif
}

// - - - - - - - - - - - - - - -
// WNC Socket Based operatioins
// - - - - - - - - - - - - - - -

nsapi_error_t WNC14A2AInterface::gethostbyname(const char* name, SocketAddress *address, nsapi_version_t version)
{
    nsapi_error_t ret = NSAPI_ERROR_OK;
    char ipAddrStr[25];

    debugOutput("ENTER gethostbyname(); IP=%s; PORT=%d; URL=%s;", address->get_ip_address(), address->get_port(), name);

    if( !m_pwnc ) 
        return (m_errors=NSAPI_ERROR_DEVICE_ERROR);
    CHK_WNCFE((m_pwnc->getWncStatus()==FATAL_FLAG), fail);

    memset(ipAddrStr,0x00,sizeof(ipAddrStr));

    //Execute DNS query.  
    _pwnc_mutex.lock();
    if( !m_pwnc->resolveUrl(_active_socket, name) )  
        ret = m_errors = NSAPI_ERROR_DEVICE_ERROR;

    //Get IP address that the URL was resolved to
    if( !m_pwnc->getIpAddr(_active_socket, ipAddrStr) )
        ret = m_errors = NSAPI_ERROR_DEVICE_ERROR;
    _pwnc_mutex.unlock();

    address->set_ip_address(ipAddrStr);

    debugOutput("EXIT gethostbyname(); IP=%s; PORT=%d; URL=%s;", address->get_ip_address(), address->get_port(), name);
    return (m_errors = ret);
}

int WNC14A2AInterface::socket_open(void **handle, nsapi_protocol_t proto) 
{
    unsigned int i;

    debugOutput("ENTER socket_open()");

    //find the next available socket...
    for( i=0; i<WNC14A2A_SOCKET_COUNT; i++ )
        if( !_sockets[i].opened )
            break;

    if( i == WNC14A2A_SOCKET_COUNT ) {
        m_errors=NSAPI_ERROR_NO_SOCKET;
        return -1;
        }

    _sockets[i].socket = i;            //save index later
    _sockets[i].opened = true;         
    _sockets[i].connected=false;
    _sockets[i].proto = (proto==NSAPI_UDP)?0:1;
    _sockets[i]._callback = NULL;
    _sockets[i]._cb_data = NULL;         

    _socRxS[i].m_rx_wnc_state = READ_START;
    _socRxS[i].m_rx_disTO = false;
    _socTxS[i].m_tx_wnc_state = TX_IDLE;

    *handle = &_sockets[i];
    
    debugOutput("EXIT socket_open; Socket=%d, OPEN=%s, protocol =%s",
                i, _sockets[i].opened?"YES":"NO", (!_sockets[i].proto)?"UDP":"TCP");
    
    return (m_errors = NSAPI_ERROR_OK);
}

int WNC14A2AInterface::socket_connect(void *handle, const SocketAddress &address) 
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;   
    int err = 0;

    debugOutput("ENTER socket_connect(); Socket=%d; IP=%s; PORT=%d;", wnc->socket, address.get_ip_address(), address.get_port());

    if (!wnc->opened ) 
        return (m_errors = NSAPI_ERROR_NO_SOCKET);

    wnc->addr = address;
                                
    _pwnc_mutex.lock();
    if( wnc->url.empty() ) 
        err = !m_pwnc->openSocketIpAddr(wnc->socket, address.get_ip_address(), address.get_port(), wnc->proto, WNC14A2A_COMMUNICATION_TIMEOUT);
     else 
        err = !m_pwnc->openSocketUrl(wnc->socket, wnc->url.c_str(), wnc->addr.get_port(), wnc->proto);
    _pwnc_mutex.unlock();

    if( !err ) {
        wnc->connected = true;
        _socRxS[wnc->socket].m_rx_wnc_state = READ_START;
        _socTxS[wnc->socket].m_tx_wnc_state = TX_IDLE;

        if( wnc->_callback != NULL ) 
            wnc->_callback( wnc->_cb_data );
        }

    return err;
}

int WNC14A2AInterface::socket_close(void *handle)
{
    WNCSOCKET *wnc = (WNCSOCKET*)handle;
    RXEVENT *rxsock;
    TXEVENT *txsock;
    bool err = false;

    debugOutput("ENTER socket_close()");

    rxsock = &_socRxS[wnc->socket];
    txsock = &_socTxS[wnc->socket];

    txsock->m_tx_wnc_state = TX_IDLE;               //reset TX state
    if( rxsock->m_rx_wnc_state != READ_START ) {    //reset RX state
        rxsock->m_rx_disTO=false;
        while( rxsock->m_rx_wnc_state !=  DATA_AVAILABLE ) 
            wait(1);  //someone called close while a read was happening
        }

    _pwnc_mutex.lock();
    if( !m_pwnc->closeSocket(wnc->socket) ) {
        m_errors = NSAPI_ERROR_DEVICE_ERROR;
        err = true;
        }
    _pwnc_mutex.unlock();

    if( !err ) {
        wnc->opened   = false;       //no longer in use
        wnc->addr     = NULL;        //not open
        wnc->connected= false;
        wnc->proto    = 1;           //assume TCP for now
        m_errors      = NSAPI_ERROR_OK;
        wnc->_cb_data = NULL;
        wnc->_callback= NULL;
        }

    debugOutput("EXIT socket_close()");
    return err;
}


void WNC14A2AInterface::socket_attach(void *handle, void (*callback)(void *), void *data)
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;

    debugOutput("ENTER/EXIT socket_attach()");
    wnc->_callback = callback;
    wnc->_cb_data = data;
}

int WNC14A2AInterface::socket_sendto(void *handle, const SocketAddress &address, const void *data, unsigned size)
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;
    
    debugOutput("ENTER socket_sendto()");
    
    if (!wnc->connected) {
       int err = socket_connect(wnc, address);
       if (err < 0) 
           return err;
       }
    wnc->addr = address;

    debugOutput("EXIT socket_sendto()");
    return socket_send(wnc, data, size);
}

int WNC14A2AInterface::socket_recvfrom(void *handle, SocketAddress *address, void *buffer, unsigned size)
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;
    debugOutput("ENTER socket_recvfrom()");

    if (!wnc->connected) {
       debugOutput("need to open a WNC socket first");
       int err = socket_connect(wnc, *address);
       if (err < 0) 
           return err;
       }

    int ret = socket_recv(wnc, (char *)buffer, size);
    if (ret >= 0 && address) 
        *address = wnc->addr;
    debugOutput("EXIT socket_recvfrom()");
    return ret;
}


int inline WNC14A2AInterface::socket_accept(nsapi_socket_t server, nsapi_socket_t *handle, SocketAddress *address) 
{
    debugOutput("ENTER/EXIT socket_accept() -- not supported");
    m_errors = NSAPI_ERROR_UNSUPPORTED;
    return -1;
}

int inline WNC14A2AInterface::socket_bind(void *handle, const SocketAddress &address) 
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;

    debugOutput("ENTER/EXIT socket_bind(), use address '%s', port %d", address.get_ip_address(),address.get_port());
    _socRxS[wnc->socket].m_rx_disTO=true;  //for us, simply disable the Rx timeout to keep monitoring for data
    return (m_errors = NSAPI_ERROR_OK);
}


int inline WNC14A2AInterface::socket_listen(void *handle, int backlog)
{
   debugOutput("ENTER/EXIT socket_listen() -- not supported");
    m_errors = NSAPI_ERROR_UNSUPPORTED;
    return -1;
}


int WNC14A2AInterface::socket_send(void *handle, const void *data, unsigned size) 
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;
    TXEVENT *txsock;

    debugOutput("ENTER socket_send() send %d bytes",size);
    txsock = &_socTxS[wnc->socket];

    if( size < 1 || data == NULL )  // should never happen but have seen it
        return 0; 

    switch( txsock->m_tx_wnc_state ) {
        case TX_IDLE:
            txsock->m_tx_wnc_state = TX_STARTING;
            debugDump_arry((const uint8_t*)data,size);
            txsock->m_tx_dptr      = (uint8_t*)data;
            txsock->m_tx_orig_size = size;
            txsock->m_tx_req_size  = (uint32_t)size;
            txsock->m_tx_total_sent= 0;
            txsock->m_tx_callback  = wnc->_callback;
            txsock->m_tx_cb_data   = wnc->_cb_data;

            if( txsock->m_tx_req_size > UART_BUFF_SIZE ) 
                txsock->m_tx_req_size= UART_BUFF_SIZE;

            if( !tx_event(txsock) ) {   //if we didn't sent all the data at once, continue in background
                txsock->m_tx_wnc_state = TX_ACTIVE;
                wnc_queue.call_in(EQ_FREQ,mbed::Callback<void()>((WNC14A2AInterface*)this,&WNC14A2AInterface::wnc_eq_event));
                return NSAPI_ERROR_WOULD_BLOCK;
                }
            // fall through 

        case TX_COMPLETE:
            debugOutput("EXIT socket_send(), sent %d bytes", txsock->m_tx_total_sent);
            txsock->m_tx_wnc_state = TX_IDLE;
            return txsock->m_tx_total_sent;

        case TX_ACTIVE:
        case TX_STARTING:
            return NSAPI_ERROR_WOULD_BLOCK;

        default:
            debugOutput("EXIT socket_send(), NSAPI_ERROR_DEVICE_ERROR");
            return (m_errors = NSAPI_ERROR_DEVICE_ERROR);
        }
}

int WNC14A2AInterface::tx_event(TXEVENT *ptr)
{
    debugOutput("ENTER tx_event(), socket %d",ptr->m_tx_socket);

    _pwnc_mutex.lock();
    if( m_pwnc->write(ptr->m_tx_socket, ptr->m_tx_dptr, ptr->m_tx_req_size) ) 
        ptr->m_tx_total_sent += ptr->m_tx_req_size;
    _pwnc_mutex.unlock();
    
    if( ptr->m_tx_total_sent < ptr->m_tx_orig_size ) {
        ptr->m_tx_dptr += ptr->m_tx_req_size;
        ptr->m_tx_req_size = ptr->m_tx_orig_size-ptr->m_tx_total_sent;

        if( ptr->m_tx_req_size > UART_BUFF_SIZE) 
            ptr->m_tx_req_size= UART_BUFF_SIZE;

        debugOutput("EXIT tx_event(), send %d more bytes.",ptr->m_tx_req_size);
        return 0;
        }
    debugOutput("EXIT tx_event, socket %d, data sent",ptr->m_tx_socket);
    ptr->m_tx_wnc_state = TX_COMPLETE;
    if( ptr->m_tx_callback != NULL ) 
        ptr->m_tx_callback( ptr->m_tx_cb_data );
    ptr->m_tx_cb_data = NULL; 
    ptr->m_tx_callback = NULL;

    return 1;
}

int WNC14A2AInterface::socket_recv(void *handle, void *data, unsigned size) 
{
    WNCSOCKET *wnc = (WNCSOCKET *)handle;
    RXEVENT *rxsock;

    rxsock = &_socRxS[wnc->socket];
    debugOutput("ENTER socket_recv(), socket %d, request %d bytes",wnc->socket, size);

    if( size < 1 || data == NULL ) { // should never happen
        return 0; 
        }

    switch( rxsock->m_rx_wnc_state ) {
        case READ_START:  //need to start a read sequence of events
            rxsock->m_rx_wnc_state= READ_INIT;
            rxsock->m_rx_dptr     = (uint8_t*)data;
            rxsock->m_rx_req_size = (uint32_t)size;
            rxsock->m_rx_total_cnt= 0;
            rxsock->m_rx_timer    = 0;
            rxsock->m_rx_return_cnt=0;

            if( rxsock->m_rx_req_size > WNC_BUFF_SIZE) 
                rxsock->m_rx_req_size= WNC_BUFF_SIZE;
                
            rxsock->m_rx_callback = wnc->_callback;
            rxsock->m_rx_cb_data  = wnc->_cb_data;

            if( !rx_event(rxsock) ){
                rxsock->m_rx_wnc_state = READ_ACTIVE;
                wnc_queue.call_in(EQ_FREQ,mbed::Callback<void()>((WNC14A2AInterface*)this,&WNC14A2AInterface::wnc_eq_event));
                return NSAPI_ERROR_WOULD_BLOCK;
                }
            // fall through 

        case DATA_AVAILABLE:
            debugOutput("EXIT socket_recv(),socket %d, return %d bytes",wnc->socket, rxsock->m_rx_return_cnt);
            debugDump_arry((const uint8_t*)data,rxsock->m_rx_return_cnt);
            rxsock->m_rx_wnc_state = READ_START;
            return rxsock->m_rx_return_cnt;

        case READ_ACTIVE:
        case READ_INIT:
            rxsock->m_rx_timer    = 0;  //reset the time-out timer
            return NSAPI_ERROR_WOULD_BLOCK;

        default:
            debugOutput("EXIT socket_recv(), NSAPI_ERROR_DEVICE_ERROR");
            return (m_errors = NSAPI_ERROR_DEVICE_ERROR);
        }
}


int WNC14A2AInterface::rx_event(RXEVENT *ptr)
{
    debugOutput("ENTER rx_event() for socket %d", ptr->m_rx_socket);
    _pwnc_mutex.lock();
    int cnt = m_pwnc->read(ptr->m_rx_socket, ptr->m_rx_dptr,  ptr->m_rx_req_size);
    _pwnc_mutex.unlock();

    if( cnt ) {  //got data, return it to the caller
        debugOutput("data received on socket %d, cnt=%d", ptr->m_rx_socket,cnt);
        ptr->m_rx_wnc_state = DATA_AVAILABLE;
        ptr->m_rx_return_cnt = cnt;
        if( ptr->m_rx_callback != NULL ) 
            ptr->m_rx_callback( ptr->m_rx_cb_data );
        ptr->m_rx_cb_data = NULL; 
        ptr->m_rx_callback = NULL;
        return 1;
        }
    if( ++ptr->m_rx_timer > (WNC14A2A_READ_TIMEOUTMS/EQ_FREQ) && !ptr->m_rx_disTO ) {  //timed out waiting, return 0 to caller
        debugOutput("EXIT rx_event(), rx data TIME-OUT!");
        ptr->m_rx_wnc_state = DATA_AVAILABLE;
        ptr->m_rx_return_cnt = 0;
        if( ptr->m_rx_callback != NULL ) 
            ptr->m_rx_callback( ptr->m_rx_cb_data );
        ptr->m_rx_cb_data = NULL; 
        ptr->m_rx_callback = NULL;
        return 1;
        }

    debugOutput("EXIT rx_event(), socket %d, sechedule for more data.",ptr->m_rx_socket);
    return 0;
}

void WNC14A2AInterface::wnc_eq_event()
{
    int done = 1;
    bool goSlow = true;

    for( unsigned int i=0; i<WNC14A2A_SOCKET_COUNT; i++ ) {
        if( _socRxS[i].m_rx_wnc_state == READ_ACTIVE || _socRxS[i].m_rx_disTO) {
            done &= rx_event(&_socRxS[i]);
            goSlow &= ( _socRxS[i].m_rx_timer > ((WNC14A2A_READ_TIMEOUTMS/EQ_FREQ)*(EQ_FREQ_SLOW/EQ_FREQ)) );

            if( goSlow ) 
                _socRxS[i].m_rx_timer = (WNC14A2A_READ_TIMEOUTMS/EQ_FREQ)*(EQ_FREQ_SLOW/EQ_FREQ);
            }

        if( _socTxS[i].m_tx_wnc_state == TX_ACTIVE ) {
            goSlow = false;
            debugOutput("CALL TX_event() for socket %d", i);
            done &= tx_event(&_socTxS[i]);
            }
        }

    if( !done )  
        wnc_queue.call_in((goSlow?EQ_FREQ_SLOW:EQ_FREQ),mbed::Callback<void()>((WNC14A2AInterface*)this,&WNC14A2AInterface::wnc_eq_event));
}
