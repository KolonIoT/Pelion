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
*/

/**
*  @file WNCIO.h
*  @brief A class that WNCInterface uses for input/output
*
*  @author James Flynn
* 
*  @date 1-Feb-2018
*  
*/
 
#ifndef __WNCIO__
#define __WNCIO__
#include <stdio.h>
#include "mbed.h"

/** WncIO class
* Used to read/write the WNC UART using FILE I/O. 
*/

class WncIO
{
public:
    //! Create class with either stdio or a pointer to a uart
    WncIO( UARTSerial * uart): m_puart(uart) {;}
    ~WncIO() {};

    //! standard printf() functionallity
    int printf( char * fmt, ...) {
            char buffer[256];
            int ret=0;
            va_list args;
            va_start (args, fmt);
            vsnprintf(buffer, sizeof(buffer), fmt, args);
            prt.lock();
            ret=m_puart->write(buffer,strlen(buffer));
            prt.unlock();
            va_end (args);
            return ret;
            }

    //! standard putc() functionallity
    int putc( int c ) {
            int ret=0;
            prt.lock();
            ret=m_puart->write((const void*)&c,1);
            prt.unlock();
            return ret;
            }

    //! standard puts() functionallity
    int puts( const char * str ) {
            int ret=0;
            prt.lock();
            ret=m_puart->write(str,strlen(str));
            prt.unlock();
            return ret;
            }

    //! return true when data is available, false otherwise
    bool readable( void ) {
        return m_puart->readable();
        }

    //! get the next character available from the uart
    int getc( void ) {
        char c;
        m_puart->read( &c, 1 );
        return c;
        }

    //! set the uart baud rate
    void baud( int baud ) {
        m_puart->set_baud(baud);
        }

private:
    UARTSerial *m_puart;
    Mutex prt;
};

#endif

