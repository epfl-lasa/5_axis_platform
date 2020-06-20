#ifdef TARGET_NUCLEO_F401RE
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
#if DEVICE_SERIAL_ASYNCH
switch( _serial.serial.index ) {
#else
switch( _serial.index ) {
#endif
        case 0: _base = USART1; _IRQ = USART1_IRQn; break;
        case 1: _base = USART2; _IRQ = USART2_IRQn; break;
        case 2: _base = USART6; _IRQ = USART6_IRQn; break;
        default: _base = NULL; _IRQ = (IRQn_Type)NULL; break;
    }
}

void MODSERIAL::initDevice(void) {};

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( (((USART_TypeDef*)_base)->SR & ( 1UL << 6 )) == 0 ) ? true : false; 
} 

#endif