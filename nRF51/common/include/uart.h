/*----------------------------------------------------------------------------*/
/*  uart.h   for debug log output                                             */
/*----------------------------------------------------------------------------*/
#ifndef UART_H
#define UART_H

#if defined(USE_DBGLOG)

#include <stdint.h>

void    uart_putc( uint8_t ch );
void    uart_puts( uint8_t * str );
void    uart_init( void );

#else /* USE_DBGLOG */

#define uart_putc(ch)
#define uart_puts(str)
#define uart_init( )

#endif /* USE_DBGLOG */


#endif  /* UART_H */
