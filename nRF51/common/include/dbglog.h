/*----------------------------------------------------------------------------*/
/*  dbglog.h   for debug log output                                           */
/*----------------------------------------------------------------------------*/
#ifndef DBGLOG_H
#define DBGLOG_H

#if defined(USE_DBGLOG)

#include <stdio.h>

#define PRINTF   printf
#define PUTS     puts

#else /* USE_DBGLOG */

#define PRINTF(...)
#define PUTS(s)

#endif /* USE_DBGLOG */

#endif  /* DBGLOG_H */
