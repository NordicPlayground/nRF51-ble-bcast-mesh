#ifndef BOOTLOADER_RTC_H__
#define BOOTLOADER_RTC_H__
#include <stdint.h>

#define RTC_MASK                    (0xFFFFFF)
#define US_TO_RTC_TICKS(time_us)    (((uint64_t) (time_us) * 32768ULL) / 1000000ULL)

#define RTC_TRANSPORT_CH            (0)
#define RTC_BL_STATE_CH             (1)

void rtc_init(void);

#endif /* BOOTLOADER_RTC_H__ */
