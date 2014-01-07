#ifndef __LM3642_H__
#define __LM3642_H__
#include <linux/types.h>
#include <linux/ioctl.h>

struct lm3642_pin{
	const char *name;
	unsigned gpio;
	unsigned init_state;
};

struct lm3642_platform_data {
	int 		num_leds;
	struct lm3642_pin *leds;
};

#define TORCH_LED_OFF (0)
#define TORCH_LED_ON   (1)

#define FLASH_LED_OFF (0)
#define FLASH_LED_ON   (1)

#define TORCH_LED_GET_STATE _IOR('o', 51, __u8)
#define TORCH_LED_SET_STATE _IOW('o', 52, __u8)
#define FLASH_LED_GET_STATE _IOR('o', 53, __u8)
#define FLASH_LED_SET_STATE _IOW('o', 54, __u8)

#define DISABLE_LED _IOW('o', 55, __u8)
#define ENABLE_LED _IOW('o', 56, __u8)

#define LM3556_IOCTL_MODE_SHUTDOWN	_IO('o', 61)
#define LM3556_IOCTL_MODE_STANDBY	_IO('o', 62)
#define LM3556_IOTCL_MODE_TORCH	_IOW('o', 63, __u32)
#define LM3556_IOCTL_MODE_FLASH	_IOW('o', 64, __u32)
#endif

