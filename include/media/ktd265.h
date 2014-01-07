
#ifndef __KTD265_H__
#define __KTD265_H__

#include <linux/ioctl.h> /* For IOCTL macros */
#define TORCH_LED_OFF (0)
#define TORCH_LED_ON   (1)

#define FLASH_LED_OFF (0)
#define FLASH_LED_ON   (1)

//camera io cotrl 51-70
#define TORCH_LED_GET_STATE _IOR('o', 51, __u8)
#define TORCH_LED_SET_STATE _IOW('o', 52, __u8)
#define FLASH_LED_GET_STATE _IOR('o', 53, __u8)
#define FLASH_LED_SET_STATE _IOW('o', 54, __u8)

#define DISABLE_LED _IOW('o', 55, __u8)
#define ENABLE_LED _IOW('o', 56, __u8)


#define KTD265_IOCTL_MODE_SHUTDOWN	_IOW('o', 61, __u8)
#define KTD265_IOCTL_MODE_STANDBY	_IOW('o', 62, __u8)
#define KTD265_IOTCL_MODE_TORCH	_IOW('o', 63, __u8)
#define KTD265_IOCTL_MODE_FLASH	_IOW('o', 64, __u8)
#define KTD265_IOCTL_MODE_LED		_IOW('o', 65, __u8)
#define KTD265_IOCTL_STRB		_IOW('o', 66, __u8)
#define KTD265_IOCTL_TIMER		_IOW('o', 67, __u8)

struct flash_led{
	const char *name;
	unsigned 	index;
	unsigned gpio;
	unsigned init_state;
};

struct flash_led_platform_data {
	int 		num_leds;
	struct flash_led *leds;
};

#endif

