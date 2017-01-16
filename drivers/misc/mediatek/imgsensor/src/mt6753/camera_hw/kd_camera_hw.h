#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_

#include <linux/types.h>

#include <mt-plat/mt_gpio.h>

#if defined CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include "pmic_drv.h"

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif

/*  */
/* Analog */
#define CAMERA_POWER_VCAM_A         PMIC_APP_MAIN_CAMERA_POWER_A
/* Digital */
#define CAMERA_POWER_VCAM_D         PMIC_APP_MAIN_CAMERA_POWER_D
/* AF */
#define CAMERA_POWER_VCAM_AF        PMIC_APP_MAIN_CAMERA_POWER_AF
/* digital io */
#define CAMERA_POWER_VCAM_IO        PMIC_APP_MAIN_CAMERA_POWER_IO
/* Digital for Sub */
#define SUB_CAMERA_POWER_VCAM_D     PMIC_APP_SUB_CAMERA_POWER_D

/* FIXME, should defined in DCT tool */
/* Main sensor */
#define CAMERA_CMRST_PIN            GPIO_CAMERA_CMRST_PIN
#define CAMERA_CMRST_PIN_M_GPIO     GPIO_CAMERA_CMRST_PIN_M_GPIO

#define CAMERA_CMPDN_PIN            GPIO_CAMERA_CMPDN_PIN
#define CAMERA_CMPDN_PIN_M_GPIO     GPIO_CAMERA_CMPDN_PIN_M_GPIO

/* FRONT sensor */
#define CAMERA_CMRST1_PIN           GPIO_CAMERA_CMRST1_PIN
#define CAMERA_CMRST1_PIN_M_GPIO    GPIO_CAMERA_CMRST1_PIN_M_GPIO

#define CAMERA_CMPDN1_PIN           GPIO_CAMERA_CMPDN1_PIN
#define CAMERA_CMPDN1_PIN_M_GPIO    GPIO_CAMERA_CMPDN1_PIN_M_GPIO

/* Define I2C Bus Num */
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        0
#else

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000
//flash
#define GPIO42  42
#define GPIO43  43
#define GPIO51  51
#define GPIO52  52

#define GPIO45 45
#define GPIO46 46
#define GPIO69 69
#define GPIO82 82
#define GPIO124 124
#define GPIO125 125
#define GPIO126 126
#define GPIO127 127
#define GPIO_MODE_00  0


#define GPIO_CAMERA_CMRST_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CAMERA_CMPDN_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CAMERA_CMRST1_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CAMERA_CMPDN1_PIN_M_GPIO   GPIO_MODE_00


#define CAMERA_CMRST_PIN            (GPIO125 | 0x80000000) 
#define CAMERA_CMRST_PIN_M_GPIO     GPIO_CAMERA_CMRST_PIN_M_GPIO

#define CAMERA_CMPDN_PIN            (GPIO124 | 0x80000000) 
#define CAMERA_CMPDN_PIN_M_GPIO     GPIO_CAMERA_CMPDN_PIN_M_GPIO

//FRONT sensor
#define CAMERA_CMRST1_PIN           (GPIO127 | 0x80000000) 
#define CAMERA_CMRST1_PIN_M_GPIO    GPIO_CAMERA_CMRST1_PIN_M_GPIO

#define CAMERA_CMPDN1_PIN           (GPIO126 | 0x80000000) 
#define CAMERA_CMPDN1_PIN_M_GPIO    GPIO_CAMERA_CMPDN1_PIN_M_GPIO

#define CAMERA_MCLK0				(GPIO45 | 0x80000000) 
#define CAMERA_MCLK1				(GPIO46 | 0x80000000) 
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

// Define I2C Bus Num
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        0
//#define GPIO_CAMERA_SUB_AVDD		 (GPIO82 | 0x80000000) //k5:share avdd with main camera
#define GPIO_CAMERA_SUB_DVDD		 (GPIO82 | 0x80000000) 
//#define GPIO_CAMERA_SUB_DOVDD	 (GPIO95 | 0x80000000)  //k5:share avdd with main camera
#define GPIO_CAM_SWSEL	                   (GPIO69 | 0x80000000)   //CAM_SWSEL

#endif /* End of #if defined CONFIG_MTK_LEGACY */


typedef enum KD_REGULATOR_TYPE_TAG {
	VCAMA,
	VCAMD,
	VCAMIO,
	VCAMAF,
} KD_REGULATOR_TYPE_T;

typedef enum {
	CAMPDN,
	CAMRST,
	CAM1PDN,
	CAM1RST,
	CAMLDO
	/*lenovo.sw wuyt3 add  for k5 camera*/
	,CAMMCLK
	/*lenovo.sw wuyt3 add end*/
} CAMPowerType;

extern void ISP_MCLK1_EN(bool En);
extern void ISP_MCLK2_EN(bool En);
extern void ISP_MCLK3_EN(bool En);

extern bool _hwPowerDown(KD_REGULATOR_TYPE_T type);
extern bool _hwPowerOn(KD_REGULATOR_TYPE_T type, int powerVolt);

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val);
int mtkcam_gpio_init(struct platform_device *pdev);

#endif
