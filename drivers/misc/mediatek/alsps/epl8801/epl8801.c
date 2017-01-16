/* drivers/hwmon/mt6516/amit/epl8801.c - EPL8801 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/input/mt.h>
#include <alsps.h>
#include "upmu_sw.h"
#include "upmu_common.h"
#include <linux/gpio.h>
#include <linux/of_irq.h>

#include <linux/wakelock.h>
#include <linux/sched.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <alsps.h>

#define MTK_LTE         1
#define MT6753

#include <cust_alsps.h>

//#include <linux/hwmsen_helper.h>
#include "epl8801.h"

#if 0
#if defined(MT6575) || defined(MT6571) || defined(MT6589) || defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
#endif
//add for fix resume issue
//#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
//add for fix resume issue end

/*lenovo-sw caoyi1 modify begin*/
#define COMPATIABLE_NAME "mediatek,epl8801"
/*lenovo-sw caoyi1 modify end*/

#define EINT_API    1

static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

/******************************************************************************
 * extern functions
*******************************************************************************/
#if !EINT_API
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
//extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
//                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
//                                     kal_bool auto_umask);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

extern int mt_eint_get_sens(unsigned int eint_num);
#endif /*EINT_API*/

/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
extern struct alsps_context *alsps_context_obj;
/*lenovo-sw caoyi1 add for als report data fast 20150624 end*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE

/******************************************************************************
 *  configuration
 ******************************************************************************/
#define LUX_PER_COUNT			300//700		// 0.7

#define COMMON_DEBUG    1
#define ALS_DEBUG       1
#define PS_DEBUG        1
#define SHOW_DBG        1
/******************************************************************************
*******************************************************************************/

#define ALS_DYN_INTT    1
/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
#define ALS_DYN_INTT_FAST   1
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/
/*lenovo-sw caoyi1 modify for close PS_DYN begin*/
#define PS_DYN_K_ONE    0
/*lenovo-sw caoyi1 modify for close PS_DYN end*/

/*lenovo-sw caoyi1 modify for PS calibration begin*/
#define LENOVO_CALI     1
/*lenovo-sw caoyi1 modify for PS calibration end*/

/*lenovo-sw caoyi1 add for MMI sensor test 20151020 begin*/
#define PS_AUTO_TEST   1
/*lenovo-sw caoyi1 add for MMI sensor test 20151020 end*/

#define TXBYTES 				2
#define RXBYTES					2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
#define EPL_DEV_NAME   		    "alsps"
#define DRIVER_VERSION          "1.2.0"

struct input_dev dev;
#if !MTK_LTE
struct hwmsen_object *ps_hw, * als_hw;
#endif
static struct epl_sensor_priv *epl_sensor_obj = NULL;
#if !MTK_LTE
static struct platform_driver epl_sensor_alsps_driver;
#endif
static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
bool polling_flag = true;
bool eint_flag = true;

static epl_optical_sensor epl_sensor;
int i2c_max_count=8;
#if !MTK_LTE
static bool first_als_report_flag = true;
#endif
//ps calibration file location
static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";

//als calibration file location
static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";

static int PS_h_offset = 3000;
static int PS_l_offset = 2000;
static int PS_MAX_XTALK = 30000;

int als_frame_time = 0;
int ps_frame_time = 0;

#if ALS_DYN_INTT
//Dynamic INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	//initial dynamic_intt_idx
int c_gain;
int dynamic_intt_lux = 0;

uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
/*lenovo-sw caoyi1 modify als data range 20150701 begin*/
uint32_t dynamic_intt_max_lux = 18000;
/*lenovo-sw caoyi1 modify als data range 20150701 end*/
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 1000;

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_8000, EPL_ALS_INTT_70};
static int als_dynamic_intt_value[] = {8000, 70};
/*lenovo-sw caoyi1 modify Lsensor value parameter 20150511 begin*/
static int als_dynamic_intt_gain[] = {EPL_GAIN_LOW, EPL_GAIN_LOW};
/*lenovo-sw caoyi1 modify Lsensor value parameter 20150511 end*/
static int als_dynamic_intt_high_thr[] = {60000, 53000};
static int als_dynamic_intt_low_thr[] = {200, 300};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
#endif


typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
} epl_raw_data;
static epl_raw_data	gRawData;

#if MTK_LTE
static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove(void);
static struct alsps_init_info epl_sensor_init_info = {
		.name = EPL_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,

};
#endif

#if PS_DYN_K_ONE
static bool ps_dyn_flag = false;
#define PS_MAX_CT	10000
#define PS_DYN_H_OFFSET 800
#define PS_DYN_L_OFFSET 500
u32 dynk_thd_low = 0;
u32 dynk_thd_high = 0;
#endif

static int wait_value[] = {0, 2, 4, 8, 12, 20, 30, 40, 50, 75, 100, 150, 200, 300, 400};
int wait_len = sizeof(wait_value)/sizeof(int);

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[ALS/PS] "
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl_sensor_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl_sensor_i2c_id[] = {{EPL_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_epl_sensor= { I2C_BOARD_INFO(EPL_DEV_NAME, (0x82>>1))};

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_sensor_i2c_remove(struct i2c_client *client);
//static int epl_sensor_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_sensor_i2c_resume(struct i2c_client *client);
#if EINT_API
static irqreturn_t epl_sensor_eint_func(int irq, void *desc);
#else
static void epl_sensor_eint_func(void);
#endif/*EINT_API*/
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
void epl_sensor_restart_polling(void);
void epl_sensor_update_mode(struct i2c_client *client);
int epl_sensor_read_ps(struct i2c_client *client);
int epl_sensor_read_als(struct i2c_client *client);
int epl_sensor_read_als_status(struct i2c_client *client);
static int als_sensing_time(int intt, int adc, int cycle);
static int ps_sensing_time(int intt, int adc, int cycle);
/*lenovo-sw caoyi1 add for MMI sensor test 20151020 begin*/
static void epl_sensor_report_ps_status(void);
/*lenovo-sw caoyi1 add for MMI sensor test 20151020 end*/

static struct epl_sensor_priv *g_epl8801_ptr;
static long long int_top_time;
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_DEF_LIGHT		= 0x4,
    CMC_BIT_TABLE			= 0x8,
} CMC_ALS_REPORT_TYPE;

#if ALS_DYN_INTT
typedef enum
{
    CMC_BIT_LSRC_NON		= 0x0,
    CMC_BIT_LSRC_SCALE     	= 0x1,
    CMC_BIT_LSRC_SLOPE		= 0x2,
    CMC_BIT_LSRC_BOTH       = 0x3,
} CMC_LSRC_REPORT_TYPE;
#endif
/*----------------------------------------------------------------------------*/
struct epl_sensor_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
    struct delayed_work  polling_work;
    struct input_dev *gs_input_dev;
    struct input_dev *hb_input_dev;

    /*i2c address group*/
    struct epl_sensor_i2c_addr  addr;

    /*misc*/
    atomic_t    trace;
    atomic_t   	als_suspend;
    atomic_t    ps_suspend;
    /*data*/
    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    ulong      	pending_intr;   	/*pending interrupt*/

    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
#if ALS_DYN_INTT
    uint32_t ratio;
    uint32_t last_ratio;
    int c_gain_h; // fluorescent (C1)
    int c_gain_l; // incandescent (C2)
    uint32_t lsource_thd_high; //different light source boundary (N) fluorescent (C1)
    uint32_t lsource_thd_low; //different light source boundary (N) incandescent (C2)
#endif
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
#if EINT_API
    struct device_node *irq_node;
    int		irq;
#endif
};
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl_sensor_i2c_driver =
{
    .probe     	= epl_sensor_i2c_probe,
    .remove     = epl_sensor_i2c_remove,
    //.detect     = epl_sensor_i2c_detect,
    .suspend    = epl_sensor_i2c_suspend,
    .resume     = epl_sensor_i2c_resume,
    .id_table   = epl_sensor_i2c_id,
    //.address_data = &epl_sensor_addr_data,
    .driver = {
        //.owner  = THIS_MODULE,
        .name   = EPL_DEV_NAME,
#ifdef CONFIG_OF
	    .of_match_table = alsps_of_match,
#endif
    },
};

/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
#if ALS_DYN_INTT_FAST
int dynamic_intt_fast_idx = 0;
bool als_first_repot = true;
#endif
/*lenovo-sw caoyi1 add for als report data fast 20150624 end*/

/*
//====================I2C write operation===============//
*/
static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}


/*
//====================I2C read operation===============//
*/
static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;

    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);

        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
            mdelay(10);
        }

        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}

/*
//====================write global variable===============//
*/
static void write_global_variable(struct i2c_client *client)
{
    u8 buf;
    //struct epl_sensor_priv *obj = epl_sensor_obj;

    //wake up chip
    buf = epl_sensor.reset | epl_sensor.power;
    epl_sensor_I2C_Write(client, 0x11, buf);

    /* read revno*/
    epl_sensor_I2C_Read(client, 0x20, 2);
    epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;

    /*ps setting*/
    buf = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
    epl_sensor_I2C_Write(client, 0x03, buf);

    buf = epl_sensor.ps.adc | epl_sensor.ps.cycle;
    epl_sensor_I2C_Write(client, 0x04, buf);

    buf = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive;
    epl_sensor_I2C_Write(client, 0x05, buf);

    buf = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
    epl_sensor_I2C_Write(client, 0x06, buf);

    buf = epl_sensor.ps.compare_reset | epl_sensor.ps.lock;
    epl_sensor_I2C_Write(client, 0x1b, buf);

    epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
    set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

    /*als setting*/
    buf = epl_sensor.als.integration_time | epl_sensor.als.gain;
    epl_sensor_I2C_Write(client, 0x01, buf);

    buf = epl_sensor.als.adc | epl_sensor.als.cycle;
    epl_sensor_I2C_Write(client, 0x02, buf);

    buf = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
    epl_sensor_I2C_Write(client, 0x07, buf);

    buf = epl_sensor.als.compare_reset | epl_sensor.als.lock;
    epl_sensor_I2C_Write(client, 0x12, buf);

    set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);


    /*osc setting*/
    epl_sensor_I2C_Write(client, 0xfc, epl_sensor.osc_sel| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

    //set mode and wait
    buf = epl_sensor.wait | epl_sensor.mode;
    epl_sensor_I2C_Write(client, 0x00, buf);

}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;

	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}
static bool read_factory_calibration(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    char buffer[100]= {0};
    if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
    {
		fp = filp_open(ps_cal_file, O_RDONLY, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.ps.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%d,%d,%d", &epl_sensor.ps.factory.cancelation, &epl_sensor.ps.factory.high_threshold, &epl_sensor.ps.factory.low_threshold);
            set_fs(fs);

            epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
            epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
            epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
        }

        epl_sensor_I2C_Write(obj->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(obj->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        epl_sensor.ps.factory.calibrated = true;
    }

    if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
    {
        fp = filp_open(als_cal_file, O_RDONLY, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.als.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%d", &epl_sensor.als.factory.lux_per_count);
            set_fs(fs);
        }
        epl_sensor.als.factory.calibrated = true;
    }
    return true;
}

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5, i;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];


    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }

    if(enable_ps == 0)
    {
        set_bit(CMC_BIT_PS, &epld->enable);
        epl_sensor_update_mode(epld->client);
    }

    polling_flag = false;

    for(i=0; i<count; i++)
    {
        msleep(50);
    	switch(epl_sensor.mode)
    	{
    		case EPL_MODE_PS:
    		case EPL_MODE_ALS_PS:
    		 	epl_sensor_read_ps(epld->client);
    			ch1 = epl_sensor.ps.data.data;
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    	if(epl_sensor.wait == EPL_WAIT_SINGLE)
    		epl_sensor_I2C_Write(epld->client,0x11, epl_sensor.power | epl_sensor.reset);
    }


    ch1 = (u16)(ch1_all/count);

    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 <= 0)
    {
        APS_ERR("[%s]:Failed: ch1 = 0\r\n", __func__);
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

    polling_flag = true;
    epl_sensor_restart_polling();
	return ch1;
}


static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{

    //set als / ps interrupt control mode and trigger type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 1: //ps interrupt and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 2: // ps polling and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;

		case 3: //ps and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}
/*
//====================initial global variable===============//
*/
static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
    //general setting
    epl_sensor.power = EPL_POWER_ON;
    epl_sensor.reset = EPL_RESETN_RUN;
    epl_sensor.mode = EPL_MODE_IDLE;
    epl_sensor.wait = EPL_WAIT_20_MS;
    epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

    //als setting
    epl_sensor.als.polling_mode = obj->hw->polling_mode_als;
    epl_sensor.als.integration_time = EPL_ALS_INTT_70;
    epl_sensor.als.gain = EPL_GAIN_LOW;
/*lenovo-sw caoyi1 modify for als fast 20150921 begin*/
    epl_sensor.als.adc = EPL_PSALS_ADC_11;//EPL_PSALS_ADC_13
/*lenovo-sw caoyi1 modify for als fast 20150921 end*/
    epl_sensor.als.cycle = EPL_CYCLE_16;
    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
    epl_sensor.als.persist = EPL_PERIST_1;
    epl_sensor.als.ir_on_control = EPL_IR_ON_CTRL_OFF;
    epl_sensor.als.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.als.ir_boost = EPL_IR_BOOST_100;
    epl_sensor.als.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.als.compare_reset = EPL_CMP_RESET;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor.als.report_type = CMC_BIT_DYN_INT; //CMC_BIT_RAW;
    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;

    //als factory
    epl_sensor.als.factory.calibration_enable =  false;
    epl_sensor.als.factory.calibrated = false;
    epl_sensor.als.factory.lux_per_count = LUX_PER_COUNT;

#if ALS_DYN_INTT

    epl_sensor.als.lsrc_type = CMC_BIT_LSRC_NON;

    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        dynamic_intt_idx = dynamic_intt_init_idx;
        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
    }

/*lenovo-sw caoyi1 modify Lsensor value parameter 20150617 begin*/
    c_gain = 388; // 0.388//21000; // 21000/1000=21
/*lenovo-sw caoyi1 modify Lsensor value parameter 20150617 end*/

    obj->lsource_thd_high = 1900; //different light source boundary (N) (1.9)
    obj->lsource_thd_low = 1500;   //1.5
    obj->c_gain_h = 21000; //fluorescent (C1) (21)
    obj->c_gain_l = 15000;  //incandescent (C2)   (15)

#endif
    //ps setting
    epl_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ps.integration_time = EPL_PS_INTT_70;
    epl_sensor.ps.gain = EPL_GAIN_LOW;
    epl_sensor.ps.adc = EPL_PSALS_ADC_12;
    epl_sensor.ps.cycle = EPL_CYCLE_32;
    epl_sensor.ps.persist = EPL_PERIST_1;
    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ps.ir_boost = EPL_IR_BOOST_300;
    epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;
    //ps factory
    epl_sensor.ps.factory.calibration_enable =  false;
    epl_sensor.ps.factory.calibrated = false;
    epl_sensor.ps.factory.cancelation= 0;

    //set als / ps interrupt control mode and trigger type
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    //write setting to sensor
    write_global_variable(client);
}

#if ALS_DYN_INTT
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;

/*lenovo-sw caoyi1 modify Lsensor value parameter 20150617 begin*/
    long dyn_intt_raw = 0;
    dyn_intt_raw = raw_data/(als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]);

    APS_LOG("[%s]: dyn_intt_raw=%ld \r\n", __func__, dyn_intt_raw);

    if(dyn_intt_raw > 0xffff)
        epl_sensor.als.dyn_intt_raw = 0xffff;
    else
        epl_sensor.als.dyn_intt_raw = dyn_intt_raw;


    lux = c_gain * epl_sensor.als.dyn_intt_raw;

    //lux = raw_data * (c_gain / als_dynamic_intt_value[dynamic_intt_idx]);
    //lux = raw_data * (c_gain *  als_dynamic_intt_value[1] / als_dynamic_intt_value[dynamic_intt_idx] );
#if ALS_DEBUG
    APS_LOG("[%s]:raw_data=%d, dyn_intt_raw=%d, lux=%ld\r\n", __func__, raw_data, epl_sensor.als.dyn_intt_raw, lux);
#endif
/*lenovo-sw caoyi1 modify Lsensor value parameter 20150617 end*/

    if(lux >= (dynamic_intt_max_lux*dynamic_intt_min_unit)){
#if ALS_DEBUG
        APS_LOG("[%s]:raw_convert_to_lux: change max lux\r\n", __func__);
#endif
        lux = dynamic_intt_max_lux * dynamic_intt_min_unit;
    }
    else if(lux <= (dynamic_intt_min_lux*dynamic_intt_min_unit)){
#if ALS_DEBUG
        APS_LOG("[%s]:raw_convert_to_lux: change min lux\r\n", __func__);
#endif
        lux = dynamic_intt_min_lux * dynamic_intt_min_unit;
    }

    return lux;
}
#endif

static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
#if ALS_DYN_INTT
	long now_lux=0, lux_tmp=0;
    bool change_flag = false;
#endif
    switch(epl_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als;
            break;

        case CMC_BIT_PRE_COUNT:
            return (als * epl_sensor.als.factory.lux_per_count)/1000;
            break;

        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
                {
                    break;
                }
            }

            if(idx >= obj->als_value_num)
            {
                APS_ERR("exceed range\n");
                idx = obj->als_value_num - 1;
            }

            if(!invalid)
            {
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
            break;
#if ALS_DYN_INTT
		case CMC_BIT_DYN_INT:

            if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON)
            {
                long luxratio = 0;
                epl_sensor_read_als_status(obj->client);

                if (epl_sensor.als.data.channels[0] == 0)
                {
                   epl_sensor.als.data.channels[0] = 1;
                   APS_ERR("[%s]:read ch0 data is 0 \r\n", __func__);
                }

                luxratio = (long)((als*dynamic_intt_min_unit) / epl_sensor.als.data.channels[0]); //lux ratio (A=CH1/CH0)

                obj->ratio = luxratio;
                if((epl_sensor.als.saturation >> 5) == 0)
                {
                    if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE || epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
                    {
                        if(obj->ratio == 0){
                            obj->last_ratio = luxratio;
                        }
                        else{
                            obj->last_ratio = (luxratio + obj->last_ratio*9)  / 10;
                        }

                        if (obj->last_ratio >= obj->lsource_thd_high)  //fluorescent (C1)
                        {
                            c_gain = obj->c_gain_h;
                        }
                        else if (obj->last_ratio <= obj->lsource_thd_low)   //incandescent (C2)
                        {
                            c_gain = obj->c_gain_l;
                        }
                        else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
                        {
                            int a = 0, b = 0, c = 0;
                            a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
                            b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
                            c = ((a * obj->last_ratio)/dynamic_intt_min_unit) + b;

                            if(c > obj->c_gain_h)
                                c_gain = obj->c_gain_h;
                            else if (c < obj->c_gain_l)
                                c_gain = obj->c_gain_l;
                            else
                                c_gain = c;
                        }
                    }
                    else
                    {
                        if (luxratio >= obj->lsource_thd_high)  //fluorescent (C1)
                        {
                            c_gain = obj->c_gain_h;
                        }
                        else if (luxratio <= obj->lsource_thd_low)   //incandescent (C2)
                        {
                            c_gain = obj->c_gain_l;
                        }
                        else{ /*mix*/
                            int a = 0, b = 0, c = 0;
                            a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
                            b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
                            c = ((a * luxratio)/dynamic_intt_min_unit) + b;

                            if(c > obj->c_gain_h)
                                c_gain = obj->c_gain_h;
                            else if (c < obj->c_gain_l)
                                c_gain = obj->c_gain_l;
                            else
                                c_gain = c;
                        }
                    }
#if ALS_DEBUG
                     APS_LOG("[%s]:ch0=%d, ch1=%d, c_gain=%d, obj->ratio=%d, obj->last_ratio=%d \r\n\n",
                                      __func__,epl_sensor.als.data.channels[0], als, c_gain, obj->ratio, obj->last_ratio);
#endif
                }
                else
                {
                    APS_LOG("[%s]: ALS saturation(%d) \r\n", __func__, (epl_sensor.als.saturation >> 5));
                }
            }

#if ALS_DEBUG
            APS_LOG("[%s]: dynamic_intt_idx=%d, als_dynamic_intt_value=%d, dynamic_intt_gain=%d, als=%d \r\n",
                                    __func__, dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx], als_dynamic_intt_gain[dynamic_intt_idx], als);
#endif

            if(als > dynamic_intt_high_thr)
        	{
          		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
                    als = dynamic_intt_high_thr;
          		    lux_tmp = raw_convert_to_lux(als);
#if ALS_DEBUG
        	      	APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
#endif
          		}
                else{
                    change_flag = true;
        			als  = dynamic_intt_high_thr;
              		lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx++;
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, als);
#endif
                }
            }
            else if(als < dynamic_intt_low_thr)
            {
                if(dynamic_intt_idx == 0){
                    //als = dynamic_intt_low_thr;
                    lux_tmp = raw_convert_to_lux(als);
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
#endif
                }
                else{
                    change_flag = true;
        			als  = dynamic_intt_low_thr;
                	lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx--;
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, als);
#endif
                }
            }
            else
            {
            	lux_tmp = raw_convert_to_lux(als);
            }

            now_lux = lux_tmp;
            dynamic_intt_lux = now_lux/dynamic_intt_min_unit;

/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
#if ALS_DYN_INTT_FAST
        	if(dynamic_intt_fast_idx >= (als_dynamic_intt_intt_num - 1))
        	{
        		epl_sensor.als.cycle = EPL_CYCLE_16;
        		dynamic_intt_fast_idx = (als_dynamic_intt_intt_num - 1);
        		APS_LOG("[%s]: ALS normal mode \r\n", __func__);
        	}
#endif
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/
            if(change_flag == true)
            {
                //int als_time = 0, ps_time = 0;
/*lenovo-sw caoyi modify for als resume report fast 20150806 begin*/
                //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable); // && atomic_read(&obj->ps_suspend)==0;
                //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable); // && atomic_read(&obj->als_suspend)==0;
/*lenovo-sw caoyi modify for als resume report fast 20150806 end*/

                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                epl_sensor_update_mode(obj->client);
/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
                dynamic_intt_lux = -1;  //change mode or change INTEG, -1 is ignore
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/
                change_flag = false;
            }
            return dynamic_intt_lux;

		break;
#endif
    }

    return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;


    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    epl_sensor_I2C_Write(client, 0x0c, low_lsb);
    epl_sensor_I2C_Write(client, 0x0d, low_msb);
    epl_sensor_I2C_Write(client, 0x0e, high_lsb);
    epl_sensor_I2C_Write(client, 0x0f, high_msb);

    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    epl_sensor_I2C_Write(client, 0x08, low_lsb);
    epl_sensor_I2C_Write(client, 0x09, low_msb);
    epl_sensor_I2C_Write(client, 0x0a, high_lsb);
    epl_sensor_I2C_Write(client, 0x0b, high_msb);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    APS_LOG("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    APS_LOG("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    APS_LOG("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    APS_LOG("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    APS_LOG("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    APS_LOG("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    APS_LOG("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    APS_LOG("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    APS_LOG("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    APS_LOG("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
    APS_LOG("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");

    epl_sensor_i2c_client = client;

    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_get_addr(struct alsps_hw *hw, struct epl_sensor_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;

    //APS_LOG("power %s\n", on ? "on" : "off");

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, EPL_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, EPL_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
}

/*----------------------------------------------------------------------------*/

int epl_sensor_read_als(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x13, 4);

    epl_sensor.als.data.channels[0] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl_sensor.als.data.channels[1] = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];


    APS_LOG("read als channel 0 = %d\n", epl_sensor.als.data.channels[0]);
    APS_LOG("read als channel 1 = %d\n", epl_sensor.als.data.channels[1]);

    return 0;
}

int epl_sensor_read_als_status(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x12, 1);

    buf = gRawData.raw_bytes[0];

    epl_sensor.als.saturation = (buf & 0x20);
    epl_sensor.als.compare_high = (buf & 0x10);
    epl_sensor.als.compare_low = (buf & 0x08);
    epl_sensor.als.interrupt_flag = (buf & 0x04);
    epl_sensor.als.compare_reset = (buf & 0x02);
    epl_sensor.als.lock = (buf & 0x01);

#if PS_DEBUG
	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
	APS_LOG("als: buf = 0x%x\n", buf);
	APS_LOG("als: sat = 0x%x\n", epl_sensor.als.saturation);
	APS_LOG("als: cmp h = 0x%x, l = %d\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
	APS_LOG("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_read_ps(struct i2c_client *client)
{

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client,0x1c, 4);

    epl_sensor.ps.data.ir_data = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl_sensor.ps.data.data = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];

	APS_LOG("[%s]: data = %d\n", __func__, epl_sensor.ps.data.data);
	APS_LOG("[%s]: ir data = %d\n", __func__, epl_sensor.ps.data.ir_data);

    return 0;
}

int epl_sensor_read_ps_status(struct i2c_client *client)
{
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client, 0x1b, 1);

    buf = gRawData.raw_bytes[0];

    epl_sensor.ps.saturation = (buf & 0x20);
    epl_sensor.ps.compare_high = (buf & 0x10);
    epl_sensor.ps.compare_low = (buf & 0x08);
    epl_sensor.ps.interrupt_flag = (buf & 0x04);
    epl_sensor.ps.compare_reset = (buf & 0x02);
    epl_sensor.ps.lock= (buf & 0x01);

#if PS_DEBUG
	APS_LOG("ps: ~~~~ PS ~~~~~ \n");
	APS_LOG("ps: buf = 0x%x\n", buf);
	APS_LOG("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
	APS_LOG("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
#endif
    return 0;
}

void epl_sensor_restart_polling(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    cancel_delayed_work(&obj->polling_work);
    schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(50));
}

void epl_sensor_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    cancel_delayed_work(&obj->polling_work);

}

static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>1];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);
#endif

    sensing_us_time = (als_intt + als_adc*2*3) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;
#if 0
    if(epl_sensor.osc_sel == EPL_OSC_SEL_4MHZ)
    {
        sensing_ms_time = sensing_ms_time/4;
    }
#endif
#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
#endif
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_adc, ps_cycle;

    ps_intt = ps_intt_value[intt>>1];
    ps_adc = adc_value[adc>>3];
    ps_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);
#endif

    sensing_us_time = (ps_intt*3 + ps_adc*2*2) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;

    if(epl_sensor.osc_sel == EPL_OSC_SEL_4MHZ)
    {
        sensing_ms_time = sensing_ms_time/4;
    }
#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
#endif
    return (sensing_ms_time + 5);
}

static int epl_sensor_get_wait_time(int ps_time, int als_time)
{
    int wait_idx = 0;
    int wait_time = 0;

    wait_time = als_time - ps_time;
    if(wait_time < 0){
        wait_time = 0;
    }
#if COMMON_DEBUG
    APS_LOG("[%s]: wait_len = %d \r\n", __func__, wait_len);
#endif
    for(wait_idx = 0; wait_idx < wait_len; wait_idx++)
	{
	    if(wait_time < wait_value[wait_idx])
	    {
	        break;
	    }
	}

	if(wait_idx >= wait_len){
        wait_idx = wait_len - 1;
	}

#if COMMON_DEBUG
	APS_LOG("[%s]: wait_idx = %d, wait = %d \r\n", __func__, wait_idx, wait_value[wait_idx]);
#endif
	return (wait_idx << 4);
}

void epl_sensor_fast_update(struct i2c_client *client)
{
    int als_fast_time = 0;

    APS_FUN();
    mutex_lock(&sensor_mutex);
    als_fast_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, EPL_CYCLE_1);

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);

    epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | EPL_CYCLE_1);
    epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);

    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_ALS);
    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);
    msleep(als_fast_time);
    APS_LOG("[%s]: msleep(%d)\r\n", __func__, als_fast_time);

    mutex_lock(&sensor_mutex);
    if(epl_sensor.als.polling_mode == 0)
    {
        //fast_mode is already ran one frame, so must to reset CMP bit for als intr mode
        //IDLE mode and CMMP reset
        epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);
        epl_sensor_I2C_Write(client, 0x12, EPL_CMP_RESET | EPL_UN_LOCK);
        epl_sensor_I2C_Write(client, 0x12, EPL_CMP_RUN | EPL_UN_LOCK);
    }

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_OFF | EPL_RESETN_RESET);

    epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);

    epl_sensor_I2C_Write(client, 0x11, EPL_POWER_ON | EPL_RESETN_RUN);
    mutex_unlock(&sensor_mutex);
}

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int als_time = 0, ps_time = 0;
/*lenovo-sw caoyi modify for als resume report fast 20150806 begin*/
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable); // && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable); // && atomic_read(&obj->als_suspend)==0;
/*lenovo-sw caoyi1 add for close ps fast 20150910 begin*/
    u8 last_mode = epl_sensor.mode;
/*lenovo-sw caoyi1 add for close ps fast 20150910 end*/
/*lenovo-sw caoyi modify for als resume report fast 20150806 end*/

    als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    als_frame_time = als_time;
    ps_frame_time = ps_time;

    polling_flag = false;

    if(enable_als == true)
    {
        epl_sensor_fast_update(obj->client);
		APS_LOG("[%s]: ALS fast mode \r\n", __func__);
    }

	APS_LOG("mode selection =0x%x\n", enable_ps | (enable_als << 1));
    //PS unlock and reset
    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

    //ALS unlock and reset
    epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

    epl_sensor.ps.interrupt_flag = EPL_INT_CLEAR;
    epl_sensor.als.interrupt_flag = EPL_INT_CLEAR;

    //**** mode selection ****
    switch((enable_als << 1) | enable_ps)
    {
        case 0: //disable all
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1: //als = 0, ps = 1
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 2: //als = 1, ps = 0
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 3: //als = 1, ps = 1
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

#if PS_DYN_K_ONE
    if(ps_dyn_flag == true)
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_mask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif
        APS_LOG("[%s]: ps_dyn_flag=%d, eint mask! \r\n", __func__, ps_dyn_flag);
    }
#endif


    //**** write setting ****
    // step 1. set sensor at idle mode
    // step 2. uplock and reset als / ps status
    // step 3. set sensor at operation mode
    // step 4. delay sensing time
    // step 5. unlock and run als / ps status

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

    // initial factory calibration variable
    read_factory_calibration();

    epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

    epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);

    epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);


#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT){
	    epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
#if 0
	    if(epl_sensor.osc_sel == EPL_OSC_SEL_4MHZ)
        {
            APS_LOG("[%s]: change OSC to 1M \r\n", __func__);
            epl_sensor.osc_sel == EPL_OSC_SEL_1MHZ;
            epl_sensor_I2C_Write(client, 0xfc, epl_sensor.osc_sel| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
        }
#endif
    }
#endif


    if(epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 0 && epl_sensor.ps.polling_mode == 0){
        int wait = 0;
        wait = epl_sensor_get_wait_time(ps_time, als_time);
        epl_sensor_I2C_Write(client, 0x00, wait | epl_sensor.mode);
        epl_sensor.wait = wait;
        APS_LOG("[%s]: epl_sensor.als.polling_mode=%d \r\n", __func__, epl_sensor.als.polling_mode);
    }
    else{
        epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | epl_sensor.mode);
    }

    epl_sensor_I2C_Write(client, 0x11, epl_sensor.reset | epl_sensor.power);

    //PS unlock and run
    epl_sensor.ps.compare_reset = EPL_CMP_RUN;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

    //ALS unlock and run
   	epl_sensor.als.compare_reset = EPL_CMP_RUN;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);


#if COMMON_DEBUG
    //**** check setting ****
    if(enable_ps == 1)
    {
        APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    }

    if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
    {
        APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    }
    APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  epl_sensor.wait | epl_sensor.mode);
	APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, epl_sensor.power | epl_sensor.reset);
	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
#endif

    if (epl_sensor.mode == EPL_MODE_IDLE)
    {
        APS_LOG("[%s] not sleep \r\n", __func__);
    }
/*lenovo-sw caoyi1 add for close ps fast 20150910 begin*/
    else if(last_mode == EPL_MODE_ALS_PS && epl_sensor.mode == EPL_MODE_ALS)
    {
        APS_LOG("[%s] EPL_MODE_ALS_PS change to EPL_MODE_ALS, not sleep \r\n", __func__);
    }
/*lenovo-sw caoyi1 add for close ps fast 20150910 end*/
    else if(epl_sensor.mode == EPL_MODE_PS)
    {
        msleep(ps_time);
        APS_LOG("[%s] PS only(%dms)\r\n", __func__, ps_time);
    }
    else if (epl_sensor.mode == EPL_MODE_ALS )
    {
        msleep(als_time);
        APS_LOG("[%s] ALS only(%dms)\r\n", __func__, als_time);
    }
    else if (epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 1)
    {
/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
        msleep(ps_time+als_time);
        APS_LOG("[%s] PS+ALS(%dms)\r\n", __func__, ps_time+als_time);
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/
    }
/*lenovo-sw caoyi1 add for MMI sensor test 20151020 begin*/
#if PS_AUTO_TEST
    epl_sensor_read_ps(obj->client);
    if((epl_sensor.ps.data.data<=epl_sensor.ps.high_threshold) && (epl_sensor.ps.data.data>=epl_sensor.ps.low_threshold))
    {
        APS_LOG("[%s]: direct report FAR(%d) \r\n", __func__, epl_sensor.ps.data.data);
        epl_sensor.ps.compare_low = 0x08;
        epl_sensor_report_ps_status();
    }
#endif
/*lenovo-sw caoyi1 add for MMI sensor test 20151020 end*/
#if PS_DYN_K_ONE
    u8 buf;
    if(ps_dyn_flag == true)
    {
        epl_sensor_I2C_Read(client, 0x1b, 1);
        buf = gRawData.raw_bytes[0];
        epl_sensor.ps.saturation = (buf & 0x20);
        APS_LOG("[%s]: 0x1b=0x%x \r\n", __func__, buf);

	    epl_sensor_read_ps(obj->client);
	    if(epl_sensor.ps.data.data < PS_MAX_CT && (epl_sensor.ps.saturation == 0))
    	{
    	    APS_LOG("[%s]: epl_sensor.ps.data.data=%d \r\n", __func__, epl_sensor.ps.data.data);
    	    dynk_thd_low = epl_sensor.ps.data.data + PS_DYN_L_OFFSET;
    		dynk_thd_high = epl_sensor.ps.data.data + PS_DYN_H_OFFSET;
    		set_psensor_intr_threshold(dynk_thd_low, dynk_thd_high);
       }
       else
       {
            if((epl_sensor.ps.high_threshold ==0)||(epl_sensor.ps.low_threshold==0))
            {
		        epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
                epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;
                APS_LOG("[%s]:threshold is err; epl_sensor.ps.data.data=%d \r\n", __func__, epl_sensor.ps.data.data);
            }
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            dynk_thd_low = epl_sensor.ps.low_threshold;
            dynk_thd_high = epl_sensor.ps.high_threshold;
       }
#if !EINT_API
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
       mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
       mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif
    }
#endif

#if PS_DYN_K_ONE
    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER && ps_dyn_flag == false)
#else
    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
#endif
    {
        //PS unlock and run
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
#if !EINT_API
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif
    }

#if PS_DYN_K_ONE
    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER && ps_dyn_flag == false)
#else
    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
#endif
    {
        //ALS unlock and run
       	epl_sensor.als.compare_reset = EPL_CMP_RUN;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
#if !EINT_API
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif
    }

#if PS_DYN_K_ONE
    if(ps_dyn_flag == true)
    {
        ps_dyn_flag = false;
        APS_LOG("[%s]: reset ps_dyn_flag ........... \r\n", __func__);
    }
#endif
    polling_flag = true;

/*lenovo-sw caoyi1 add for Psensor eint 20150603 begin*/
    if(enable_ps == 1)
    {
        APS_LOG("[%s]: unmask INTR \r\n", __func__);
#if !EINT_API
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif
    }
/*lenovo-sw caoyi1 add for Psensor eint 20150603 end*/

    //mutex_unlock(&sensor_mutex);

}


/*----------------------------------------------------------------------------*/
#if EINT_API
static irqreturn_t epl_sensor_eint_func(int irq, void *desc)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    // APS_LOG(" interrupt fuc\n");

    int_top_time = sched_clock();

    if(!obj)
    {
        return IRQ_HANDLED;
    }

    disable_irq_nosync(obj->irq);

    schedule_delayed_work(&obj->eint_work, 0);

    return IRQ_HANDLED;
}

#else
void epl_sensor_eint_func(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    // APS_LOG(" interrupt fuc\n");

    if(!obj)
    {
        return;
    }
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
    mt_eint_mask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif
    schedule_delayed_work(&obj->eint_work, 0);
}
#endif/*EINT_API*/


static void epl_sensor_report_ps_status(void)
{
#if MTK_LTE
    int ps_state;
#else
    hwm_sensor_data sensor_data;
#endif
    int err;
#if PS_DEBUG
    APS_FUN();
#endif

#if MTK_LTE
    ps_state = epl_sensor.ps.compare_low >> 3;
    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, ps_status=%d \r\n", __func__, epl_sensor.ps.data.data, ps_state);
    err = ps_report_interrupt_data(ps_state);
    if(err != 0)
    {
        APS_ERR("epl_sensor_eint_work err: %d\n", err);
    }

#else

    sensor_data.values[0] = epl_sensor.ps.compare_low >> 3;
#if PS_DEBUG
	sensor_data.values[1] = epl_sensor.ps.data.data;
#else
    sensor_data.values[1] = 0;
#endif
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

	if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}
#endif
}

static void epl_sensor_intr_als_report_lux(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
#if MTK_LTE
    u16 lux;
#else
    hwm_sensor_data sensor_data;
  	int err;
#endif

    APS_LOG("[%s]: IDEL MODE \r\n", __func__);
    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

    epl_sensor_read_als(obj->client);
#if MTK_LTE
    lux = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
#if !defined(MT6753) //MT6753 dont support
    if((err = als_report_interrupt_data(lux)))
    {
        APS_ERR("epl_sensor call als_report_interrupt_data fail = %d\n", err);
    }
#endif
    APS_LOG("[%s]: ALS Lux=%d \r\n", __func__, lux);
#else
    sensor_data.values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
	sensor_data.values[1] = epl_sensor.als.data.channels[1];
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif
	epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

	//set dynamic threshold
	if(epl_sensor.als.compare_high >> 4)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;

		if (epl_sensor.als.high_threshold > 60000)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
		}
	}
	if(epl_sensor.als.compare_low>> 3)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;

		if (epl_sensor.als.high_threshold < 250)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
		}
	}

    if(epl_sensor.als.high_threshold < epl_sensor.als.low_threshold)
	{
	    APS_LOG("[%s]:recover default setting \r\n", __FUNCTION__);
	    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
	    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
	}
#if !MTK_LTE
	if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}
#endif
	//write new threshold
	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);
    APS_LOG("[%s]: MODE=0x%x \r\n", __func__, epl_sensor.mode);
}

/*----------------------------------------------------------------------------*/
static void epl_sensor_eint_work(struct work_struct *work)
{
    //hwm_sensor_data sensor_data;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    //bool isWork;
    //int err;

    eint_flag = false;
    mutex_lock(&sensor_mutex);

#if PS_DEBUG
    APS_LOG("xxxxxxxxxxx\n\n");
#endif
    if(enable_ps && epl_sensor.ps.polling_mode == 0)
    {
        epl_sensor_read_ps_status(obj->client);

        if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_ps)
        {
#if PS_DEBUG
	        epl_sensor_read_ps(obj->client);
#endif
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
            wake_lock_timeout(&ps_lock, 1*HZ);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
            epl_sensor_report_ps_status();
            //if(polling_flag == true)
            {
                //PS unlock and run
        		epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        		epl_sensor.ps.lock = EPL_UN_LOCK;
        		epl_sensor_I2C_Write(obj->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
    		}
        }
    }

    if(enable_als && obj->hw->polling_mode_als == 0)
    {
        epl_sensor_read_als_status(obj->client);

        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_als)
        {
            epl_sensor_intr_als_report_lux();
            if(polling_flag == true)
            {
                //ALS unlock and run
        		epl_sensor.als.compare_reset = EPL_CMP_RUN;
        		epl_sensor.als.lock = EPL_UN_LOCK;
        		epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
    		}
        }
    }

    mutex_unlock(&sensor_mutex);
    eint_flag = true;

    //if(polling_flag == true)
    {
#if EINT_API
        enable_irq(obj->irq);
#else
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif/*EINT_API*/
    }
}



/*----------------------------------------------------------------------------*/
int epl_sensor_setup_eint(struct i2c_client *client)
{
#if EINT_API
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
	int ret;
	u32 ints[2] = { 0, 0 };
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	struct platform_device *alsps_pdev = get_alsps_platformdev();

	APS_LOG("epl_sensor_setup_eint\n");

    g_epl8801_ptr = obj;
	/*configure to GPIO function, external interrupt */
#ifndef FPGA_EARLY_PORTING
    /* gpio setting */
	pinctrl = devm_pinctrl_get(&alsps_pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}

	if (epl_sensor_obj->irq_node) {
		of_property_read_u32_array(epl_sensor_obj->irq_node, "debounce", ints,
					   ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		pinctrl_select_state(pinctrl, pins_cfg);

		epl_sensor_obj->irq = irq_of_parse_and_map(epl_sensor_obj->irq_node, 0);
		APS_LOG("epl8801_obj->irq = %d\n", epl_sensor_obj->irq);
		if (!epl_sensor_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(epl_sensor_obj->irq, epl_sensor_eint_func, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq(epl_sensor_obj->irq);
	}
	else
	{
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	enable_irq(epl_sensor_obj->irq);
#endif				/* #ifndef FPGA_EARLY_PORTING */


#else
    APS_LOG("epl_sensor_setup_eint\n");
    /*configure to GPIO function, external interrupt*/

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#if defined(MT6575) || defined(MT6571) || defined(MT6589)
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_sensor_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_sensor_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif /*EINT_API*/


    return 0;
}




/*----------------------------------------------------------------------------*/
static int epl_sensor_init_client(struct i2c_client *client)
{
    //struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0;

    APS_LOG("I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr, __LINE__);

    /*  interrupt mode */


    APS_FUN();
#if 0
    if(obj->hw->polling_mode_ps == 0)
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_mask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif

        if((err = epl_sensor_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl_sensor interrupt setup\n");
    }
#endif

    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }


    /*  interrupt mode */
    //if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFB value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFB));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    //u16 ch0, ch1, ch2, ch3;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d, osc=%d \n",epl_sensor.wait >> 4, epl_sensor.mode, epl_sensor.osc_sel);
    len += snprintf(buf+len, PAGE_SIZE-len, "interrupt control = %d\n", epl_sensor.interrupt_control >> 4);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);
    if(enable_ps)
    {
    len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
    len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 1, epl_sensor.ps.gain);
    len += snprintf(buf+len, PAGE_SIZE-len, "adc = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.adc >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
    len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
    len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
#if PS_DYN_K_ONE
    len += snprintf(buf+len, PAGE_SIZE-len, "Dyn thr(L/H) = (%ld/%ld)\n", (long)dynk_thd_low, (long)dynk_thd_high);
#endif
    len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
    len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
    len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 1, epl_sensor.als.gain);
    len += snprintf(buf+len, PAGE_SIZE-len, "adc = %d, cycle = %d\n", epl_sensor.als.adc >> 3, epl_sensor.als.cycle);
#if ALS_DYN_INTT
    if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "lsource_thd_low=%d, lsource_thd_high=%d \n", epld->lsource_thd_low, epld->lsource_thd_high);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d\n", epl_sensor.als.saturation >> 5);

        if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE || epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "real_ratio = %d\n", epld->ratio);
            len += snprintf(buf+len, PAGE_SIZE-len, "use_ratio = %d\n", epld->last_ratio);
        }
        else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SLOPE)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "ratio = %d\n", epld->ratio);
        }
    }
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "c_gain = %d\n", c_gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "dynamic_intt_lux = %d\n", dynamic_intt_lux);
    }
#endif
    if(epl_sensor.als.polling_mode == 0)
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);

    if(enable_als != mode)
    {
        if(mode)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
#endif
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }

        epl_sensor_update_mode(obj->client);
    }

    return count;
}

/*lenovo-sw caoyi1 add als enable attr 20150901 begin*/
static ssize_t epl_sensor_show_als_enable(struct device_driver *ddri, char *buf)
{
      ssize_t len = 0;
      struct epl_sensor_priv *obj = epl_sensor_obj;
      bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
      APS_FUN();

      APS_LOG("[%s]: enable_als=%d \r\n", __func__, enable_als);

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", enable_als);
      return len;
}
/*lenovo-sw caoyi1 add als enable attr 20150901 end*/

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_FUN();

    sscanf(buf, "%hu", &mode);
    if(enable_ps != mode)
    {
        if(mode)
        {
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
            //wake_lock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
            set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K_ONE
            ps_dyn_flag = true;
#endif
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
            //wake_unlock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
        }
        epl_sensor_update_mode(obj->client);
    }
    return count;
}

/*lenovo-sw caoyi1 add ps enable attr 20150901 begin*/
static ssize_t epl_sensor_show_ps_enable(struct device_driver *ddri, char *buf)
{
      ssize_t len = 0;
      struct epl_sensor_priv *obj = epl_sensor_obj;
      bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
      APS_FUN();

      APS_LOG("[%s]: enable_als=%d \r\n", __func__, enable_ps);

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", enable_ps);
      return len;
}
/*lenovo-sw caoyi1 add ps enable attr 20150901 end*/

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_cal_raw(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    for(i=0; i<count; i++)
    {
        switch(epl_sensor.mode)
        {
            case EPL_MODE_PS:
                if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
                    epl_sensor_read_ps(obj->client);
                ch1 = epl_sensor.ps.data.data;
            break;

            case EPL_MODE_ALS:
                epl_sensor_read_als(obj->client);
                ch1 = epl_sensor.als.data.channels[1];
            break;
        }

        msleep(50);
        ch1_all = ch1_all + ch1;
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client, 0x11,  epl_sensor.power | epl_sensor.reset);
    }

    ch1 = (u16)ch1_all/count;

    APS_LOG("cal_raw = %d \r\n" , ch1);

    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);

    return  len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int low, high;
    APS_FUN();
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d,%d", &low, &high);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            epl_sensor.ps.low_threshold = low;
            epl_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        break;

        case EPL_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            epl_sensor.als.low_threshold = low;
            epl_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        break;

    }

    return  count;
}

static ssize_t epl_sensor_show_threshold(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    ssize_t len = 0;

    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
        break;

        case EPL_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
        break;

    }
    return  len;

}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_wait_time(struct device_driver *ddri, const char *buf, size_t count)
{
    //struct epl_sensor_priv *obj = epl_sensor_obj;
    int val;

    sscanf(buf, "%d",&val);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
        case EPL_MODE_ALS:
        case EPL_MODE_ALS_PS:
            epl_sensor.wait = (val & 0xf) << 4;
            break;
    }


    return count;
}
#if 0
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
    APS_FUN();

    sscanf(buf, "%d", &value);

    value = value & 0x03;

	switch (epl_sensor.mode)
	{
        case EPL_MODE_PS:
            epl_sensor.ps.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;
    }

	epl_sensor_update_mode(epld->client);

	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int value=0;
    APS_FUN();

    clear_bit(CMC_BIT_PS, &obj->enable);
    clear_bit(CMC_BIT_ALS, &obj->enable);

    sscanf(buf, "%d",&value);

    switch (value)
    {
        case 0:
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1:
            set_bit(CMC_BIT_ALS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 2:
            set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 3:
            set_bit(CMC_BIT_ALS, &obj->enable);
            set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    epl_sensor_update_mode(obj->client);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
                break;

                case 1:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_VOLTAGE;
                break;
            }

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive);
            break;

    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_contrl(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    uint8_t  data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_OFF;
                 break;

                case 1:
                    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
                break;
            }

            data = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive;
            APS_LOG("%s: 0x05 = 0x%x\n", __FUNCTION__, data);

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_boost(struct device_driver *ddri, const char *buf, size_t count)
{
    int data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &data);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.ir_boost = (data & 0x03) << 2;
            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_drive(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch(epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.ir_drive = (value & 0x03);
            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_boost | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_interrupt_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            if(!obj->hw->polling_mode_ps)
            {
                epl_sensor.ps.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
                APS_LOG("%s: 0x06 = 0x%x\n", __FUNCTION__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
            }
            break;

        case EPL_MODE_ALS: //als
            if(!obj->hw->polling_mode_als)
            {
                epl_sensor.als.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
                APS_LOG("%s: 0x07 = 0x%x\n", __FUNCTION__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
            }
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_offset(struct device_driver *ddri, const char *buf, size_t count)
{
    int data = 0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d",&data);

    epl_sensor_I2C_Write(obj->client, 0x22, (u8)(data & 0xff));
    epl_sensor_I2C_Write(obj->client, 0x23, (u8)((data & 0xff00) >> 8));

    epl_sensor_I2C_Read(obj->client, 0x23, 2);
    APS_LOG("%s: 0x22, 0x23 = 0x%x, 0x%x\n", __FUNCTION__, gRawData.raw_bytes[0], gRawData.raw_bytes[1]);
    epl_sensor_update_mode(obj->client);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_osc(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    uint8_t  data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch(value)
    {
        case 0:
            epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;
            break;

        case 1:
            epl_sensor.osc_sel = EPL_OSC_SEL_4MHZ;
            break;

    }

    data = epl_sensor.osc_sel | EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON;
    APS_LOG("%s: 0xfc = 0x%x \n", __FUNCTION__, data);
    epl_sensor_I2C_Write(obj->client, 0xfc, data);
    epl_sensor_update_mode(obj->client);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.integration_time = (value & 0x1f) << 1;
            epl_sensor_I2C_Write(obj->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            epl_sensor_I2C_Read(obj->client, 0x03, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.integration_time = (value & 0x1f) << 1;
            epl_sensor_I2C_Write(obj->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
            epl_sensor_I2C_Read(obj->client, 0x01, 1);
            APS_LOG("%s: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_adc(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

static ssize_t epl_sensor_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
        case EPL_MODE_PS:
            epl_sensor.ps.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    APS_FUN();

    sscanf(buf, "%d", &value);
    epl_sensor.als.report_type = value & 0xf;

    return count;
}


static ssize_t epl_sensor_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_ps;
#endif
    sscanf(buf, "%d",&epld->hw->polling_mode_ps);
#if !MTK_LTE
    hwmsen_detach(ID_PROXIMITY);
#endif
    APS_LOG("epld->hw->polling_mode_ps=%d \r\n", epld->hw->polling_mode_ps);
#if !MTK_LTE
    epl_sensor.ps.polling_mode = epld->hw->polling_mode_ps;
    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_ps;
    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if(hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#endif
    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_als_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_ps;
#endif

    sscanf(buf, "%d",&epld->hw->polling_mode_als);
#if !MTK_LTE
    hwmsen_detach(ID_LIGHT);
#endif
    APS_LOG("epld->hw->polling_mode_als=%d \r\n", epld->hw->polling_mode_als);
    epl_sensor.als.polling_mode = epld->hw->polling_mode_als;

#if !MTK_LTE
    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_als;
    obj_ps.sensor_operate = epl_sensor_als_operate;

    if(hwmsen_attach(ID_LIGHT, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#endif
    epl_sensor_update_mode(epld->client);

    return count;
}
#if 0
static ssize_t epl_sensor_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	APS_FUN();

	if(!epl_sensor_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);
	return count;
}
#endif
/*----------------------------------------------------------------------------*/

static ssize_t epl_sensor_store_unlock(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int mode;
    APS_FUN();

    sscanf(buf, "%d",&mode);

    APS_LOG("mode = %d \r\n", mode);
	switch (mode)
	{
		case 0: //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 1: //PS unlock and reset
        	epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 2: //ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;

        case 3: //ALS unlock and reset
    		epl_sensor.als.compare_reset = EPL_CMP_RESET;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        break;

		case 4: //ps+als
		    //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			//ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;
	}
    /*double check PS or ALS lock*/


	return count;
}

static ssize_t epl_sensor_store_als_ch_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ch_sel;
    APS_FUN();

    sscanf(buf, "%d",&ch_sel);

    APS_LOG("channel selection = %d \r\n", ch_sel);
	switch (ch_sel)
	{
		case 0: //ch0
		    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_0;
		break;

		case 1: //ch1
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
		break;

		case 2: //ch2
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_2;
		break;

		case 3: //ch3
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_3;
		break;
	}
    epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&epl_sensor.als.factory.lux_per_count);
    return count;
}
#if 0
static ssize_t epl_sensor_store_ps_cal_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    APS_FUN();

    sscanf(buf, "%d",&epl_sensor.ps.factory.calibration_enable);

    return count;
}
#endif

static ssize_t epl_sensor_store_i2c_max_count(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&i2c_max_count);
    return count;
}

static ssize_t epl_sensor_show_ps_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.ps.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_als_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.als.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_ps_run_cali(struct device_driver *ddri, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();

    ret = epl_run_ps_calibration(epld);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}

static ssize_t epl_sensor_show_pdata(struct device_driver *ddri, char *buf)
{
      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
      bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
      APS_FUN();

      if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
      {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps(epld->client);
        mutex_unlock(&sensor_mutex);
      }

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.ps.data.data);
      return len;

}

static ssize_t epl_sensor_show_als_data(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    ssize_t len = 0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
    APS_FUN();

#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        APS_LOG("[%s]: dynamic_intt_lux = %d \r\n", __func__, dynamic_intt_lux);
        len += snprintf(buf + len, PAGE_SIZE - len, "%d", dynamic_intt_lux);
    }
    else
#endif
    {
        if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
    	    epl_sensor_read_als(epld->client);
    	    mutex_unlock(&sensor_mutex);
    	}
    	len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.als.data.channels[1]);
    }
    return len;
}

#if ALS_DYN_INTT
static ssize_t epl_sensor_store_c_gain(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int c_h,c_l;
    APS_FUN();

    if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_NON)
    {
        sscanf(buf, "%d",&c_h);
        c_gain = c_h;
        APS_LOG("c_gain = %d \r\n", c_gain);
    }
    else
    {
        sscanf(buf, "%d,%d",&c_l, &c_h);
        epld->c_gain_h = c_h;
        epld->c_gain_l = c_l;
    }

	return count;
}

static ssize_t epl_sensor_store_lsrc_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int type;
    //struct epl_sensor_priv *epld = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d",&type);

    epl_sensor.als.lsrc_type = type;

    APS_LOG("epl_sensor.als.lsrc_type = %d \r\n", epl_sensor.als.lsrc_type);

	return count;
}

static ssize_t epl_sensor_store_lsrc_thd(struct device_driver *ddri, const char *buf, size_t count)
{
    int lsrc_thrl, lsrc_thrh;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d,%d",&lsrc_thrl, &lsrc_thrh);

    epld->lsource_thd_low = lsrc_thrl;
    epld->lsource_thd_high = lsrc_thrh;

    APS_LOG("lsource_thd=(%d,%d) \r\n", epld->lsource_thd_low, epld->lsource_thd_high);

	return count;
}

#endif

static ssize_t epl_sensor_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;
    APS_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);
    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}

#if !EINT_API
static ssize_t epl_sensor_store_intr_type(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
	int value;
	u8 intt_buf;

    sscanf(buf, "%d",&value);

    if(value == 0) //low, edge
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        //mt_eint_set_sens(CUST_EINT_ALS_NUM, 0);
        mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_sensor_eint_func, 0);
#else
        mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
        mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_sensor_eint_func, 0);
#endif

    }
    else if (value == 1) //low, level
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        //mt_eint_set_sens(CUST_EINT_ALS_NUM, 1);
        mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_sensor_eint_func, 0);
#else
        mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_LEVEL_SENSITIVE);
        mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_sensor_eint_func, 0);
#endif
    }
    else if (value == 2) //high, edge
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        APS_LOG("[%s]:Dont support \r\n", __func__);
#else
        mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
        mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, epl_sensor_eint_func, 0);
#endif
    }
    else if (value == 3) //high, level
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        APS_LOG("[%s]:Dont support \r\n", __func__);
#else
        mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_LEVEL_SENSITIVE);
        mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, epl_sensor_eint_func, 0);
#endif
    }
#if !EINT_API
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
#endif

    return count;
}

static ssize_t epl_sensor_show_sen(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int polarity;
    //polarity = mt_eint_get_sens(CUST_EINT_ALS_NUM);
    APS_FUN();
    APS_LOG("polarity=%d \r\n", polarity);

    len += snprintf(buf+len, PAGE_SIZE-len, "%d, %s \n", polarity, (polarity==1 ? "LOW":"EDGE"));

    return len;
}
#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150511 begin*/
#if LENOVO_CALI
static ssize_t epl_sensor_store_ps_h_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int thr_h;
    APS_FUN();

    sscanf(buf, "%d",&thr_h);

    APS_LOG("[%s]: thr_h=%d", __func__, thr_h);

    epld->hw->ps_threshold_high = thr_h;
    epl_sensor.ps.high_threshold = thr_h;

    set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    return count;
}

static ssize_t epl_sensor_show_ps_h_threshold(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("[%s]: epl_sensor.ps.high_threshold=%d \r\n", __func__, epl_sensor.ps.high_threshold);

    len += snprintf(buf+len, PAGE_SIZE-len, "%d", epl_sensor.ps.high_threshold);

    return len;
}

static ssize_t epl_sensor_store_ps_l_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int thr_l;
    APS_FUN();

    sscanf(buf, "%d",&thr_l);

    APS_LOG("[%s]: thr_l=%d", __func__, thr_l);

    epld->hw->ps_threshold_low = thr_l;
    epl_sensor.ps.low_threshold = thr_l;

    set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    return count;
}

static ssize_t epl_sensor_show_ps_l_threshold(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("[%s]: epl_sensor.ps.low_threshold=%d \r\n", __func__, epl_sensor.ps.low_threshold);

    len += snprintf(buf+len, PAGE_SIZE-len, "%d", epl_sensor.ps.low_threshold);

    return len;
}

#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150511 end*/

/*lenovo-sw caoyi1 add for selftest function 20150715 begin*/
static ssize_t epl_sensor_show_selftest(struct device_driver *ddri, char *buf)
{
      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
      bool result;
      APS_FUN();

      if((i2c_smbus_read_byte_data(epld->client, 0x21)) != EPL_REVNO)
      {
            result = false;
      }
      else
      {
            result = true;
      }
      APS_LOG("[%s]: result=%d \r\n", __func__, result);

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", result);
      return len;

}
/*lenovo-sw caoyi1 add for selftest function 20150715 end*/

/*lenovo-sw caoyi1 add for ps status attr 20150901 begin*/
static ssize_t epl_sensor_show_ps_status(struct device_driver *ddri, char *buf)
{
      ssize_t len = 0;
      int ps_status;
      APS_FUN();

      ps_status = epl_sensor.ps.compare_low >> 3;
      APS_LOG("[%s]: ps_status=%d \r\n", __func__, ps_status);

      len += snprintf(buf + len, PAGE_SIZE - len, "%d", ps_status);
      return len;

}
/*lenovo-sw caoyi1 add for ps status attr 20150901 end*/

/*lenovo-sw caoyi1 add ps_status for TP 20151013 begin*/
#if 0
int epl_sensor_show_ps_status_for_tp()
{
	int ps_status=-1;
	struct epl_sensor_priv *obj = epl_sensor_obj;
	bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	if ( obj == NULL ) {
		APS_ERR("null pointer!!\n");
		return -1;
        }

	APS_LOG("[%s]: enable_ps=%d \r\n", __func__, enable_ps);

	if(enable_ps != 1)
	{
		return -1;
	}
	else
	{
		ps_status = epl_sensor.ps.compare_low >> 3;
		APS_LOG("[%s]: ps_status=%d \r\n", __func__, ps_status);

		return ps_status;
	}
}
#endif
/*lenovo-sw caoyi1 add ps_status for TP 20151013 end*/

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IWUSR | S_IRUGO, epl_sensor_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    				S_IWUSR | S_IRUGO, epl_sensor_show_reg,   				NULL										);
/*lenovo-sw caoyi1 add als enable attr 20150901 begin*/
static DRIVER_ATTR(als_enable,					S_IWUSR | S_IRUGO, epl_sensor_show_als_enable,   		epl_sensor_store_als_enable					);
/*lenovo-sw caoyi1 add als enable attr 20150901 end*/
static DRIVER_ATTR(als_report_type,				S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_als_report_type				);
static DRIVER_ATTR(als_polling_mode,			S_IWUSR | S_IRUGO, epl_sensor_show_als_polling,   		epl_sensor_store_als_polling_mode				);
static DRIVER_ATTR(als_lux_per_count,			S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_als_lux_per_count				);
/*lenovo-sw caoyi1 add ps enable attr 20150901 begin*/
static DRIVER_ATTR(ps_enable,					S_IWUSR | S_IRUGO, epl_sensor_show_ps_enable,   		epl_sensor_store_ps_enable					);
/*lenovo-sw caoyi1 add ps enable attr 20150901 end*/
static DRIVER_ATTR(ps_offset,					S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_ps_offset						);
static DRIVER_ATTR(ps_polling_mode,			    S_IWUSR | S_IRUGO, epl_sensor_show_ps_polling,   					 		epl_sensor_store_ps_polling_mode				);
static DRIVER_ATTR(ir_mode,					    S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ir_mode						);
static DRIVER_ATTR(ir_boost,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ir_boost						);
static DRIVER_ATTR(ir_drive,					S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_ir_drive						);
static DRIVER_ATTR(ir_on,						S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_ir_contrl						);
static DRIVER_ATTR(interrupt_type,				S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_interrupt_type				);
static DRIVER_ATTR(osc,						    S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_osc							);
static DRIVER_ATTR(integration,					S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_integration					);
static DRIVER_ATTR(adc,						    S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_adc							);
static DRIVER_ATTR(cycle,						S_IWUSR | S_IRUGO, NULL,								epl_sensor_store_cycle						);
static DRIVER_ATTR(i2c_max_count,			    S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_i2c_max_count				);
static DRIVER_ATTR(mode,						S_IWUSR | S_IRUGO, NULL,   							epl_sensor_store_mode							);
static DRIVER_ATTR(wait_time,					S_IWUSR | S_IRUGO, NULL,   					 		epl_sensor_store_wait_time						);
static DRIVER_ATTR(set_threshold,     			S_IWUSR | S_IRUGO, epl_sensor_show_threshold,                epl_sensor_store_threshold						);
static DRIVER_ATTR(cal_raw, 					S_IWUSR | S_IRUGO, epl_sensor_show_cal_raw, 	  		NULL										);
static DRIVER_ATTR(unlock,				        S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_unlock						);
static DRIVER_ATTR(als_ch,				        S_IWUSR | S_IRUGO, NULL,			                    epl_sensor_store_als_ch_sel					);
static DRIVER_ATTR(run_ps_cali, 				S_IWUSR | S_IRUGO, epl_sensor_show_ps_run_cali, 	  		NULL										);
static DRIVER_ATTR(pdata,                       S_IWUSR | S_IRUGO, epl_sensor_show_pdata, NULL);
static DRIVER_ATTR(als_data,                    S_IWUSR | S_IRUGO, epl_sensor_show_als_data, NULL);
#if ALS_DYN_INTT
static DRIVER_ATTR(als_dyn_c_gain,              S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_c_gain                     );
static DRIVER_ATTR(als_dyn_lsrc_type,           S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_lsrc_type                  );
static DRIVER_ATTR(als_dyn_lsrc_thd,            S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_lsrc_thd                   );
#endif
static DRIVER_ATTR(i2c_w,                       S_IWUSR | S_IRUGO, NULL,                               epl_sensor_store_reg_write                  );
#if !EINT_API
static DRIVER_ATTR(trig_intr_type,					S_IWUSR | S_IRUGO, epl_sensor_show_sen,           epl_sensor_store_intr_type                    );
#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150421 begin*/
#if LENOVO_CALI
static DRIVER_ATTR(cali_param_1,                S_IWUSR | S_IRUGO, epl_sensor_show_ps_h_threshold,     epl_sensor_store_ps_h_threshold                  );
static DRIVER_ATTR(cali_param_2,                S_IWUSR | S_IRUGO, epl_sensor_show_ps_l_threshold,     epl_sensor_store_ps_l_threshold                  );
#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150421 end*/
/*lenovo-sw caoyi1 add for selftest function 20150715 begin*/
static DRIVER_ATTR(selftest,                S_IWUSR | S_IRUGO, epl_sensor_show_selftest,           NULL                  );
/*lenovo-sw caoyi1 add for selftest function 20150715 end*/
/*lenovo-sw caoyi1 add for ps status attr 20150901 begin*/
static DRIVER_ATTR(ps_status,                S_IWUSR | S_IRUGO, epl_sensor_show_ps_status,           NULL                  );
/*lenovo-sw caoyi1 add for ps status attr 20150901 end*/

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl_sensor_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_polling_mode,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_ps_offset,
    &driver_attr_ps_polling_mode,
    &driver_attr_mode,
    &driver_attr_ir_mode,
    &driver_attr_ir_boost,
    &driver_attr_ir_drive,
    &driver_attr_ir_on,
    &driver_attr_interrupt_type,
    &driver_attr_cal_raw,
    &driver_attr_set_threshold,
    &driver_attr_wait_time,
    &driver_attr_osc,
    &driver_attr_integration,
    &driver_attr_adc,
    &driver_attr_cycle,
    &driver_attr_i2c_max_count,
    &driver_attr_unlock,
    &driver_attr_als_ch,
    &driver_attr_run_ps_cali,
    &driver_attr_pdata,
    &driver_attr_als_data,
#if ALS_DYN_INTT
    &driver_attr_als_dyn_c_gain,
    &driver_attr_als_dyn_lsrc_type,
    &driver_attr_als_dyn_lsrc_thd,
#endif
    &driver_attr_i2c_w,
#if !EINT_API
    &driver_attr_trig_intr_type,
#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150421 begin*/
#if LENOVO_CALI
    &driver_attr_cali_param_1,
    &driver_attr_cali_param_2,
#endif
/*lenovo-sw caoyi1 add for Psensor calibration ATTR 20150421 end*/
/*lenovo-sw caoyi1 add for selftest function 20150715 begin*/
    &driver_attr_selftest,
/*lenovo-sw caoyi1 add for selftest function 20150715 end*/
/*lenovo-sw caoyi1 add for ps status attr 20150901 begin*/
    &driver_attr_ps_status,
/*lenovo-sw caoyi1 add for ps status attr 20150901 end*/
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl_sensor_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_sensor_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_sensor_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_sensor_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_sensor_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*lenovo-sw caoyi1 modify for PS calibration begin*/
#if LENOVO_CALI
static int epl_sensor_ps_average_val = 0;

static void epl_sensor_WriteCalibration(struct epl_sensor_priv *obj, HWMON_PS_STRUCT *data_cali)
{
	// struct PS_CALI_DATA_STRUCT *ps_data_cali;
	APS_LOG("le_WriteCalibration  1 %d," ,data_cali->close);
	APS_LOG("le_WriteCalibration  2 %d," ,data_cali->far_away);
	APS_LOG("le_WriteCalibration  3 %d,", data_cali->valid);
	//APS_LOG("le_WriteCalibration  4 %d,", data_cali->pulse);

	if(data_cali->valid == 1)
	{
		//atomic_set(&obj->ps_thd_val_high, data_cali->close);
		//atomic_set(&obj->ps_thd_val_low, data_cali->far_away);
		epl_sensor.ps.high_threshold = data_cali->close;
		epl_sensor.ps.low_threshold = data_cali->far_away;
	}else{
		/*lenovo-sw molg1 add default value for uncalibration 20141222 begin*/
		epl_sensor.ps.high_threshold = 4000;
		epl_sensor.ps.low_threshold = 3000;
		/*lenovo-sw molg1 add default value for uncalibration 20141222 end*/
	}
}

static int epl_sensor_read_data_for_cali(struct i2c_client *client, HWMON_PS_STRUCT *ps_data_cali)
{
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	struct epl_sensor_priv *obj = i2c_get_clientdata(client);
	int als_time = 0, ps_time = 0;
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
	int i=0 ,res = 0;
	/*Lenovo huangdra 20130713 modify u16 will overflow,change to u32*/
	u16 data[32];
	u32 sum=0, data_cali=0;
	int enable_ps = 0;

	als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	enable_ps = test_bit(CMC_BIT_PS, &obj->enable);

	if(enable_ps == 0)
	{
		set_bit(CMC_BIT_PS, &obj->enable);
		epl_sensor_update_mode(client);
	}
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/

	for(i = 0; i < 10; i++)
	{
		mdelay(als_time+ps_time+wait_value[epl_sensor.wait>>4]);//50

		if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps(obj->client);
            mutex_unlock(&sensor_mutex);
        }
		/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
		data[i] = epl_sensor.ps.data.data;
		/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
		if(res != 0)
		{
			APS_ERR("epl_sensor_read_data_for_cali fail: %d\n", i);
			break;
		}
		else
		{
			APS_LOG("[%d]sample = %d\n", i, data[i]);
			sum += data[i];
		}
	}


	if(i < 10)
	{
		res=  -1;

		return res;
	}
	else
	{
		data_cali = sum / 10;
		epl_sensor_ps_average_val = data_cali;
		APS_LOG("epl_sensor_read_data_for_cali data = %d",data_cali);
		/*Lenovo-sw caoyi1 modify for Aubest p-sensor calibration 2014-08-06 start*/
		if( data_cali>6000)
		{
			APS_ERR("epl_sensor_read_data_for_cali fail value to high: %d\n", data_cali);
			return -2;
		}
#if PS_DYN_K_ONE
		if((data_cali>0)&&(data_cali <=10))
		{
			ps_data_cali->close =data_cali + dynk_thd_high;
			ps_data_cali->far_away =data_cali + dynk_thd_low;
		}
		else if(data_cali <=100)
		{
			ps_data_cali->close =data_cali + dynk_thd_high;
			ps_data_cali->far_away =data_cali + dynk_thd_low;
		}
		else if(data_cali <=200)
		{
			ps_data_cali->close =data_cali + dynk_thd_high;
			ps_data_cali->far_away =data_cali + dynk_thd_low;
		}
		else if(data_cali<6000)
		{
			ps_data_cali->close =data_cali + dynk_thd_high;
			ps_data_cali->far_away =data_cali + dynk_thd_low;
		}
		else
		{
			APS_ERR("epl_sensor_read_data_for_cali IR current 0\n");
			return -2;
		}
#else
		if((data_cali>0)&&(data_cali <=10))
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali <=100)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali <=200)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali<6000)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else
		{
			APS_ERR("epl_sensor_read_data_for_cali IR current 0\n");
			return -2;
		}
#endif
		ps_data_cali->valid = 1;
		/*Lenovo-sw caoyi1 modify for Aubest p-sensor calibration 2014-08-06 end*/
		APS_LOG("epl_sensor_read_data_for_cali close = %d,far_away = %d,valid = %d\n",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
		//APS_LOG("rpr400_read_data_for_cali avg=%d max=%d min=%d\n",data_cali, max, min);
	}

	return res;
}
#endif
/*lenovo-sw caoyi1 modify for PS calibration end*/

/*----------------------------------------------------------------------------*/
static long epl_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err = 0;
#if 0
    int ps_result;
	int ps_cali;
	int threshold[2];
#endif
/*lenovo-sw caoyi1 modify for PS calibration begin*/
#if LENOVO_CALI
    HWMON_PS_STRUCT ps_cali_temp;
#endif
/*lenovo-sw caoyi1 modify for PS calibration end*/
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable_ps != enable)
            {
                if(enable)
                {
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
                    //wake_lock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
                    //wake_unlock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
                }
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps_status(obj->client);
            mutex_unlock(&sensor_mutex);
            dat = epl_sensor.ps.compare_low >> 3;

            //unlock and run
            epl_sensor.ps.compare_reset = EPL_CMP_RUN;
            epl_sensor.ps.lock = EPL_UN_LOCK;
            epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps(obj->client);
            mutex_unlock(&sensor_mutex);
            dat = epl_sensor.ps.data.data;

            //unlock and run
            epl_sensor.ps.compare_reset = EPL_CMP_RUN;
            epl_sensor.ps.lock = EPL_UN_LOCK;
            epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

            APS_LOG("ioctl ps raw value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable_als != enable)
            {
                if(enable)
                {
                    set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
                    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
                    {
                        dynamic_intt_idx = dynamic_intt_init_idx;
                        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                    }
#endif
                }
                else
                {
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
                epl_sensor_update_mode(obj->client);
            }
            break;

        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA:
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                dat = dynamic_intt_lux;
                APS_LOG("ALS_DYN_INTT: ioctl get als raw data = %d\n", dat);
            }
            else
#endif
            {
                dat = epl_sensor.als.data.channels[1];
                APS_LOG("ioctl get als raw data = %d\n", dat);
            }

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:
            mutex_lock(&sensor_mutex);
            epl_sensor_read_als(obj->client);
            mutex_unlock(&sensor_mutex);
            dat = epl_sensor.als.data.channels[1];

            epl_sensor.als.compare_reset = EPL_CMP_RUN;
            epl_sensor.als.lock = EPL_UN_LOCK;
            epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

            APS_LOG("ioctl get als raw data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
#if 0
/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
                msleep(ALS_DELAY+2*PS_DELAY+30);
            }
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps(obj->client);
            mutex_unlock(&sensor_mutex);

            if(epl_sensor.ps.data.data > obj->hw->ps_threshold_high)
			{
			    ps_result = 0;
			}
			else
			    ps_result = 1;

			APS_LOG("[%s] ps_result = %d \r\n", __func__, ps_result);

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_IOCTL_CLR_CALI:
#if 0
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
#else

            APS_LOG("[%s]: ALSPS_IOCTL_CLR_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_GET_CALI:
#if 0
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
#else
            APS_LOG("[%s]: ALSPS_IOCTL_GET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_SET_CALI:
#if 0
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
#else
            APS_LOG("[%s]: ALSPS_IOCTL_SET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
#if 0
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
#else
            APS_LOG("[%s] set threshold high: %d, low: %d\n", __func__, threshold[0],threshold[1]);
            obj->hw->ps_threshold_high = threshold[0];
            obj->hw->ps_threshold_low = threshold[1];
            set_psensor_intr_threshold(obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
#endif
			break;

		case ALSPS_GET_PS_THRESHOLD_HIGH:
#if 0
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_high;
            APS_LOG("[%s] get threshold high: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
#if 0
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_low;
            APS_LOG("[%s] get threshold low: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		/*------------------------------------------------------------------------------------------*/
#endif

/*lenovo-sw caoyi1 modify for PS calibration begin*/
#if LENOVO_CALI
    case ALSPS_SET_PS_CALI:
			if(copy_from_user(&ps_cali_temp, ptr, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;
			}
			epl_sensor_WriteCalibration(obj, &ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
	break;

    case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			{

				cancel_delayed_work_sync(&obj->eint_work);
				disable_irq(epl_sensor_obj->irq);
				err = epl_sensor_read_data_for_cali(obj->client,&ps_cali_temp);
				if(err < 0 ){
					goto err_out;
				}
				epl_sensor_WriteCalibration(obj, &ps_cali_temp);
				if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
				{
					err = -EFAULT;
					goto err_out;
				}
			}
	break;

    case ALSPS_GET_PS_AVERAGE:
			enable = epl_sensor_ps_average_val;
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
	break;
			//lenovo-sw, shanghai, add by chenlj2, for geting ps average val, 2012-05-14 end
			//lenovo-sw, shanghai, add by chenlj2, for AVAGO project, 2012-10-09 start
		case ALSPS_GET_PS_FAR_THRESHOLD:
			enable = epl_sensor.ps.low_threshold;
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_CLOSE_THRESHOLD:
			enable = epl_sensor.ps.high_threshold;
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			//lenovo-sw, shanghai, add by chenlj2, for AVAGO project, 2012-10-09 end
			/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 end*/

#endif
/*lenovo-sw caoyi1 modify for PS calibration end*/
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl_sensor_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_open,
    .release = epl_sensor_release,
    .unlocked_ioctl = epl_sensor_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl_sensor_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_sensor_fops,
};


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
/*lenovo-sw caoyi1 add for null point 20150828 begin*/
        if ( obj == NULL ) {
                APS_ERR("null pointer!!\n");
                return -EINVAL;;
        }
/*lenovo-sw caoyi1 add for null point 20150828 end*/
    //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 0
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);
        if(enable_ps == 1){
            atomic_set(&obj->ps_suspend, 0);
            APS_LOG("[%s]: ps enable \r\n", __func__);
        }
        else{
            atomic_set(&obj->ps_suspend, 1);
            APS_LOG("[%s]: ps disable \r\n", __func__);
            epl_sensor_update_mode(obj->client);
        }

        epl_sensor_power(obj->hw, 0);
    }
#endif
    APS_LOG("[%s]: finish \r\n", __func__);
    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_resume(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
/*lenovo-sw caoyi1 add for null point 20150828 begin*/
        if ( obj == NULL ) {
                APS_ERR("null pointer!!\n");
                return -EINVAL;
        }
/*lenovo-sw caoyi1 add for null point 20150828 end*/
    //bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    //bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
    APS_FUN();
#if 0
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->als_suspend, 0);
    if(enable_ps == 1)
    {
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }

    epl_sensor_power(obj->hw, 1);
#endif
    APS_LOG("[%s]: finish \r\n", __func__);
    return 0;
}



/*----------------------------------------------------------------------------*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void epl_sensor_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
    if(enable_ps == 1){
        //atomic_set(&obj->ps_suspend, 0);
        APS_LOG("[%s]: ps is enabled! \r\n", __func__);
    }
    else{
        //atomic_set(&obj->ps_suspend, 1);
        APS_LOG("[%s]: ps is disabled! \r\n", __func__);
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150806 begin*/
        //epl_sensor_update_mode(obj->client);
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150806 end*/
    }


}



/*----------------------------------------------------------------------------*/
static void epl_sensor_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150806 begin*/
    //inti ALS setting
#if 0 //ALS_DYN_INTT
    if(enable_als == 1 && epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        dynamic_intt_idx = dynamic_intt_init_idx;
        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
#if ALS_DYN_INTT_FAST
        als_first_repot = true;
        dynamic_intt_fast_idx = 0;
        epl_sensor.als.cycle = EPL_CYCLE_4;
        APS_LOG("[%s]: ALS fast mode \r\n", __func__);
#endif
    }
#endif

    if(enable_ps == 1)
    {
        //atomic_set(&obj->ps_suspend, 0);
        APS_LOG("[%s]: ps is enabled! \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps is disabled! \r\n", __func__);
        //epl_sensor_update_mode(obj->client);
    }

#if 0
    if(enable_als == 1)
    {
        do
        {
            mutex_lock(&sensor_mutex);
            epl_sensor_read_als(obj->client);
            mutex_unlock(&sensor_mutex);

            epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
            APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
            dynamic_intt_fast_idx++;
            APS_LOG("[%s]: dynamic_intt_fast_idx=%d \r\n", __func__, dynamic_intt_fast_idx);
        }while(-1 == dynamic_intt_lux);

        if(als_first_repot == true && dynamic_intt_lux != -1)
        {
            als_first_repot = false;
            APS_LOG("[%s]: First ALS report = %d\n", __func__, dynamic_intt_lux);
    	    als_data_report(alsps_context_obj->idev, dynamic_intt_lux, SENSOR_STATUS_ACCURACY_MEDIUM);
        }
    }
#endif
    APS_LOG("[%s]: finish \r\n", __func__);
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150806 end*/
}
#endif

#if MTK_LTE /*MTK_LTE start .................*/
/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150703 begin*/
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150703 end*/
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150703 begin*/
	APS_LOG("[%s] als enable en = %d, als_suspend = %d\n", __func__, en,  atomic_read(&obj->als_suspend));
/*lenovo-sw caoyi1 modify for late_resume init als fastmode 20150703 end*/


    if(enable_als != en)
    {
        if(en)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
#if ALS_DYN_INTT_FAST
                als_first_repot = true;
                dynamic_intt_fast_idx = 0;
/*lenovo-sw caoyi1 modify for als fast 20150921 begin*/
                epl_sensor.als.cycle = EPL_CYCLE_1; //EPL_CYCLE_4;
/*lenovo-sw caoyi1 modify for als fast 20150921 end*/
                       APS_LOG("[%s]: ALS fast mode \r\n", __func__);
#endif
/*lenovo-sw caoyi1 add for als report data fast 20150624 end*/
            }
#endif
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);

/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
            //If als is disable, update ALS value for HAL layer
            APS_LOG("[%s]: update ALS value is -2 for HAL \r\n", __func__);
/*lenovo-sw caoyi1 add 20150918 begin*/
            als_data_report(alsps_context_obj->idev_als, -2, SENSOR_STATUS_ACCURACY_MEDIUM);
/*lenovo-sw caoyi1 add 20150918 end*/
/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/

        }
        epl_sensor_update_mode(obj->client);

/*lenovo-sw caoyi modify for als resume report fast 20150806 begin*/
        if(en == 1)
        {
            do
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);

                epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
                dynamic_intt_fast_idx++;
                APS_LOG("[%s]: dynamic_intt_fast_idx=%d \r\n", __func__, dynamic_intt_fast_idx);
            }while(-1 == dynamic_intt_lux);

            if(als_first_repot == true && dynamic_intt_lux != -1)
            {
		als_first_repot = false;
		APS_LOG("[%s]: First ALS report = %d\n", __func__, dynamic_intt_lux);
/*lenovo-sw caoyi1 add 20150918 begin*/
		als_data_report(alsps_context_obj->idev_als, dynamic_intt_lux, SENSOR_STATUS_ACCURACY_MEDIUM);
/*lenovo-sw caoyi1 add 20150918 end*/
            }
        }
/*lenovo-sw caoyi modify for als resume report fast 20150806 end*/
    }

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;
/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
	int als_value = -1;
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/
	struct epl_sensor_priv *obj = epl_sensor_obj;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}
/*lenovo-sw caoyi1 add for als report data fast 20150618 begin*/
#if ALS_DYN_INTT_FAST
    if(polling_flag == true && eint_flag == true)
    {
        do
        {

        mutex_lock(&sensor_mutex);
        if(!((epl_sensor.mode&0x08)>>3))    //if ges mode, update ALS in ges process
        {
            epl_sensor_read_als(obj->client);
        }
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client, 0x11, epl_sensor.power | epl_sensor.reset);
        mutex_unlock(&sensor_mutex);


                epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
                als_value = dynamic_intt_lux;
                dynamic_intt_fast_idx++;
                APS_LOG("[%s]: dynamic_intt_fast_idx=%d \r\n", __func__, dynamic_intt_fast_idx);

        }while(-1 == als_value);
    }
    else
    {
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);
        als_value = -1;
    }
    *value = als_value;

/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
    if(als_first_repot == true && als_value != -1)
    {
        als_first_repot = false;
        APS_LOG("[%s]: First ALS report = %d\n", __func__, als_value);
	als_data_report(alsps_context_obj->idev_als, als_value, SENSOR_STATUS_ACCURACY_MEDIUM);
    }
/*lenovo-sw caoyi1 add for als report data fast 20150624 begin*/
#else
    if(polling_flag == true && eint_flag == true)
    {
        mutex_lock(&sensor_mutex);
        if(!((epl_sensor.mode&0x08)>>3))    //if ges mode, update ALS in ges process
        {
            epl_sensor_read_als(obj->client);
        }
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client, 0x11, epl_sensor.power | epl_sensor.reset);
        mutex_unlock(&sensor_mutex);
    }
    else
    {
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);
        dynamic_intt_lux = -1;  //change mode or change INTEG, -1 is ignore
    }

    if(epl_sensor.als.report_type != CMC_BIT_DYN_INT){
        *value = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
    }
#if ALS_DYN_INTT
    else{
        if(polling_flag == true && eint_flag == true){
            epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
            APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
        }
        *value = dynamic_intt_lux;
    }
#endif
#endif
/*lenovo-sw caoyi1 add for als report data fast 20150618 end*/

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_LOG("ps enable = %d\n", en);

    if(enable_ps != en)
    {
        if(en)
        {
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
            //wake_lock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
            set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K_ONE
            ps_dyn_flag = true;
#endif
        }
        else
        {

            clear_bit(CMC_BIT_PS, &obj->enable);
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
            //wake_unlock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
        }
        epl_sensor_update_mode(client);
    }

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{

    int err = 0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;

    APS_LOG("---ps_get_data---\n\n");

    if(polling_flag == true && eint_flag == true)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps_status(client);
        epl_sensor_read_ps(client);
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
        mutex_unlock(&sensor_mutex);
    }
    else
    {
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running\r\n", __func__, polling_flag, eint_flag);
	}

    *value = epl_sensor.ps.compare_low >> 3;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*----------------------------------------------------------------------------*/
#else /*MTK_LTE*/
/*----------------------------------------------------------------------------*/
int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    APS_LOG("epl_sensor_ps_operate command = %x\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            //msleep(5);
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                APS_LOG("ps enable = %d\n", value);
                if(enable_ps != value)
                {
                    if(value)
                    {
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
                        //wake_lock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
                        set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K_ONE
                        ps_dyn_flag = true;
#endif
                    }
                    else
                    {

                        clear_bit(CMC_BIT_PS, &obj->enable);
/*lenovo-sw caoyi modify ps wake_lock 20150720 begin*/
                        //wake_unlock(&ps_lock);
/*lenovo-sw caoyi modify ps wake_lock 20150720 end*/
                    }
                    epl_sensor_update_mode(client);
                }
            }

            break;

        case SENSOR_GET_DATA:
            APS_LOG(" get ps data !!!!!!\n");

            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                if(polling_flag == true && eint_flag == true)
                    first_als_report_flag = false;
                err = -EINVAL;
            }
            else
            {
                APS_LOG("---SENSOR_GET_DATA---\n\n");
                if(polling_flag == true && eint_flag == true)
                {
                    mutex_lock(&sensor_mutex);
                    epl_sensor_read_ps_status(client);
                    epl_sensor_read_ps(client);
                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
                    mutex_unlock(&sensor_mutex);
     		    }
     		    else
     		    {
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running\r\n", __func__, polling_flag, eint_flag);
     		    }

                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = epl_sensor.ps.compare_low >> 3;
                sensor_data->values[1] = epl_sensor.ps.data.data;
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;


        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;



    }

    return err;

}


int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value = 0;
    hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
    APS_LOG("epl_sensor_als_operate command = %x\n",command);

    switch (command)
    {
        case SENSOR_DELAY:
            //msleep(5);
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;


        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("als enable = %d\n", value);

                value = *(int *)buff_in;
                if(enable_als != value)
                {
                    if(value)
                    {
                        set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
                        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
                        {
                            dynamic_intt_idx = dynamic_intt_init_idx;
                            epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                            epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                        }
#endif
                    }
                    else
                    {
                        clear_bit(CMC_BIT_ALS, &obj->enable);
                    }
                    epl_sensor_update_mode(client);
                }
            }
            break;


        case SENSOR_GET_DATA:
            APS_LOG("get als data !!!!!!\n");

            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) || first_als_report_flag == true)
            {
                APS_ERR("get sensor data parameter error!\n");
                if(polling_flag == true && eint_flag == true)
                    first_als_report_flag = false;
                err = -EINVAL;
            }
            else
            {
                if(polling_flag == true && eint_flag == true)
                {
                    mutex_lock(&sensor_mutex);
                    if(!((epl_sensor.mode&0x08)>>3))    //if ges mode, update ALS in ges process
                    {
                        epl_sensor_read_als(client);
                    }
                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
                    mutex_unlock(&sensor_mutex);
                }
                else
                {
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);
                }
                sensor_data = (hwm_sensor_data *)buff_out;

                if(epl_sensor.als.report_type != CMC_BIT_DYN_INT){
                    sensor_data->values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                }
#if ALS_DYN_INTT
                else{
                    if(polling_flag == true && eint_flag == true){
                        epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                        APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
                    }
                    sensor_data->values[0] = dynamic_intt_lux;
                }
#endif
                sensor_data->values[1] = epl_sensor.als.data.channels[1];
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]);
                if(epl_sensor.wait == EPL_WAIT_SINGLE)
                    epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
            }
            break;

        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;

    }

    return err;

}
#endif

/*----------------------------------------------------------------------------*/
#if 0
static int epl_sensor_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, EPL_DEV_NAME);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_sensor_priv *obj;

#if MTK_LTE
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
#else
    struct hwmsen_object obj_ps, obj_als;
#endif
    int err = 0; //, i=0;
    APS_FUN();

    epl_sensor_dumpReg(client);

    if((i2c_smbus_read_byte_data(client, 0x21)) != EPL_REVNO){
        APS_LOG("elan ALS/PS sensor is failed. \n");
        goto exit;
    }

    //client->timing = 400;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl_sensor_obj = obj;
    //obj->hw = get_cust_alsps_hw();
    obj->hw =hw;

    epl_sensor_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));


    INIT_DELAYED_WORK(&obj->eint_work, epl_sensor_eint_work);
    INIT_DELAYED_WORK(&obj->polling_work, epl_sensor_polling_work);

    obj->client = client;

    mutex_init(&sensor_mutex);
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
#if EINT_API
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
#endif
    obj->enable = 0;
    obj->pending_intr = 0;

    epl_sensor_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);

    if((err = epl_sensor_init_client(client)))
    {
        goto exit_init_failed;
    }

    if((err = misc_register(&epl_sensor_device)))
    {
        APS_ERR("epl_sensor_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#if MTK_LTE /*MTK_LTE start .................*/
    if((err = epl_sensor_create_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = false;


	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}


	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
/*lenovo-sw caoyi1 modify for Psensor input report event 20150511 begin*/
	ps_ctl.is_report_input_direct = true; //false
/*lenovo-sw caoyi1 modify for Psensor input report event 20150511 end*/
    ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

#else /*MTK_LTE */

    if((err = epl_sensor_create_attr(&epl_sensor_alsps_driver.driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = epl_sensor_obj;

    ps_hw = &obj_ps;

    if( obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }


    obj_als.self = epl_sensor_obj;

    als_hw = &obj_als;

    if( obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_als.sensor_operate = epl_sensor_als_operate;
    APS_LOG("als polling mode\n");


    if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

#endif  /*MTK_LTE end .................*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    obj->early_drv.suspend  = epl_sensor_early_suspend,
    obj->early_drv.resume   = epl_sensor_late_resume,
    register_early_suspend(&obj->early_drv);
#endif

    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        epl_sensor_setup_eint(client);
#if MTK_LTE
    alsps_init_flag = 0;
#endif
    APS_LOG("%s: OK\n", __FUNCTION__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&epl_sensor_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_sensor_i2c_client = NULL;
#if MTK_LTE
    alsps_init_flag = -1;
#endif
    APS_ERR("%s: err = %d\n", __FUNCTION__, err);
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_remove(struct i2c_client *client)
{
    int err;
#if MTK_LTE
    if((err = epl_sensor_delete_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#else
    if((err = epl_sensor_delete_attr(&epl_sensor_i2c_driver.driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#endif
    if((err = misc_deregister(&epl_sensor_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl_sensor_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}


#if !MTK_LTE
/*----------------------------------------------------------------------------*/
static int epl_sensor_probe(struct platform_device *pdev)
{
    struct alsps_hw *hw = get_cust_alsps_hw();

    epl_sensor_power(hw, 1);

    if(i2c_add_driver(&epl_sensor_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_remove(struct platform_device *pdev)
{
    struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("EPL8801 remove \n");
    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}



/*----------------------------------------------------------------------------*/
static struct platform_driver epl_sensor_alsps_driver =
{
    .probe      = epl_sensor_probe,
    .remove     = epl_sensor_remove,
    .driver     = {
        .name  = "als_ps",
        //.owner = THIS_MODULE,
    }
};
#endif

#if MTK_LTE
/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
	epl_sensor_power(hw, 1);

	if(i2c_add_driver(&epl_sensor_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
    //struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("epl_sensor remove \n");

    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init epl_sensor_init(void)
{
    const char *name = COMPATIABLE_NAME;
    APS_FUN();

    hw = get_alsps_dts_func(name, hw);

    if (!hw)
    {
    	APS_ERR("get dts info fail\n");
    }
#if MTK_LTE
	//i2c_register_board_info(hw->i2c_num, &i2c_epl_sensor, 1);
    alsps_driver_add(&epl_sensor_init_info);
#else
    i2c_register_board_info(hw->i2c_num, &i2c_epl_sensor, 1);
    if(platform_driver_register(&epl_sensor_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
#endif


    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl_sensor_exit(void)
{
    APS_FUN();
#if !MTK_LTE
    platform_driver_unregister(&epl_sensor_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL8801 ALPsr driver");
MODULE_LICENSE("GPL");

