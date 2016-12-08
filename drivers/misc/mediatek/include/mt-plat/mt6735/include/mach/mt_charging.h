#ifndef _CUST_BAT_H_
#define _CUST_BAT_H_

/* stop charging while in talking mode */
//lenovo-sw mahj2 modify for enable charge in talking Begin
//#define STOP_CHARGING_IN_TAKLING
//lenovo-sw mahj2 modify for enable charge in talking End
#define TALKING_RECHARGE_VOLTAGE 3800
#define TALKING_SYNC_TIME		   60

/* Battery Temperature Protection */
#define MTK_TEMPERATURE_RECHARGE_SUPPORT
#define MAX_CHARGE_TEMPERATURE  50
#define MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE	47
#define MIN_CHARGE_TEMPERATURE  0
#define MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE	6
#define ERR_CHARGE_TEMPERATURE  0xFF

/* Linear Charging Threshold */
#define V_PRE2CC_THRES 3400
#define V_CC2TOPOFF_THRES		4050
#define RECHARGING_VOLTAGE      4110
#define CHARGING_FULL_CURRENT    100

/* Charging Current Setting */
#define USB_CHARGER_CURRENT_SUSPEND			0
#define USB_CHARGER_CURRENT_UNCONFIGURED	CHARGE_CURRENT_70_00_MA
#define USB_CHARGER_CURRENT_CONFIGURED		CHARGE_CURRENT_500_00_MA

#define USB_CHARGER_CURRENT					CHARGE_CURRENT_500_00_MA
//lenovo-sw mahj2 modify at 20150318 Begin
#define AC_CHARGER_CURRENT					CHARGE_CURRENT_2050_00_MA
//lenovo-sw mahj2 modify at 20150318 End
#define NON_STD_AC_CHARGER_CURRENT			CHARGE_CURRENT_500_00_MA
#define CHARGING_HOST_CHARGER_CURRENT       CHARGE_CURRENT_650_00_MA
#define APPLE_0_5A_CHARGER_CURRENT          CHARGE_CURRENT_500_00_MA
#define APPLE_1_0A_CHARGER_CURRENT          CHARGE_CURRENT_650_00_MA
#define APPLE_2_1A_CHARGER_CURRENT          CHARGE_CURRENT_800_00_MA
/*Begin, lenovo-sw mahj2 modify for current limit at 20141113 */
#define AC_CHARGER_CURRENT_LIMIT	              CHARGE_CURRENT_1000_00_MA   //lenovo standard 0.3C
/*End, lenovo-sw mahj2 modify for current limit at 20141113 */

/*Begin, lenovo-sw chailu1 add for 45-50  0.5c   20150318 */
#define LENOVO_AC_CHARGER_CURRENT_LIMIT_HTEMP_SUPPORT
#ifdef LENOVO_AC_CHARGER_CURRENT_LIMIT_HTEMP_SUPPORT
#define AC_CHARGER_CURRENT_LIMIT_HTEMP	    CHARGE_CURRENT_1650_00_MA//0.5c
#endif
/*End, lenovo-sw chailu1 add for 45-50  0.5c   20150318 */

/*Begin, lenovo-sw mahj2 add 45-50  CV limit fuction */
#define LENOVO_TEMP_POS_45_TO_POS_50_CV_LiMIT_SUPPORT
#ifdef LENOVO_TEMP_POS_45_TO_POS_50_CV_LiMIT_SUPPORT
#define LENOVO_TEMP_POS_45_TO_POS_50_CV_VOLTAGE		BATTERY_VOLT_04_140000_V	   
#endif
/*End, lenovo-sw mahj2 add 45-50  CV limit fuction */

/* Precise Tunning */
#define BATTERY_AVERAGE_DATA_NUMBER	3
#define BATTERY_AVERAGE_SIZE 30

/* charger error check */
#define V_CHARGER_ENABLE 0				/* 1:ON , 0:OFF	*/
#define V_CHARGER_MAX 6500				/* 6.5 V	*/
#define V_CHARGER_MIN 4400				/* 4.4 V	*/

/* Tracking TIME */
#define ONEHUNDRED_PERCENT_TRACKING_TIME	10	/* 10 second	*/
#define NPERCENT_TRACKING_TIME 20	/* 20 second	*/
#define SYNC_TO_REAL_TRACKING_TIME 60	/* 60 second	*/
#define V_0PERCENT_TRACKING							3450 /*3450mV	*/

/* Battery Notify */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
//lenovo-sw mahj2 modify for bat temp protect Begin
//#define BATTERY_NOTIFY_CASE_0002_VBATTEMP
//lenovo-sw mahj2 modify for bat temp protect End

//#define BATTERY_NOTIFY_CASE_0003_ICHARGING
//lenovo-sw mahj2 modify for opening bat vlotage protect Begin
#define BATTERY_NOTIFY_CASE_0004_VBAT
//lenovo-sw mahj2 modify for opening bat vlotage protect End
//#define BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME

/* High battery support */
//lenovo-sw mahj2 modify for high battery voltage Begin
#define HIGH_BATTERY_VOLTAGE_SUPPORT
//lenovo-sw mahj2 modify for high battery voltage End

/* JEITA parameter */
/*#define MTK_JEITA_STANDARD_SUPPORT*/
#define CUST_SOC_JEITA_SYNC_TIME 30
#define JEITA_RECHARGE_VOLTAGE  4110	/* for linear charging */
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
#define JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE		BATTERY_VOLT_04_340000_V
#define JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE		BATTERY_VOLT_04_040000_V
#define JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE		BATTERY_VOLT_04_040000_V
#else
#define JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE	BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE	BATTERY_VOLT_04_200000_V
#define JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE	BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE	BATTERY_VOLT_03_900000_V
#define JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE		BATTERY_VOLT_03_900000_V
#endif
/* For JEITA Linear Charging only */
#define JEITA_NEG_10_TO_POS_0_FULL_CURRENT  120
#define JEITA_TEMP_POS_45_TO_POS_60_RECHARGE_VOLTAGE  4000
#define JEITA_TEMP_POS_10_TO_POS_45_RECHARGE_VOLTAGE  4100
#define JEITA_TEMP_POS_0_TO_POS_10_RECHARGE_VOLTAGE   4000
#define JEITA_TEMP_NEG_10_TO_POS_0_RECHARGE_VOLTAGE   3800
#define JEITA_TEMP_POS_45_TO_POS_60_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_POS_10_TO_POS_45_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_POS_0_TO_POS_10_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_NEG_10_TO_POS_0_CC2TOPOFF_THRESHOLD	3850


/*lenovo-sw weiweij added for charging terminate as 0.1c*/
#ifndef CONFIG_MTK_HAFG_20
#define LENOVO_CHARGING_TERM
#endif

#ifdef LENOVO_CHARGING_TERM
#define LENOVO_CHARGING_TERM_CUR_STAGE_1	1	//256ma
#define LENOVO_CHARGING_TERM_CUR_STAGE_2	0	//128ma
#endif
/*lenovo-sw weiweij added for charging terminate as 0.1c end*/
/* For CV_E1_INTERNAL */
#define CV_E1_INTERNAL
/* lenovo-sw mahj2 enable pmic to control charging led 2015-06-25 */
#define LENOVO_USE_PMIC_CONTROL_LED
/* lenovo-sw mahj2 enable pmic to control charging led 2015-06-25 */
/*lenovo-sw mahj2 added for ntc temp cut 1 degree Begin*/
#define LENOVO_NTC_TEMP_CUT_2_DEGREE
/*lenovo-sw mahj2 added for ntc temp cut 1 degree End*/
//lenovo-sw mahj2 add for show charging ic id Begin
#define LENOVO_SHOW_CHARGING_IC_ID
//lenovo-sw mahj2 add for show charging ic id End
//lenovo-sw mahj2 add for thermal support Begin
#define LENOVO_THERMAL_SUPPORT
//lenovo-sw mahj2 add for thermal support End

/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define CONFIG_DIS_CHECK_BATTERY
#endif

#ifdef CONFIG_MTK_FAN5405_SUPPORT
#define FAN5405_BUSNUM 1
#endif

#define MTK_PLUG_OUT_DETECTION

#endif /* _CUST_BAT_H_ */
