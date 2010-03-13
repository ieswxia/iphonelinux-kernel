#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <mach/pmu.h>
#include "i2c.h"
#include "gpio.h"

typedef struct PMURegisterData {
	uint8_t reg;
	uint8_t data;
} PMURegisterData;

typedef enum PowerSupplyType {
	PowerSupplyTypeError,
	PowerSupplyTypeBattery,
	PowerSupplyTypeFirewire,
	PowerSupplyTypeUSBHost,
	PowerSupplyTypeUSBBrick500mA,
	PowerSupplyTypeUSBBrick1000mA
} PowerSupplyType;

static int iphone_pmu_get_reg(int reg) {
	uint8_t registers[1];
	uint8_t out[1];

	registers[0] = reg;

	iphone_i2c_rx(PMU_I2C_BUS, PMU_GETADDR, registers, 1, out, 1);
	return out[0];
}

static int iphone_pmu_write_reg(int reg, int data, int verify) {
	uint8_t command[2];
	uint8_t pmuReg = reg;
	uint8_t buffer = 0;

	command[0] = reg;
	command[1] = data;

	iphone_i2c_tx(PMU_I2C_BUS, PMU_SETADDR, command, sizeof(command));

	if(!verify)
		return 0;

	iphone_i2c_rx(PMU_I2C_BUS, PMU_GETADDR, &pmuReg, 1, &buffer, 1);

	if(buffer == data)
		return 0;
	else
		return -1;
}

/*static int iphone_pmu_write_regs(const PMURegisterData* regs, int num) {
	int i;
	for(i = 0; i < num; i++) {
		iphone_pmu_write_reg(regs[i].reg, regs[i].data, 1);
	}

	return 0;
}

static void iphone_pmu_write_oocshdwn(int data) {
	uint8_t registers[1];
	uint8_t discardData[5];
	uint8_t poweroffData[] = {7, 0xAA, 0xFC, 0x0, 0x0, 0x0};
	registers[0] = 2;
	iphone_i2c_rx(PMU_I2C_BUS, PMU_GETADDR, registers, sizeof(registers), discardData, sizeof(data));
	iphone_i2c_tx(PMU_I2C_BUS, PMU_SETADDR, poweroffData, sizeof(poweroffData));
	iphone_pmu_write_reg(PMU_OOCSHDWN, data, 0);
	while(1) {
		udelay(100000);
	}
}

static void iphone_pmu_poweroff(void) {
	//lcd_shutdown();
	iphone_pmu_write_oocshdwn(PMU_OOCSHDWN_GOSTBY);
}

static int query_adc(int flags) {
	uint8_t lower;
	iphone_pmu_write_reg(PMU_ADCC3, 0, 0);
	iphone_pmu_write_reg(PMU_ADCC3, 0, 0);
	udelay(30);
	iphone_pmu_write_reg(PMU_ADCC2, 0, 0);
	iphone_pmu_write_reg(PMU_ADCC1, PMU_ADCC1_ADCSTART | (PMU_ADCC1_ADC_AV_16 << PMU_ADCC1_ADC_AV_SHIFT) | (PMU_ADCC1_ADCINMUX_BATSNS_DIV << PMU_ADCC1_ADCINMUX_SHIFT) | flags, 0);
	udelay(30000);
	lower = iphone_pmu_get_reg(PMU_ADCS3);
	if((lower & 0x80) == 0x80) {
		uint8_t upper = iphone_pmu_get_reg(PMU_ADCS1);
		return ((upper << 2) | (lower & 0x3)) * 6000 / 1023;
	} else {
		return -1;
	}
}

static PowerSupplyType identify_usb_charger(void) {
	int dn;
	int dp;
	int x;

	iphone_gpio_pin_output(PMU_GPIO_CHARGER_IDENTIFY_DN, 1);
	dn = query_adc(PMU_ADCC1_ADCINMUX_ADCIN2_DIV << PMU_ADCC1_ADCINMUX_SHIFT);
	if(dn < 0)
		dn = 0;
	iphone_gpio_pin_output(PMU_GPIO_CHARGER_IDENTIFY_DN, 0);

	iphone_gpio_pin_output(PMU_GPIO_CHARGER_IDENTIFY_DP, 1);
	dp = query_adc(PMU_ADCC1_ADCINMUX_ADCIN2_DIV << PMU_ADCC1_ADCINMUX_SHIFT);
	if(dp < 0)
		dp = 0;
	iphone_gpio_pin_output(PMU_GPIO_CHARGER_IDENTIFY_DP, 0);

	if(dn < 99 || dp < 99) {
		return PowerSupplyTypeUSBHost;
	}

	x = (dn * 1000) / dp;
	if((x - 1291) <= 214) {
		return PowerSupplyTypeUSBBrick1000mA;
	}

	if((x - 901) <= 219 && dn <= 367 ) {
		return PowerSupplyTypeUSBBrick500mA;
	} else {
		return PowerSupplyTypeUSBHost;
	}
}

static PowerSupplyType iphone_pmu_get_power_supply(void) {
	int mbcs1 = iphone_pmu_get_reg(PMU_MBCS1);

	if(mbcs1 & PMU_MBCS1_ADAPTPRES)
		return PowerSupplyTypeFirewire;

	if(mbcs1 & PMU_MBCS1_USBOK)
		return identify_usb_charger();
	else
		return PowerSupplyTypeBattery;

}

static void iphone_pmu_charge_settings(int UseUSB, int SuspendUSB, int StopCharger) {
	PowerSupplyType type = iphone_pmu_get_power_supply();

	if(type != PowerSupplyTypeUSBHost)	// No need to suspend USB, since we're not plugged into a USB host
		SuspendUSB = 0;

	if(SuspendUSB)
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_SUSPEND, 1);
	else
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_SUSPEND, 0);

	if(StopCharger) {
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_SUSPEND, 1);
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_SHUTDOWN, 1);
	} else {
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_SUSPEND, 0);
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_SHUTDOWN, 0);
	}

	if(type == PowerSupplyTypeUSBBrick500mA || type == PowerSupplyTypeUSBBrick1000mA || (type == PowerSupplyTypeUSBHost && UseUSB))
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_500_1000, 1);
	else
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_500_1000, 0);

	if(type == PowerSupplyTypeUSBBrick1000mA)
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_1000, 1);
	else
		iphone_gpio_pin_output(PMU_GPIO_CHARGER_USB_1000, 0);
}

static int iphone_pmu_get_battery_voltage(void) {
	return query_adc(0);
}*/

static int iphone_pmu_get_seconds(void) {
	return bcd2bin(iphone_pmu_get_reg(PMU_RTCSC) & PMU_RTCSC_MASK);
}

static int iphone_pmu_get_minutes(void) {
	return bcd2bin(iphone_pmu_get_reg(PMU_RTCMN) & PMU_RTCMN_MASK);
}

static int iphone_pmu_get_hours(void) {
	return bcd2bin(iphone_pmu_get_reg(PMU_RTCHR) & PMU_RTCHR_MASK);
}

static int iphone_pmu_get_day(void) {
	return bcd2bin(iphone_pmu_get_reg(PMU_RTCDT) & PMU_RTCDT_MASK);
}

static int iphone_pmu_get_month(void) {
	return iphone_pmu_get_reg(PMU_RTCMT) & PMU_RTCMT_MASK;
}

static int iphone_pmu_get_year(void) {
	return bcd2bin(iphone_pmu_get_reg(PMU_RTCYR) & PMU_RTCYR_MASK);
}

static int iphone_pmu_set_seconds(int num) {
	return iphone_pmu_write_reg(PMU_RTCSC, bin2bcd(num) & PMU_RTCSC_MASK, 0);
}

static int iphone_pmu_set_minutes(int num) {
	return iphone_pmu_write_reg(PMU_RTCMN, bin2bcd(num) & PMU_RTCMN_MASK, 0);
}

static int iphone_pmu_set_hours(int num) {
	return iphone_pmu_write_reg(PMU_RTCHR, bin2bcd(num) & PMU_RTCHR_MASK, 0);
}

static int iphone_pmu_set_day(int num) {
	return iphone_pmu_write_reg(PMU_RTCDT, bin2bcd(num) & PMU_RTCDT_MASK, 0);
}

static int iphone_pmu_set_month(int num) {
	return iphone_pmu_write_reg(PMU_RTCMT, num & PMU_RTCMT_MASK, 0);
}

static int iphone_pmu_set_year(int num) {
	return iphone_pmu_write_reg(PMU_RTCYR, bin2bcd(num) & PMU_RTCYR_MASK, 0);
}

/*static int iphone_pmu_get_dayofweek(void) {
	return iphone_pmu_get_reg(PMU_RTCWD) & PMU_RTCWD_MASK;
}

static int iphone_pmu_set_dayofweek(int num) {
	return iphone_pmu_write_reg(PMU_RTCWD, num & PMU_RTCWD_MASK, 0);
}*/

static int iphone_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	rtc_tm->tm_sec = iphone_pmu_get_seconds();
	rtc_tm->tm_min = iphone_pmu_get_minutes();
	rtc_tm->tm_hour = iphone_pmu_get_hours();
	rtc_tm->tm_mday = iphone_pmu_get_day();
	rtc_tm->tm_mon = iphone_pmu_get_month() - 1;
	rtc_tm->tm_year = iphone_pmu_get_year() + 101;

	return 0;
}

static int iphone_rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	iphone_pmu_set_seconds(rtc_tm->tm_sec);
	iphone_pmu_set_minutes(rtc_tm->tm_min);
	iphone_pmu_set_hours(rtc_tm->tm_hour);
	iphone_pmu_set_day(rtc_tm->tm_mday);
	iphone_pmu_set_month(rtc_tm->tm_mon + 1);
	iphone_pmu_set_year(rtc_tm->tm_year - 101);

	return 0;
}

static const struct rtc_class_ops iphone_rtcops = {
	.read_time	= iphone_rtc_gettime,
	.set_time	= iphone_rtc_settime,
};

static int __devinit iphone_pmu_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret = 0;

	rtc = rtc_device_register("iphone", &pdev->dev, &iphone_rtcops,
			THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	platform_set_drvdata(pdev, rtc);

	return 0;

err_nortc:

	return ret;
}

static int __devexit iphone_pmu_remove(struct platform_device *dev)
{
	struct rtc_device *rtc = platform_get_drvdata(dev);
	platform_set_drvdata(dev, NULL);
	rtc_device_unregister(rtc);
	return 0;
}

static struct platform_driver iphone_pmu_driver = {
	.probe		= iphone_pmu_probe,
	.remove		= __devexit_p(iphone_pmu_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "iphone-pmu",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *iphone_pmu_device;

static int __init iphone_pmu_init(void)
{
	int ret;

	ret = platform_driver_register(&iphone_pmu_driver);
	if(!ret)
	{
		iphone_pmu_device = platform_device_register_simple("iphone-pmu", 0,
								NULL, 0);

		if (IS_ERR(iphone_pmu_device)) {
			platform_driver_unregister(&iphone_pmu_driver);
			ret = PTR_ERR(iphone_pmu_device);
		}
	}

	return ret;
}

static void __exit iphone_pmu_exit(void)
{
	platform_device_unregister(iphone_pmu_device);
	platform_driver_unregister(&iphone_pmu_driver);
}

module_init(iphone_pmu_init);
module_exit(iphone_pmu_exit);

MODULE_LICENSE("GPL");

