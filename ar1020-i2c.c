/*
 * Microchip I2C ar102[0-1]Touchscreen Driver
 *
 * Copyright (c) 2015 Robert Boissel CDSSoft.
 * Copyright (c) 2011 Steve Grahovac Microchip Technology, Inc.
 *
 * Based on a previous work by Copyright (C) 2011 Microchip Technology, Inc.
 * Steve Grahovac <steve.grahovac@microchip.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * http://www.microchip.com/
 * datasheet DS41393b.
 */


#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define MODULE_NAME "AR1020_I2C"

#define ts_error(fmt, arg...) printk(KERN_ERR  MODULE_NAME " %s:%s: " fmt "\n", strrchr(__FILE__, '/'), __func__, ## arg)
#define ts_notice(fmt, arg...) printk(KERN_NOTICE  MODULE_NAME " %s:%s: " fmt "\n", strrchr(__FILE__, '/'), __func__, ## arg)
#define ts_debug(fmt, arg...) printk(KERN_DEBUG  MODULE_NAME " %s:%s: " fmt "\n", strrchr(__FILE__, '/'), __func__, ## arg)

#define AR1020_I2C_HEADER	0x55
#define AR1020_I2C_TRP_SZ	5	/* The maximum packet byte length for Touch Reporting Protocole */
#define AR1020_I2C_CP_SZ	14	/* The maximum packet byte length for Command Protocole */

#define AR1020_X_RAW_MAX	4095	/* Max value sent by controler for X */
#define AR1020_Y_RAW_MAX	4095	/* Max value sent by controler for Y */

/* The private data structure that is referenced within the I2C bus driver */
struct ar1020_i2c_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
};

enum command_status {
	SUCCES = 0,
	UNRECONIZED_CMD = 1,
	UNRECONIZED_HEADER = 3,
	TIMOUT = 4,
	CALIBRATE_CANCEL = 0xfc,
};

enum command_set_id {
	GET_VERSION,
	ENABLE_TOUCH,
	DISABLE_TOUCH,
	CALIBRATE_MODE,
	REGISTER_READ,
	REGISTER_WRITE,
	REGISTER_START_ADDRESS_REQUEST,
	REGISTERS_WRITE_TO_EEPROM,
	EEPROM_READ,
	EEPROM_WRITE,
	EEPROM_WRITE_TO_REGISTERS,
};

struct command_set {
	int value;	// command value
	int req_size;	// request size, if > 100 the size variable (req_size - 100) equal minimal siz
	int ans_size;	// answer size? idem req_size
};
/*
0x10	// GET_VERSION
0x12	// ENABLE_TOUCH
0x13	// DISABLE_TOUCH
0x14	// CALIBRATE_MODE
0x20	//  REGISTER_READ
0x21	//  REGISTER_WRITE
0x22	//  REGISTER_START_ADDRESS_REQUEST
0x23	//  REGISTERS_WRITE_TO_EEPROM
0x28	//  EEPROM_READ
0x29	//  EEPROM_WRITE
0x2B	//  EEPROM_WRITE_TO_REGISTERS
*/

/* These are all the sysfs variables used to store and retrieve information
   from a user-level application */
static char sendBuffer[100];
static char receiveBuffer[100];
static int commandMode=0;
static int commandDataPending=0;
static int minX=0;
static int maxX=AR1020_X_RAW_MAX;
static int minY=0;
static int maxY=AR1020_Y_RAW_MAX;
static int swapAxes=0;
static int invertX=0;
static int invertY=0;
static int lastPUCoordX=0;
static int lastPUCoordY=0;


/* Since the reference to private data is stored within the I2C
   bus driver, we will store another reference within this driver
   so the sysfs related function may also access this data */
struct ar1020_i2c_priv *privRef=NULL;

/******************************************************************************
Function:
	commandDataPending_show()

Description:
	Display value of "commandDataPending" variable to application that is
	requesting it's value.
******************************************************************************/
static ssize_t commandDataPending_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", commandDataPending);
}

/******************************************************************************
Function:
	commandDataPending_store()

Description:
	Save value to "commandDataPending" variable from application that is
	requesting this.
******************************************************************************/
static ssize_t commandDataPending_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d", &commandDataPending);
	return count;
}

static struct kobj_attribute commandDataPending_attribute =
	__ATTR(commandDataPending, 0666, commandDataPending_show, commandDataPending_store);


/******************************************************************************
Function:
	commandMode_show()

Description:
	Display value of "commandMode" variable to application that is
	requesting it's value.
******************************************************************************/
static ssize_t commandMode_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", commandMode);
}

/******************************************************************************
Function:
	commandMode_store()

Description:
	Save value to "commandMode" variable from application that is
	requesting this.
******************************************************************************/
static ssize_t commandMode_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d", &commandMode);
	return count;

}

static struct kobj_attribute commandMode_attribute =
	__ATTR(commandMode, 0666, commandMode_show, commandMode_store);

/******************************************************************************
Function:
	receiveBuffer_show()

Description:
	Display value of "receiveBuffer" variable to application that is
	requesting it's value.
******************************************************************************/
static ssize_t receiveBuffer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	/* since we have now read the receiveBuffer, receive data is no longer pending */
	commandDataPending=0;
	return sprintf(buf, "%s", receiveBuffer);
}

/******************************************************************************
Function:
	receiveBuffer_store()

Description:
	Save value to "receiveBuffer" variable from application that is
	requesting this.
******************************************************************************/
static ssize_t receiveBuffer_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	return snprintf(receiveBuffer,sizeof(receiveBuffer),"%s",buf);
}

static struct kobj_attribute receiveBuffer_attribute =
	__ATTR(receiveBuffer, 0666, receiveBuffer_show, receiveBuffer_store);

/******************************************************************************
Function:
	sendBuffer_show()

Description:
	Display value of "sendBuffer" variable to application that is
	requesting it's value.
******************************************************************************/
static ssize_t sendBuffer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%s", sendBuffer);
}

/******************************************************************************
Function:
	sendBuffer_store()

Description:
	Save value to "sendBuffer" variable from application that is
	requesting this.
******************************************************************************/
static ssize_t sendBuffer_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int commandByte[8];
	char buff[8];
	int numCommandBytes;
	int idx, ret;

	memset(sendBuffer,0, sizeof(sendBuffer));
	commandDataPending=0;

	/* disallow commands to be sent until command mode is enabled */
	if (0==commandMode) {
		ts_notice("Warning: command bytes will be ignored until commandMode is enabled");
		return count;
	}

	numCommandBytes=sscanf(buf,"%x %x %x %x %x %x %x %x",&commandByte[0],&commandByte[1],&commandByte[2],
&commandByte[3],&commandByte[4],&commandByte[5],&commandByte[6],&commandByte[7]);


	ts_debug("Processed %d bytes.",numCommandBytes);

	/* Verify command string to send to controller is valid */
	if (numCommandBytes<3) {
		ts_notice("Insufficient command bytes to process.");
	} else if (commandByte[0]!=0x0) {
		ts_notice("Leading zero required when sending I2C commands.");
	} else if (commandByte[1]!=0x55) {
		ts_notice("Invalid header byte (0x55 expected).");
	} else if (commandByte[2] != (numCommandBytes-3)) {
		ts_notice("Number of command bytes specified not valid for current string.");
	}

	ts_debug("sending command bytes: >>>");
	for (idx=0; idx<numCommandBytes; idx++) {
		buff[idx]=(char)commandByte[idx];
		printk(KERN_DEBUG "0x%02x ", commandByte[idx]);
	}
	printk(KERN_DEBUG "\n");
	ts_debug("sending command bytes: <<<");

	ret = i2c_master_send (privRef->client,buff,numCommandBytes);
	if (ret < 0)
		ts_debug("i2c_master_send failed with %d", ret);
	return snprintf(sendBuffer,sizeof(sendBuffer),"%s",buf);
}

static struct kobj_attribute sendBuffer_attribute =
	__ATTR(sendBuffer, 0666, sendBuffer_show, sendBuffer_store);

/******************************************************************************
Function:
	calibrationSettings_show()

Description:
	Display value of "calibrationSettings" variable to application that is
	requesting it's value.	The handling of the calibration settings has
	been grouped together.
******************************************************************************/
static ssize_t calibrationSettings_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int calibrationSetting=0;

	if (strcmp(attr->attr.name, "minX") == 0)
		calibrationSetting = minX;
	else if (strcmp(attr->attr.name, "maxX") == 0)
		calibrationSetting = maxX;
	else if (strcmp(attr->attr.name, "minY") == 0)
		calibrationSetting = minY;
	else if (strcmp(attr->attr.name, "maxY") == 0)
		calibrationSetting = maxY;
	else if (strcmp(attr->attr.name, "swapAxes") == 0)
		calibrationSetting = swapAxes;
	else if (strcmp(attr->attr.name, "invertX") == 0)
		calibrationSetting = invertX;
	else if (strcmp(attr->attr.name, "invertY") == 0)
		calibrationSetting = invertY;
	else if (strcmp(attr->attr.name, "lastPUCoordX") == 0)
		calibrationSetting = lastPUCoordX;
	else if (strcmp(attr->attr.name, "lastPUCoordY") == 0)
		calibrationSetting = lastPUCoordY;

	return sprintf(buf, "%d\n", calibrationSetting);
}

/******************************************************************************
Function:
	calibrationSettings_store()

Description:
	Save calibration setting to corresponding variable from application
	that is requesting this.
******************************************************************************/
static ssize_t calibrationSettings_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int calibrationSetting;

	sscanf(buf, "%d", &calibrationSetting);

	if (strcmp(attr->attr.name, "minX") == 0)
		minX = calibrationSetting;
	else if (strcmp(attr->attr.name, "maxX") == 0)
		maxX = calibrationSetting;
	else if (strcmp(attr->attr.name, "minY") == 0)
		minY = calibrationSetting;
	else if (strcmp(attr->attr.name, "maxY") == 0)
		maxY = calibrationSetting;
	else if (strcmp(attr->attr.name, "swapAxes") == 0)
		swapAxes = calibrationSetting;
	else if (strcmp(attr->attr.name, "invertX") == 0)
		invertX = calibrationSetting;
	else if (strcmp(attr->attr.name, "invertY") == 0)
		invertY = calibrationSetting;
	else if (strcmp(attr->attr.name, "lastPUCoordX") == 0)
		lastPUCoordX = calibrationSetting;
	else if (strcmp(attr->attr.name, "lastPUCoordY") == 0)
		lastPUCoordY = calibrationSetting;

	return count;
}

/* Defines sysfs variable associations */
static struct kobj_attribute minX_attribute =
	__ATTR(minX, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute maxX_attribute =
	__ATTR(maxX, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute minY_attribute =
	__ATTR(minY, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute maxY_attribute =
	__ATTR(maxY, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute swapAxes_attribute =
	__ATTR(swapAxes, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute invertX_attribute =
	__ATTR(invertX, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute invertY_attribute =
	__ATTR(invertY, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute lastPUCoordX_attribute =
	__ATTR(lastPUCoordX, 0666, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute lastPUCoordY_attribute =
	__ATTR(lastPUCoordY, 0666, calibrationSettings_show, calibrationSettings_store);

/*
 * Create a group of calibration attributes so we may work with them
 * as a set.
 */
static struct attribute *attrs[] = {
	&commandDataPending_attribute.attr,
	&commandMode_attribute.attr,
	&receiveBuffer_attribute.attr,
	&sendBuffer_attribute.attr,
	&minX_attribute.attr,
	&maxX_attribute.attr,
	&minY_attribute.attr,
	&maxY_attribute.attr,
	&swapAxes_attribute.attr,
	&invertX_attribute.attr,
	&invertY_attribute.attr,
	&lastPUCoordX_attribute.attr,
	&lastPUCoordY_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *ar1020_kobj;

static irqreturn_t touch_irq(int irq, void *dev_id);

/******************************************************************************
Function:
	decodeAR1020Packet()

Description:
	Decode Packets of Touch Report Protocole data from a device path using
	AR1XXX protocol (touch corrdinates)
******************************************************************************/
void decodeAR1020PacketTRP(struct ar1020_i2c_priv *priv, char *packet, int size)
{
	int x_raw, y_raw; /* raw coord */
	int button;
	int idx;
	int x_adj, y_adj; /* adjusted coord */

	/****************************************************
	Data format, 5 bytes: PEN, DATA1, DATA2, DATA3, DATA4

	SYNC [7:0]: 1,0,0,0,0,TOUCHSTATUS[0:0]
	DATA1[7:0]: 0,X-LOW[6:0]
	DATA2[7:0]: 0,X-HIGH[4:0]
	DATA3[7:0]: 0,Y-LOW[6:0]
	DATA4[7:0]: 0,Y-HIGH[4:0]

	TOUCHSTATUS: 0 = Touch up, 1 = Touch down
	****************************************************/

	if (size < AR1020_I2C_TRP_SZ) {
		ts_debug("not enough data %d", size);
		return;
	}
	if (!(packet[0] & 0x80)) {
		ts_debug("Sync bit not set !");
		return;
	}
	for (idx = 1; idx < AR1020_I2C_TRP_SZ; idx++)
		if ((packet[idx] & 0x80)) {
			ts_debug("byte %d not valid", idx);
			return;
		}

	x_raw = ((packet[2] & 0x1f) << 7) | (packet[1] & 0x7f);
	y_raw = ((packet[4] & 0x1f) << 7) | (packet[3] & 0x7f);
	button = (packet[0] & 1);

	if (!button) {
		lastPUCoordX = x_raw;
		lastPUCoordY = y_raw;
	}

	if (swapAxes) {
		int temp = x_raw;
		x_raw = y_raw;
		y_raw = temp;
	}

	if (invertX)
		x_raw = AR1020_X_RAW_MAX - x_raw;

	if (invertY)
		y_raw = AR1020_Y_RAW_MAX - y_raw;

	if (x_raw < minX)
		x_adj = 0;
	else if (x_raw > maxX)
		x_adj = AR1020_X_RAW_MAX;
	else
		/* percentage across calibration area times the maximum controller width */
		x_adj = ((x_raw - minX) * AR1020_X_RAW_MAX) / (maxX - minX);

	if (y_raw < minY)
		y_adj = 0;
	else if (y_raw > maxY)
		y_adj = AR1020_Y_RAW_MAX;
	else
		/* percentage across calibration area times the maximum controller height */
		y_adj = ((y_raw - minY) * AR1020_Y_RAW_MAX) / (maxY - minY);

	input_report_abs(priv->input, ABS_X, x_adj);
	input_report_abs(priv->input, ABS_Y, y_adj);
	input_report_key(priv->input, BTN_TOUCH, button);
	input_sync(priv->input);
}

/******************************************************************************
Function:
	decodeAR1020PacketCP()

Description:
	Decode Packets of Command Protocole data from a device path using
	AR1XXX protocol (response following a command).
******************************************************************************/
void decodeAR1020PacketCP(struct ar1020_i2c_priv *priv, char *packet, int size)
{
	int idx;
	int buffer_rem = sizeof(receiveBuffer);
	int buffer_idx = 0;
	int ret;
	commandDataPending=1;

	for (idx = 0; idx < size; idx++)
	{
		ret = snprintf(&receiveBuffer[buffer_idx], buffer_rem, "0x%02x ", 0xff & packet[idx]);
		if (ret < 0) {
			ts_error("snprintf failed");
			return;
		} else if (ret == buffer_rem) {
			ts_error("snprintf troncated");
			return;
		}
		buffer_idx += ret;
		buffer_rem -= ret;
	}

	if (packet[0] != 0x55)
		ts_error("invalid header byte");
	if (packet[1] > AR1020_I2C_CP_SZ)
		ts_error("invalid byte count");

	snprintf(&receiveBuffer[buffer_idx], buffer_rem, "\n");
	ts_debug("command response: %s",receiveBuffer);
	return;
}

/******************************************************************************
Function:
	ar1020_i2c_readdata()

Description:
	When the controller interrupt is asserted, this function is scheduled
	to be called to read the controller data within the
	touch_irq() function.
******************************************************************************/
static void ar1020_i2c_readdata(struct work_struct *work)
{
	struct ar1020_i2c_priv *priv = container_of(work, struct ar1020_i2c_priv, work);
	char buff[AR1020_I2C_CP_SZ];
	int ret;

	ret = i2c_master_recv(priv->client, buff, sizeof(buff));
	if (ret < 0) {
		ts_error("i2c_master_recv failed with %d", ret);
		return;
	}

	/* We want to ensure we only read packets when we are not in the middle of command communication.
	   Disable command mode after receiving command response to resume receiving packets. */
	if (commandMode)
		decodeAR1020PacketCP(priv, buff, ret);
	else
		decodeAR1020PacketTRP(priv, buff, ret);
}

/******************************************************************************
Function:
	ar1020_i2c_probe()

Description:
	After the kernel's platform specific source files have been modified to
	reference the "ar1020_i2c" driver, this function will then be called.
	This function needs to be called to finish registering the driver.
******************************************************************************/
static int ar1020_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ar1020_i2c_priv *priv=NULL;
	struct input_dev *input_dev=NULL;
	int err=0;

	ts_debug("begin");

	if (!client) {
		ts_error("client pointer is NULL");
		err = -EINVAL;
		goto error;
	}

	if (!client->irq) {
		ts_error("no IRQ set for touch controller");
		err = -EINVAL;
		goto error;
	}

	priv = kzalloc(sizeof(struct ar1020_i2c_priv), GFP_KERNEL);
	if (!priv) {
		ts_error("kzalloc error");
		err = -ENOMEM;
		goto error;
	}

	/* Backup pointer so sysfs helper functions may also have access to private data */
	privRef=priv;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_error("input allocate error");
		err = -ENOMEM;
		goto error;
	}

	priv->client = client;
	priv->input = input_dev;

	ts_debug("Using IRQ %d set via board's platform setting.", client->irq);

	INIT_WORK(&priv->work, ar1020_i2c_readdata);

	input_dev->name = "AR1020 Touchscreen";
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, AR1020_X_RAW_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, AR1020_Y_RAW_MAX, 0, 0);
	err = input_register_device(input_dev);
	if (err) {
		ts_error("error registering input device");
		goto error;
	}

	/* set type and register gpio pin as our interrupt */
	irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_RISING);
	request_irq(client->irq, touch_irq, 0, "AR1020 I2C IRQ", priv);

	i2c_set_clientdata(client, priv);

	return 0;

 error:

	if (input_dev)
		input_free_device(input_dev);

	if (priv)
		kfree(priv);

	return err;
}

/******************************************************************************
Function:
	ar1020_i2c_remove()

Description:
	Unregister/remove the kernel driver from memory.
******************************************************************************/
static int ar1020_i2c_remove(struct i2c_client *client)
{
	struct ar1020_i2c_priv *priv = (struct ar1020_i2c_priv *)i2c_get_clientdata(client);

	free_irq(client->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

/* This structure describe a list of supported slave chips */
static const struct i2c_device_id ar1020_i2c_id[] = {
	{ "ar1020_i2c", 0 },
	{ }
};

/******************************************************************************
Function:
	touch_irq()

Description:
	After the interrupt is asserted for the controller, this
	is the first function that is called.  Since this is a time sensitive
	function, we need to immediately schedule work so the integrity of
	properly system operation

	This function needs to be called to finish registering the driver.
******************************************************************************/
static irqreturn_t touch_irq(int irq, void *dev_id)
{
	struct ar1020_i2c_priv *priv = (struct ar1020_i2c_priv *)dev_id;
	int err;

	if (!priv) {
		ts_error("no private data");
		err = -EINVAL;
		return err;
	}

	 /* delegate I2C transactions since hardware interupts need to be handled very fast */
	schedule_work(&priv->work);

	return IRQ_HANDLED;
}

static const struct of_device_id ar1020_dt_ids[] = {
	{ .compatible = "microchip,ar1020_ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar1020_dt_ids);


/* This is the initial set of information information the kernel has
   before probing drivers on the system, */
static struct i2c_driver ar1020_i2c_driver = {
	.driver = {
		.name	= "ar1020_i2c",
		.of_match_table = of_match_ptr(ar1020_dt_ids),
	},
	.probe		= ar1020_i2c_probe,
	.remove		= ar1020_i2c_remove,
	/* suspend/resume functions not needed since controller automatically
	   put's itself to sleep mode after configurable short period of time */
	.suspend	= NULL,
	.resume		= NULL,
	.id_table	= ar1020_i2c_id,
};

/******************************************************************************
Function:
	ar1020_i2c_init()

Description:
	This function is called during startup even if the platform specific
	files have not been setup yet.
******************************************************************************/
static int __init ar1020_i2c_init(void)
{
	int retval;
	ts_debug("begin");
	memset(receiveBuffer,0, sizeof(receiveBuffer));
	memset(sendBuffer,0, sizeof(sendBuffer));

	/*
	 * Creates a kobject "ar1020" that appears as a sub-directory
	 * under "/sys/kernel".
	 */
	ar1020_kobj = kobject_create_and_add("ar1020", kernel_kobj);
	if (!ar1020_kobj) {
		ts_error("cannot create kobject");
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(ar1020_kobj, &attr_group);
	if (retval) {
		ts_error("error registering ar1020-i2c driver's sysfs interface");
		kobject_put(ar1020_kobj);
	}

	retval = i2c_add_driver(&ar1020_i2c_driver);
	ts_debug("i2c_add_driver return %d", retval);
	return retval;
}

/******************************************************************************
Function:
	ar1020_i2c_exit()

Description:
	This function is called after ar1020_i2c_remove() immediately before
	being removed from the kernel.
******************************************************************************/
static void __exit ar1020_i2c_exit(void)
{
	ts_debug("begin");
	kobject_put(ar1020_kobj);
	i2c_del_driver(&ar1020_i2c_driver);
}

MODULE_AUTHOR("Robert Boissel <r.boissel@studiel.fr>");
MODULE_DESCRIPTION("AR1020 touchscreen I2C bus driver");
MODULE_LICENSE("GPL");

/* Enable the ar1020_i2c_init() to be run by the kernel during initialization */
module_init(ar1020_i2c_init);

/* Enables the ar1020_i2c_exit() to be called during cleanup.  This only
has an effect if the driver is compiled as a kernel module. */
module_exit(ar1020_i2c_exit);

