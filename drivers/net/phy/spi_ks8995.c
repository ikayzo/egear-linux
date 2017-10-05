/*
 * SPI driver for Micrel/Kendin KS8995M and KSZ8864RMN ethernet switches
 *
 * Copyright (C) 2008 Gabor Juhos <juhosg at openwrt.org>
 *
 * This file was based on: drivers/spi/at25.c
 *     Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <net/dsa.h>

#include <linux/spi/spi.h>

#define DRV_VERSION		"0.1.1"
#define DRV_DESC		"Micrel KS8995 Ethernet switch SPI driver"

/* ------------------------------------------------------------------------ */

#define KS8995_REG_ID0		0x00    /* Chip ID0 */
#define KS8995_REG_ID1		0x01    /* Chip ID1 */

#define KS8995_REG_GC0		0x02    /* Global Control 0 */
#define KS8995_REG_GC1		0x03    /* Global Control 1 */
#define KS8995_REG_GC2		0x04    /* Global Control 2 */
#define KS8995_REG_GC3		0x05    /* Global Control 3 */
#define KS8995_REG_GC4		0x06    /* Global Control 4 */
#define KS8995_REG_GC5		0x07    /* Global Control 5 */
#define KS8995_REG_GC6		0x08    /* Global Control 6 */
#define KS8995_REG_GC7		0x09    /* Global Control 7 */
#define KS8995_REG_GC8		0x0a    /* Global Control 8 */
#define KS8995_REG_GC9		0x0b    /* Global Control 9 */

#define KS8995_REG_PC(p, r)	((0x10 * (p + 1)) + r)		/* Port Control */
#define KS8995_REG_PS(p, r)	((0x10 * (p + 1)) + r + 0xe)	/* Port Status */

#define KS8995_REG_TPC0		0x60    /* TOS Priority Control 0 */
#define KS8995_REG_TPC1		0x61    /* TOS Priority Control 1 */
#define KS8995_REG_TPC2		0x62    /* TOS Priority Control 2 */
#define KS8995_REG_TPC3		0x63    /* TOS Priority Control 3 */
#define KS8995_REG_TPC4		0x64    /* TOS Priority Control 4 */
#define KS8995_REG_TPC5		0x65    /* TOS Priority Control 5 */
#define KS8995_REG_TPC6		0x66    /* TOS Priority Control 6 */
#define KS8995_REG_TPC7		0x67    /* TOS Priority Control 7 */

#define KS8995_REG_MAC(i)	(0x68 + i)	/* MAC address 0 */

#define KS8995_REG_IAC0		0x6e    /* Indirect Access Control 0 */
#define KS8995_REG_IAC1		0x6f    /* Indirect Access Control 0 */
#define KS8995_REG_IAD7		0x70    /* Indirect Access Data 7 */
#define KS8995_REG_IAD6		0x71    /* Indirect Access Data 6 */
#define KS8995_REG_IAD5		0x72    /* Indirect Access Data 5 */
#define KS8995_REG_IAD4		0x73    /* Indirect Access Data 4 */
#define KS8995_REG_IAD3		0x74    /* Indirect Access Data 3 */
#define KS8995_REG_IAD2		0x75    /* Indirect Access Data 2 */
#define KS8995_REG_IAD1		0x76    /* Indirect Access Data 1 */
#define KS8995_REG_IAD0		0x77    /* Indirect Access Data 0 */

#define KSZ8864_REG_ID1		0xfe	/* Chip ID in bit 7 */

#define KS8995_REGS_SIZE	0x80
#define KSZ8864_REGS_SIZE	0x100

#define ID1_CHIPID_M		0xf
#define ID1_CHIPID_S		4
#define ID1_REVISION_M		0x7
#define ID1_REVISION_S		1
#define ID1_START_SW		1	/* start the switch */

#define FAMILY_KS8995		0x95
#define CHIPID_M		0x04

#define KS8995_CMD_WRITE	0x02U
#define KS8995_CMD_READ		0x03U

#define KS8995_SOFT_RESET_DELAY	10  /* usec */
#define KS8995_HARD_RESET_DELAY	500 /* msec */
#define KS8995_RESET_PULSE_LEN	10  /* msec */

struct ks8995_pdata {
	int gpio_pwrdn;
	int gpio_reset;
};

struct ks8995_switch {
	struct spi_device	*spi;
	struct mutex		lock;
	struct ks8995_pdata	*pdata;
	struct bin_attribute	regs_attr;
};

static inline u8 get_chip_id(u8 val)
{
	return (val >> ID1_CHIPID_S) & ID1_CHIPID_M;
}

static inline u8 get_chip_rev(u8 val)
{
	return (val >> ID1_REVISION_S) & ID1_REVISION_M;
}

/* ------------------------------------------------------------------------ */

static int ks8995_read(struct ks8995_switch *ks, char *buf,
		 unsigned offset, size_t count)
{
	u8 cmd[2];
	struct spi_transfer t[2];
	struct spi_message m;
	int err;

	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t[0].tx_buf = cmd;
	t[0].len = sizeof(cmd);
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	cmd[0] = KS8995_CMD_READ;
	cmd[1] = offset;

	mutex_lock(&ks->lock);
	err = spi_sync(ks->spi, &m);
	mutex_unlock(&ks->lock);

	return err ? err : count;
}


static int ks8995_write(struct ks8995_switch *ks, char *buf,
		 unsigned offset, size_t count)
{
	u8 cmd[2];
	struct spi_transfer t[2];
	struct spi_message m;
	int err;

	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t[0].tx_buf = cmd;
	t[0].len = sizeof(cmd);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	cmd[0] = KS8995_CMD_WRITE;
	cmd[1] = offset;

	mutex_lock(&ks->lock);
	err = spi_sync(ks->spi, &m);
	mutex_unlock(&ks->lock);

	return err ? err : count;
}

static inline int ks8995_read_reg(struct ks8995_switch *ks, u8 addr, u8 *buf)
{
	return (ks8995_read(ks, buf, addr, 1) != 1) ? -EIO : 0;
}

static inline int ks8995_write_reg(struct ks8995_switch *ks, u8 addr, u8 val)
{
	char buf = val;

	return (ks8995_write(ks, &buf, addr, 1) != 1) ? -EIO : 0;
}

/* ------------------------------------------------------------------------ */

static inline int ks8995_have_pwrdn_gpio(struct ks8995_switch *ks)
{
	return gpio_is_valid(ks->pdata->gpio_pwrdn);
}

static inline int ks8995_have_reset_gpio(struct ks8995_switch *ks)
{
	return gpio_is_valid(ks->pdata->gpio_reset);
}

static void ks8995_power_off(struct ks8995_switch *ks)
{
	if (ks8995_have_pwrdn_gpio(ks)) {
		gpio_set_value_cansleep(ks->pdata->gpio_pwrdn, 0);
		msleep(KS8995_HARD_RESET_DELAY);
	}
}

static void ks8995_power_on(struct ks8995_switch *ks)
{
	if (ks8995_have_pwrdn_gpio(ks))
		gpio_set_value_cansleep(ks->pdata->gpio_pwrdn, 1);
}

static int ks8995_stop(struct ks8995_switch *ks)
{
	return ks8995_write_reg(ks, KS8995_REG_ID1, 0);
}

static int ks8995_start(struct ks8995_switch *ks)
{
	return ks8995_write_reg(ks, KS8995_REG_ID1, 1);
}

static int ks8995_reset(struct ks8995_switch *ks)
{
	int err;

	ks8995_power_on(ks);

	if (ks8995_have_reset_gpio(ks)) {
		gpio_set_value_cansleep(ks->pdata->gpio_reset, 0);
		msleep(KS8995_RESET_PULSE_LEN);
		gpio_set_value_cansleep(ks->pdata->gpio_reset, 1);
		msleep(KS8995_HARD_RESET_DELAY);
	} else {
		err = ks8995_stop(ks);
		if (err)
			return err;
		udelay(KS8995_SOFT_RESET_DELAY);
	}

	return ks8995_start(ks);
}

/* ------------------------------------------------------------------------ */

static ssize_t ks8995_registers_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct ks8995_switch *ks8995;

	dev = container_of(kobj, struct device, kobj);
	ks8995 = dev_get_drvdata(dev);

	if (unlikely(off > ks8995->regs_attr.size))
		return 0;

	if ((off + count) > ks8995->regs_attr.size)
		count = ks8995->regs_attr.size - off;

	if (unlikely(!count))
		return count;

	return ks8995_read(ks8995, buf, off, count);
}


static ssize_t ks8995_registers_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct ks8995_switch *ks8995;

	dev = container_of(kobj, struct device, kobj);
	ks8995 = dev_get_drvdata(dev);

	if (unlikely(off >= ks8995->regs_attr.size))
		return -EFBIG;

	if ((off + count) > ks8995->regs_attr.size)
		count = ks8995->regs_attr.size - off;

	if (unlikely(!count))
		return count;

	return ks8995_write(ks8995, buf, off, count);
}


static const struct bin_attribute ks8995_registers_attr = {
	.attr = {
		.name   = "registers",
		.mode   = S_IRUSR | S_IWUSR,
	},
	.size   = KS8995_REGS_SIZE,
	.read   = ks8995_registers_read,
	.write  = ks8995_registers_write,
};

static ssize_t ks8995_pwrdn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ks8995_switch *ks = dev_get_drvdata(dev);
	int pwrdn;

	if (!ks8995_have_pwrdn_gpio(ks))
		return -ENODEV;

	pwrdn = !gpio_get_value(ks->pdata->gpio_pwrdn);

	return sprintf(buf, "%d\n", pwrdn);
}

static ssize_t ks8995_pwrdn_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;
	struct ks8995_switch *ks = dev_get_drvdata(dev);

	if (!ks8995_have_pwrdn_gpio(ks))
		return -ENODEV;

	if (count >= 1 && buf[0] == '0')
		ks8995_power_on(ks);
	else if (count >= 1 && buf[0] == '1')
		ks8995_power_off(ks);
	else
		ret = -EINVAL;

	return ret;
}

static DEVICE_ATTR(pwrdn, 0644, ks8995_pwrdn_show, ks8995_pwrdn_store);

static ssize_t ks8995_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;
	struct ks8995_switch *ks = dev_get_drvdata(dev);

	if (!ks8995_have_reset_gpio(ks))
		return -ENODEV;

	if (count >= 1 && buf[0] == '1')
		ret = ks8995_reset(ks);
	else
		ret = -EINVAL;

	return ret;
}

static DEVICE_ATTR(reset, 0200, NULL, ks8995_reset_store);

static int ks8995_create_files(struct spi_device *spi)
{
	int ret;

	sysfs_attr_init(&dev_attr_pwrdn);
	sysfs_attr_init(&dev_attr_reset);
	sysfs_bin_attr_init(&ks8995_registers_attr);

	ret = device_create_file(&spi->dev, &dev_attr_pwrdn);
	if (ret) {
		dev_err(&spi->dev, "Unable to create sysfs 'pwrdn'\n");
		goto out;
	}

	ret = device_create_file(&spi->dev, &dev_attr_reset);
	if (ret) {
		dev_err(&spi->dev, "Unable to create sysfs 'reset'\n");
		goto out_remove_pwrdn;
	}

	ret = device_create_bin_file(&spi->dev, &ks8995_registers_attr);
	if (ret) {
		dev_err(&spi->dev, "Unable to create sysfs 'registers'\n");
		goto out_remove_reset;
	}

	return 0;

out_remove_reset:
	device_remove_file(&spi->dev, &dev_attr_reset);
out_remove_pwrdn:
	device_remove_file(&spi->dev, &dev_attr_pwrdn);
out:
	return ret;
}

static void ks8995_remove_files(struct spi_device *spi)
{
	device_remove_bin_file(&spi->dev, &ks8995_registers_attr);
	device_remove_file(&spi->dev, &dev_attr_reset);
	device_remove_file(&spi->dev, &dev_attr_pwrdn);
}

/* ------------------------------------------------------------------------ */

static void ks8995_request_gpio(int *gpio, struct device_node *np,
		char const *name, char const *label, int init_value)
{
	*gpio = of_get_named_gpio(np, name, 0);
	if (!gpio_is_valid(*gpio))
		goto out;

	if (gpio_request(*gpio, label) != 0);
		goto out;

	if (gpio_direction_output(*gpio, init_value) != 0);
		goto out_free_gpio;

	return;

out_free_gpio:
	gpio_free(*gpio);
out:
	*gpio = -1;
	return;
}

static struct ks8995_switch *ks;

static int ks8995_probe(struct spi_device *spi)
{
	struct ks8995_pdata     *pdata;
	struct device_node      *np = spi->dev.of_node;
	u8      ids[2];
	int     err;

	ks = devm_kzalloc(&spi->dev, sizeof(*ks), GFP_KERNEL);
	if (!ks)
		return -ENOMEM;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	spi->dev.platform_data = pdata;

	/* Obtain GPIOs for pwrdn/reset if they exist */
	ks8995_request_gpio(&pdata->gpio_pwrdn, np,
			"pwrdn-gpio", "ks8995 pwrdn", 0);
	ks8995_request_gpio(&pdata->gpio_reset, np,
			"reset-gpio", "ks8995 reset", 1);

	mutex_init(&ks->lock);
	ks->pdata = pdata;
	ks->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, ks);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "spi_setup failed, err=%d\n", err);
		return err;
	}

	ks8995_power_on(ks);

	err = ks8995_read(ks, ids, KS8995_REG_ID0, sizeof(ids));
	if (err < 0) {
		dev_err(&spi->dev, "unable to read id registers, err=%d\n",
				err);
		return err;
	}

	switch (ids[0]) {
	case FAMILY_KS8995:
		break;
	default:
		dev_err(&spi->dev, "unknown family id:%02x\n", ids[0]);
		return -ENODEV;
	}

	memcpy(&ks->regs_attr, &ks8995_registers_attr, sizeof(ks->regs_attr));
	if (get_chip_id(ids[1]) != CHIPID_M) {
		u8 val;

		/* Check if this is a KSZ8864RMN */
		err = ks8995_read(ks, &val, KSZ8864_REG_ID1, sizeof(val));
		if (err < 0) {
			dev_err(&spi->dev,
				"unable to read chip id register, err=%d\n",
				err);
			return err;
		}
		if ((val & 0x80) == 0) {
			dev_err(&spi->dev, "unknown chip:%02x,0\n", ids[1]);
			return err;
		}
		ks->regs_attr.size = KSZ8864_REGS_SIZE;
	}

	err = ks8995_reset(ks);
	if (err)
		return err;

	err = ks8995_create_files(spi);
	if (err) {
		dev_err(&spi->dev, "unable to create sysfs files, err=%d\n",
				    err);
		return err;
	}

	if (get_chip_id(ids[1]) == CHIPID_M) {
		dev_info(&spi->dev,
			 "KS8995 device found, Chip ID:%x, Revision:%x\n",
			 get_chip_id(ids[1]), get_chip_rev(ids[1]));
	} else {
		dev_info(&spi->dev,
			 "KSZ8864 device found, Chip ID:%x, Revision:%x\n",
			 get_chip_id(ids[1]), get_chip_rev(ids[1]));
	}

	return 0;
}

static int ks8995_remove(struct spi_device *spi)
{
       struct ks8995_switch *ks8995 = spi_get_drvdata(spi);

       ks8995_remove_files(spi);

       if (ks8995_have_pwrdn_gpio(ks8995))
               gpio_free(ks8995->pdata->gpio_pwrdn);
       if (ks8995_have_reset_gpio(ks8995))
               gpio_free(ks8995->pdata->gpio_reset);

       return 0;
}

/* ------------------------------------------------------------------------ */

static struct spi_driver ks8995_driver = {
	.driver = {
		.name	    = "spi-ks8995",
		.owner	   = THIS_MODULE,
	},
	.probe	  = ks8995_probe,
	.remove	  = ks8995_remove,
};

static int ks8995_dsa_setup(struct dsa_switch *ds)
{
	int i;
	int ret = -EIO;

	/*
	* TODO: (someday...) This should really use the data_switch
	* configuration data that is provided by the device tree
	* file, but we really just have a static use case, so I don't
	* see any immediate need to implement that (SCV).
	*/

	BUG_ON(!ks);

	// Forward packets to port based on port mask VLAN ID
	if (ks8995_write_reg(ks, KS8995_REG_GC3, 0x02))
		goto out;

	// Add ingress port VLAN ID to all packets on port 1
	if (ks8995_write_reg(ks, KS8995_REG_PC(0, 0), 0x04))
		goto out;

	// Set port VLAN membership for port 1 to all ports
	if (ks8995_write_reg(ks, KS8995_REG_PC(0, 1), 0x0f))
		goto out;

	// Set port VLAN ID for port 1 to 0
	if (ks8995_write_reg(ks, KS8995_REG_PC(0, 4), 0x00))
		goto out;

	for (i = 1; i <= 3; i++) {
		// port mask
		u8 mask = 0x01 << i;

		// Remove VLAN tag on packets egressing external ports
		if (ks8995_write_reg(ks, KS8995_REG_PC(i, 0), 0x02))
			goto out;

		// Set port VLAN membership for external ports
		if (ks8995_write_reg(ks, KS8995_REG_PC(i, 1), 0x01 | mask))
			goto out;

		// Set port VLAN ID for external ports to port mask
		if (ks8995_write_reg(ks, KS8995_REG_PC(i, 4), mask))
			goto out;
	}

	ret = 0;
out:
	return ret;

}

static int ks8995_set_addr(struct dsa_switch *ds, u8 *addr)
{
	int i;
	int ret = -EIO;

	for (i = 0; i < ETH_ALEN; i++) {
		if (ks8995_write_reg(ks, KS8995_REG_MAC(i), addr[i]))
			goto out;
	}

	ret = 0;
out:
	return ret;
}

static int ks8995_port_status(int port)
{
	int ret;
	u8 status = 0;

	ret = ks8995_read_reg(ks, KS8995_REG_PS(port, 0), &status);
	if (0 == ret) {
		ret = status;
	}

	return ret;
}

#define ANEG_FLAG(status) (!!(status & 0x40))
#define LINK_FLAG(status) (!!(status & 0x20))

static int ks8995_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	/* TODO: MDIO could be emulated over the SPI bus */
	int ret;
	switch (regnum) {
		case MII_PHYSID1:
			ret =  0;
			break;
		case MII_PHYSID2:
			ret = port;
			break;
		case MII_BMSR:
			ret = ks8995_port_status(port);
			if (ret >= 0) {
				/* see mii.h */
				ret = (0xfe08) |
					(ANEG_FLAG(ret) << 6) |
					(LINK_FLAG(ret) << 2);
			}
			break;
		default:
			ret = -EOPNOTSUPP;
			break;
	}

	return ret;
}

static int ks8995_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
       /* TODO: MDIO could be emulated over the SPI bus */
       return -EOPNOTSUPP;
}

static void ks8995_poll_link(struct dsa_switch *ds)
{
	int i;
	int status = 0;

	for (i = 0; i < DSA_MAX_PORTS; i++) {
		struct net_device *dev = ds->ports[i];

		if (NULL == dev) {
			continue;
		}

		if (dev->flags & IFF_UP) {
			status = ks8995_port_status(i);
			if (status < 0) {
				continue;
			}
			status = LINK_FLAG(status);
		}

		if (!status) {
			if (netif_carrier_ok(dev)) {
				netdev_info(dev, "link down\n");
				netif_carrier_off(dev);
			}
		}
		else {
			if (!netif_carrier_ok(dev)) {
				netdev_info(dev, "link up\n");
				netif_carrier_on(dev);
			}
		}
	}
}

static char *ks8995_dsa_probe(struct device *host_dev, int sw_addr)
{
       return "Micrel KS8995";
}

static struct dsa_switch_driver ks8995_dsa_driver = {
       .tag_protocol    = DSA_TAG_PROTO_VID,
       .probe           = ks8995_dsa_probe,
       .setup           = ks8995_dsa_setup,
       .set_addr        = ks8995_set_addr,
       .phy_read        = ks8995_phy_read,
       .phy_write       = ks8995_phy_write,
       .poll_link       = ks8995_poll_link,
};

static int __init ks8995_init(void)
{
	int ret = 0;

	ret = spi_register_driver(&ks8995_driver);
	if (!ret) {
		register_switch_driver(&ks8995_dsa_driver);
	}

	return ret;
}
module_init(ks8995_init);

static void __exit ks8995_exit(void)
{
	unregister_switch_driver(&ks8995_dsa_driver);
	spi_unregister_driver(&ks8995_driver);
}
module_exit(ks8995_exit);

MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Gabor Juhos <juhosg at openwrt.org>");
MODULE_LICENSE("GPL v2");
