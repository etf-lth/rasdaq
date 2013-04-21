/*
 * Radaq ADC driver
 *
 * Copyright (c) 2013 Fredrik Ahlberg
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>

#define PROTOCOL_VERSION 0x01

enum {
    PROTO_VERSION   = 0,
    PROTO_LED       = 1,
    PROTO_MODE      = 2,
    PROTO_STRB_WR   = 3,
    PROTO_STRB_RST  = 4,
    PROTO_POWER     = 5,
    PROTO_STANDBY   = 6,
    PROTO_RANGE     = 7,
    PROTO_STARTCNV  = 8,
    PROTO_REFEN     = 9,
    PROTO_STATUS    = 10, 
};

#define ERROR_INVALID_WRITE 0x01
#define ERROR_INVALID_READ  0x02

static int radaq_read_i2c(struct device *dev, uint8_t *data, uint8_t off)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &off,
        }, {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = data,
        }
    };

    if (i2c_transfer(client->adapter, msgs, 2) == 2)
        return 0;

    return -EIO;
}

static int radaq_write_i2c(struct device *dev, uint8_t data, uint8_t off)
{
    struct i2c_client *client = to_i2c_client(dev);
    uint8_t buffer[2] = {off, data};

    if (i2c_master_send(client, buffer, 2) == 2)
        return 0;

    return -EIO;
}

static int radaq_procfs_led_read(char *page, char **start,
    off_t off, int count, int *eof, void *data)
{
    struct device *dev = data;
    uint8_t reg;
    int len;

    radaq_read_i2c(dev, &reg, PROTO_LED);

    len = snprintf(page, count, "%02x\n", reg);

    return len;
}

static int radaq_procfs_led_write(struct file *file,
    const char *buffer, unsigned long count, void *data)
{
    struct device *dev = data;
    uint32_t reg;

    sscanf(buffer, "%02x\n", &reg);

    radaq_write_i2c(dev, reg, PROTO_LED);

    return count;
}

static int radaq_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    uint8_t reg;
    struct proc_dir_entry *ledfs;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    if (radaq_read_i2c(dev, &reg, PROTO_VERSION)) {
        dev_err(dev, "Unable to read protocol version\n");
        return -ENODEV;
    }
    
    dev_notice(dev, "Protocol version = %02x\n", reg);

    if (reg != PROTOCOL_VERSION) {
        dev_err(dev, "Version not supported\n");
        return -ENODEV;
    }

    // Create procfs entry for controlling the leds
    ledfs = create_proc_entry("radaq_led", 0644, NULL);
    if (ledfs) {
        ledfs->data = dev;
        ledfs->read_proc = radaq_procfs_led_read;
        ledfs->write_proc = radaq_procfs_led_write;
        //ledfs->owner = THIS_MODULE;
    }

    return 0;
}

static int __devexit radaq_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id radaq_id[] = {
    { "radaq", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, radaq_id);

static struct i2c_driver radaq_driver = {
    .driver = {
        .name   = "radaq",
        .owner  = THIS_MODULE,
    },
    .probe      = radaq_probe,
    .remove     = __devexit_p(radaq_remove),
    .id_table   = radaq_id
};

module_i2c_driver(radaq_driver);

MODULE_AUTHOR("Fredrik Ahlberg <fredrik@etf.nu>");
MODULE_DESCRIPTION("Radaq ADC driver");
MODULE_LICENSE("Proprietary");
