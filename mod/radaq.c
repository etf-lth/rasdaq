/*
 * Radaq ADC driver
 *
 * Copyright (c) 2013 Fredrik Ahlberg
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#define GPIOSET(x) (0x1c+(x)*4)
#define GPIOCLR(x) (0x28+(x)*4)
#define GPIOLEV(x) (0x34+(x)*4)

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

static struct gpio radaq_gpios[] = {
    { 3,    GPIOF_IN, "Radaq D0" },
    { 4,    GPIOF_IN, "Radaq D1" },
    { 14,   GPIOF_IN, "Radaq D2" },
    { 15,   GPIOF_IN, "Radaq D3" },
    { 17,   GPIOF_IN, "Radaq D4" },
    { 18,   GPIOF_IN, "Radaq D5" },
    { 27,   GPIOF_IN, "Radaq D6" },
    { 22,   GPIOF_IN, "Radaq D7" },
    { 23,   GPIOF_IN, "Radaq D8" },
    { 24,   GPIOF_IN, "Radaq D9" },
    { 10,   GPIOF_IN, "Radaq D10" },
    { 9,    GPIOF_IN, "Radaq D11" },
    { 25,   GPIOF_IN, "Radaq D12" },
    { 8,    GPIOF_IN, "Radaq D13" },
    { 11,   GPIOF_IN, "Radaq D14" },
    { 7,    GPIOF_IN, "Radaq D15" },
    { 30,   GPIOF_OUT_INIT_HIGH, "Radaq CS" },
    { 31,   GPIOF_OUT_INIT_HIGH, "Radaq RD" },
    { 2,    GPIOF_IN, "Radaq INT" },
};

struct radaq {
};

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

static int radaq_write_reg(struct device *dev, uint32_t reg)
{
    int idx;

    dev_notice(dev, "Writing ADC Config = %08x\n", reg);

    // set software mode
    radaq_write_i2c(dev, 0x01, PROTO_MODE);

    // set data bus to high word
    for (idx=0; idx<16; idx++) {
        gpio_direction_output(radaq_gpios[idx].gpio, reg & (0x10000 << idx));
    }

    // set /wr high
    radaq_write_i2c(dev, 0x01, PROTO_REFEN);

    // assert /cs
    gpio_set_value(30, 0);

    // stobe /wr
    radaq_write_i2c(dev, 0, PROTO_STRB_WR);
    
    // set data bus to low word
    for (idx=0; idx<16; idx++) {
        gpio_set_value(radaq_gpios[idx].gpio, reg & (1 << idx));
    }
    
    // stobe /wr
    radaq_write_i2c(dev, 0, PROTO_STRB_WR);
    
    // deassert /cs
    gpio_set_value(30, 1);

    // stop driving data bus
    for (idx=0; idx<16; idx++) {
        gpio_direction_input(radaq_gpios[idx].gpio);
    }
    
    // set hardware mode
    radaq_write_i2c(dev, 0x00, PROTO_MODE);

    return 0;
}

static void radaq_read_result(uint32_t *data)
{
    void __iomem *gpio = __io_address(GPIO_BASE);
    int idx, delay;

    // assert /cs
    writel(1 << 30, gpio + GPIOCLR(0));

    for (idx=0; idx<8; idx++) {
        // assert /rd
        writel(1 << 31, gpio + GPIOCLR(0));
        
        // dummy read
        for (delay=0; delay<10000; delay++) {
            data[idx] = readl(gpio + GPIOLEV(0));
        }

        // sample data bus
        data[idx] = readl(gpio + GPIOLEV(0));

        // deassert /rd
        writel(1 << 31, gpio + GPIOSET(0));

        // delay for good measure
        for (delay=0; delay<10000; delay++) {
            writel(1 << 31, gpio + GPIOSET(0));
        }
    }

    // deassert /cs
    writel(1 << 30, gpio + GPIOSET(0));
}

static inline int16_t radaq_unswizzle(uint32_t reg)
{
    int idx;
    uint16_t res = 0;

    for (idx=0; idx<16; idx++) {
        if (reg & (1 << radaq_gpios[idx].gpio)) {
            res |= 1 << idx;
        }
    }

    return res;
}

static irqreturn_t radaq_isr(int irq, void *data)
{
    uint32_t result[8], idx;

    radaq_read_result(result);

    printk(KERN_NOTICE "%s: we've got irq %d\n", __FUNCTION__, irq);

    for (idx=0; idx<8; idx++) {
        printk(KERN_NOTICE "%s: result %d = %d\n",
                __FUNCTION__, idx, radaq_unswizzle(result[idx]));
    }

    return IRQ_HANDLED;
}

static int radaq_procfs_test_read(char *page, char **start,
    off_t off, int count, int *eof, void *data)
{
    struct device *dev = data;
    int len;

    len = snprintf(page, count, "%02x\n", 0);

    return len;
}

static int radaq_procfs_test_write(struct file *file,
    const char *buffer, unsigned long count, void *data)
{
    struct device *dev = data;

    radaq_write_i2c(dev, 0x01, PROTO_POWER);

    mdelay(100);

    radaq_write_i2c(dev, 0, PROTO_STRB_RST);
    radaq_write_i2c(dev, 0x00, PROTO_STANDBY);
    radaq_write_reg(dev, (1<<31)|(1<<27)|(1<<13)|0x000003ff);
    radaq_write_i2c(dev, 0x01, PROTO_REFEN);

    mdelay(100);

    radaq_write_i2c(dev, 0x00, PROTO_STARTCNV);

    return count;
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
    struct radaq *rdq;
    struct proc_dir_entry *ledfs, *testfs;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    // Try communicate with the MCU and check protocol version
    if (radaq_read_i2c(dev, &reg, PROTO_VERSION)) {
        dev_err(dev, "Unable to read protocol version\n");
        return -ENODEV;
    }
    
    dev_notice(dev, "Protocol version = %02x\n", reg);

    if (reg != PROTOCOL_VERSION) {
        dev_err(dev, "Version not supported\n");
        return -ENODEV;
    }

    // Register GPIO pins to detect conflicts
    if (gpio_request_array(radaq_gpios, ARRAY_SIZE(radaq_gpios))) {
        dev_err(dev, "Unable to claim GPIOs\n");
        return -ENODEV;
    }

    // Allocate a radaq struct
    rdq = kmalloc(sizeof(struct radaq), GFP_KERNEL);
    
    // Register GPIO interrupt
    if (request_irq(gpio_to_irq(2), radaq_isr, IRQ_TYPE_EDGE_RISING, "Radaq", rdq)) {
        dev_err(dev, "Unable to register GPIO interrupt\n");

        gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));
        kfree(rdq);

        return -ENODEV;
    }

    // Create procfs entry for controlling the leds
    ledfs = create_proc_entry("radaq_led", 0644, NULL);
    if (ledfs) {
        ledfs->data = dev;
        ledfs->read_proc = radaq_procfs_led_read;
        ledfs->write_proc = radaq_procfs_led_write;
    } else {
        dev_warn(dev, "Unable to register procfs entry\n");
    }
    
    // Create procfs entry for testing
    testfs = create_proc_entry("radaq_test", 0644, NULL);
    if (testfs) {
        testfs->data = dev;
        testfs->read_proc = radaq_procfs_test_read;
        testfs->write_proc = radaq_procfs_test_write;
    } else {
        dev_warn(dev, "Unable to register procfs entry\n");
    }

    // Save out handle
    i2c_set_clientdata(client, rdq);

    return 0;
}

static int __devexit radaq_remove(struct i2c_client *client)
{
    struct radaq *rdq = i2c_get_clientdata(client);

    // Remove procfs entry
    remove_proc_entry("radaq_led", NULL);
    remove_proc_entry("radaq_test", NULL);

    // Release GPIOs
    gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));

    // Free out struct
    kfree(rdq);

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
MODULE_LICENSE("GPL");
