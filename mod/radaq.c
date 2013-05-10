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
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

/*
 * IOCTLs for userspace interface
 */
enum {
    RADAQ_IOCTL_SAMPLERATE      = 0xcafe0000,
    RADAQ_IOCTL_CHANNELS        = 0xcafe0001,
    RADAQ_IOCTL_BUFFER_SIZE     = 0xcafe0002,
    RADAQ_IOCTL_ARM             = 0xcafe0003,
};

/*
 * Max samplerate
 */
#define SMPRATE_MAX 500000

#define BUFSIZE 1024
#define CHANNELS 8

/*
 * BCM2835 GPIO helpers
 */
#define GPIOSET(x) (0x1c+(x)*4)
#define GPIOCLR(x) (0x28+(x)*4)
#define GPIOLEV(x) (0x34+(x)*4)

/*
 * Radaq MCU protocol
 */
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
    PROTO_BURST     = 11,
    PROTO_RUN       = 12,
};

#define ERROR_INVALID_WRITE 0x01
#define ERROR_INVALID_READ  0x02

/*
 * GPIOs we'll use
 */
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

/*
 * Some static data
 */
static dev_t devno;
static struct class *class;

struct radaq {
    struct cdev *cdev;
    struct device *i2cdev, *ctrldev;

    int16_t buffer[2][BUFSIZE*CHANNELS];
    size_t hpage, tpage, idx;
    struct semaphore sem;
    int channels;
    size_t bufsz;
    uint8_t leds;
    int running, overrun;
} rdq;

/*
 * Read a register on the MCU
 */
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

/*
 * Write a register to the MCU
 */
static int radaq_write_i2c(struct device *dev, uint8_t data, uint8_t off)
{
    struct i2c_client *client = to_i2c_client(dev);
    uint8_t buffer[2] = {off, data};

    if (i2c_master_send(client, buffer, 2) == 2)
        return 0;

    return -EIO;
}

static void radaq_set_led(int led, int state)
{
    if (state) {
        rdq.leds |= (1 << led);
    } else {
        rdq.leds &= ~(1 << led);
    }

    radaq_write_i2c(rdq.i2cdev, rdq.leds, PROTO_LED);
}

/*
 * Write the ADC config register
 */
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

/*
 * Read the sampled data from the ADC
 */
static inline void radaq_read_result(uint32_t *data, int channels)
{
    void __iomem *gpio = __io_address(GPIO_BASE);
    int delay;

    // assert /cs
    writel(1 << 30, gpio + GPIOCLR(0));

    while (channels--) {
        // assert /rd
        writel(1 << 31, gpio + GPIOCLR(0));
        
        // dummy read
        for (delay=0; delay<1000; delay++) {
            *data = readl(gpio + GPIOLEV(0));
        }

        // sample data bus
        *data++ = readl(gpio + GPIOLEV(0));

        // deassert /rd
        writel(1 << 31, gpio + GPIOSET(0));

        // delay for good measure
        for (delay=0; delay<1000; delay++) {
            writel(1 << 31, gpio + GPIOSET(0));
        }
    }

    // deassert /cs
    writel(1 << 30, gpio + GPIOSET(0));
}

/*
 * Unswizzle the bits to account for the GPIO to data bus mapping
 */
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

/*
 * Conversion Complete Interrupt Service Routine (tm)
 */
static irqreturn_t radaq_isr(int irq, void *data)
{
    uint32_t result[CHANNELS], idx;
    //struct radaq *rdq = data;

    if (!rdq.running) {
        printk("isr: not running!\n");
        return IRQ_HANDLED;
    }

    radaq_read_result(result, rdq.channels);

    //printk(KERN_NOTICE "%s: we've got irq %d\n", __FUNCTION__, irq);

    /*if (rdq.idx >= 1024) {
        printk("kaos\n");
        return IRQ_HANDLED;
    }*/

    for (idx=0; idx<rdq.channels; idx++) {
        //struct radaq *rdq = data;
        rdq.buffer[rdq.hpage][rdq.idx++] = radaq_unswizzle(result[idx]);
        /*printk(KERN_NOTICE "%s: result %d = %d\n",
                __FUNCTION__, idx, radaq_unswizzle(result[idx]));*/
        if (rdq.idx == BUFSIZE) {
            rdq.idx = 0;
            rdq.hpage = !rdq.hpage;

            //printk("isr: flippin' pages!\n");

            if (rdq.hpage == rdq.tpage) {
                rdq.overrun = 1;
                rdq.running = 0;
                printk("isr: buffer overrun!\n");
            }

            up(&rdq.sem);

            //printk("isr: done.\n");
        //} else {
            //radaq_write_i2c(rdq->dev, 0x00, PROTO_STARTCNV);
            //printk("isr: idx=%d, trigging again\n", rdq->idx);
        }
    }

    return IRQ_HANDLED;
}

#if 0
static int radaq_procfs_test_read(char *page, char **start,
    off_t off, int count, int *eof, void *data)
{
    struct device *dev = data;
    struct radaq *rdq = i2c_get_clientdata(to_i2c_client(dev));
    int len;

    len = snprintf(page, count, "%d %d %d %d %d %d %d %d\n",
            rdq->buffer[rdq->idx][0], rdq->buffer[rdq->idx][1],
            rdq->buffer[rdq->idx][2], rdq->buffer[rdq->idx][3],
            rdq->buffer[rdq->idx][4], rdq->buffer[rdq->idx][5],
            rdq->buffer[rdq->idx][6], rdq->buffer[rdq->idx][7]);
    //rdq->idx++;

    if (rdq->idx == 1024) {
        printk("reached end of file.\n");
    } else {
        printk("now idx %d\n", rdq->idx);
    }

    return len;
}

static int radaq_procfs_test_write(struct file *file,
    const char *buffer, unsigned long count, void *data)
{
    struct device *dev = data;
    struct radaq *rdq = i2c_get_clientdata(to_i2c_client(dev));

    uint32_t reg;

    sscanf(buffer, "%02x\n", &reg);

    switch (reg) {
        case 1:

            rdq->idx = 0;

            radaq_write_i2c(dev, 0x00, PROTO_BURST);

            printk("waiting for semaphore...\n");
            down_interruptible(&rdq->sem);
            printk("semaphore held.\n");

            rdq->idx = 0;
            break;

        case 2:
            rdq->idx++;
    }

    return count;
}
#endif

#if 0
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
#endif

static void radaq_initialize(void)
{
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_POWER);

    mdelay(100);

    radaq_write_i2c(rdq.i2cdev, 0, PROTO_STRB_RST);
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_STANDBY);
    radaq_write_reg(rdq.i2cdev, (1<<31)|(1<<27)|(1<<13)|0x000003ff);
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_REFEN);

    mdelay(100);

    radaq_set_led(1, 1);
}

static void radaq_shutdown(void)
{
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_RUN);
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_REFEN);
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_STANDBY);
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_POWER);

    radaq_set_led(0, 0);
    radaq_set_led(1, 0);
}

static void radaq_start(void)
{
    rdq.running = 1;
    radaq_set_led(0, 1);
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_RUN);
    printk("radaq_start: running!\n");
}

/*
 * Userspace interface
 */
static int radaq_open(struct inode *inode, struct file *filp)
{
    printk("%s\n", __FUNCTION__);
    return 0;
}

static int radaq_release(struct inode *inode, struct file *filp)
{
    printk("%s\n", __FUNCTION__);
    return 0;
}

ssize_t radaq_read(struct file *filp, char *buf, size_t count, loff_t *offp)
{
    printk("%s(%p, %u, %lld)\n",
            __FUNCTION__, buf, count, *offp);
    
    if (rdq.overrun) {
        printk("read: overrun\n");
        return 0;
    }

    if (!rdq.running) {
        radaq_start();
    }

    if (down_interruptible(&rdq.sem)) {
        printk("read: aborted\n");
        radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_RUN);
        radaq_set_led(0, 0);
        return 0;
    }

    if (rdq.overrun) {
        printk("read: overrun\n");
        radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_RUN);
        radaq_set_led(0, 0);
        return 0;
    }

    if (copy_to_user((void *) buf, rdq.buffer[rdq.tpage], count)) {
        printk("read: failed to copy to userspace\n");
        return 0;
    }

    rdq.tpage = !rdq.tpage;

    return count;
}

ssize_t radaq_write(struct file *filp, const char *buf, size_t count, loff_t *offp)
{
    printk("%s(%p, %u, %lld)\n",
            __FUNCTION__, buf, count, *offp);
    return 0;
}

static long radaq_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    printk("%s(%u, %08lx)\n",
            __FUNCTION__, cmd, arg);

    switch (cmd) {
    case RADAQ_IOCTL_SAMPLERATE:
        printk("RADAQ_IOCTL_SAMPLERATE(fs=%luHz)\n", arg);
        if (arg < 1 || arg > SMPRATE_MAX) {
            return -EINVAL;
        }
        // XXX Implement!
        break;

    case RADAQ_IOCTL_CHANNELS:
        printk("RADAQ_IOCTL_CHANNELS(n=%lu)\n", arg);
        if (arg < 1 || arg > CHANNELS) {
            return -EINVAL;
        }
        rdq.channels = arg;
        break;

    case RADAQ_IOCTL_BUFFER_SIZE:
        printk("RADAQ_IOCTL_BUFFER_SIZE\n");
        if (copy_to_user((void *) arg, &rdq.bufsz, sizeof(rdq.bufsz))) {
            return -EINVAL;
        }
        break;

    case RADAQ_IOCTL_ARM:
        printk("RADAQ_IOCTL_ARM\n");
        rdq.idx = 0;
        rdq.hpage = 0;
        rdq.tpage = 0;
        rdq.running = 0;
        rdq.overrun = 0;
        break;
    }

    return 0;
}

static struct file_operations radaq_fops = {
    .read = radaq_read,
    .write = radaq_write,
    .unlocked_ioctl = radaq_unlocked_ioctl,
    .open = radaq_open,
    .release = radaq_release,
};

/*
 * Detect and initialize device
 */
static int radaq_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct device *i2cdev = &client->dev;
    uint8_t reg;
    //struct radaq *rdq;
    //struct proc_dir_entry *ledfs, *testfs;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    // Try communicate with the MCU and check protocol version
    if (radaq_read_i2c(i2cdev, &reg, PROTO_VERSION)) {
        dev_err(i2cdev, "Unable to read protocol version\n");
        return -ENODEV;
    }
    
    dev_notice(i2cdev, "Protocol version = %02x\n", reg);

    if (reg != PROTOCOL_VERSION) {
        dev_err(i2cdev, "Version not supported\n");
        return -ENODEV;
    }

    // Register GPIO pins to detect conflicts
    if (gpio_request_array(radaq_gpios, ARRAY_SIZE(radaq_gpios))) {
        dev_err(i2cdev, "Unable to claim GPIOs\n");
        return -ENODEV;
    }

    // Allocate a radaq struct
    //rdq = kmalloc(sizeof(struct radaq), GFP_KERNEL);
    rdq.idx = 0;
    sema_init(&rdq.sem, 0);
    rdq.i2cdev = i2cdev;
    memset(rdq.buffer, 0, sizeof(rdq.buffer));
    rdq.channels = CHANNELS;
    rdq.bufsz = BUFSIZE;
    rdq.hpage = 0;
    rdq.tpage = 0;
    rdq.running = 0;
    rdq.overrun = 0;
    rdq.leds = 0;

    // Alloc cdev
    rdq.cdev = cdev_alloc();
    cdev_init(rdq.cdev, &radaq_fops);
    rdq.cdev->owner = THIS_MODULE;

    if (cdev_add(rdq.cdev, devno, 1) < 0) {
        dev_err(i2cdev, "Unable to add device\n");
        cdev_del(rdq.cdev);
        gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));
        //kfree(rdq);
        return -ENOMEM;
    }

    rdq.ctrldev = device_create(class, NULL, devno, NULL, "radaq");
    if (IS_ERR(rdq.ctrldev)) {
        dev_err(i2cdev, "Unable to create device, error %ld\n", PTR_ERR(rdq.ctrldev));
        cdev_del(rdq.cdev);
        gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));
        return -ENOMEM;
    }

    // Register GPIO interrupt
    if (request_irq(gpio_to_irq(2), radaq_isr, IRQ_TYPE_EDGE_RISING, "Radaq", NULL)) {
        dev_err(i2cdev, "Unable to register GPIO interrupt\n");

        gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));
        //kfree(rdq);

        return -ENODEV;
    }

    radaq_initialize();

#if 0
    // Create procfs entry for controlling the leds
    ledfs = create_proc_entry("radaq_led", 0644, NULL);
    if (ledfs) {
        ledfs->data = i2cdev;
        ledfs->read_proc = radaq_procfs_led_read;
        ledfs->write_proc = radaq_procfs_led_write;
    } else {
        dev_warn(i2cdev, "Unable to register procfs entry\n");
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
#endif

    // Save out handle
    //i2c_set_clientdata(client, rdq);

    return 0;
}

static int __devexit radaq_remove(struct i2c_client *client)
{
#if 0
    struct radaq *rdq = i2c_get_clientdata(client);
#endif
    
    radaq_shutdown();

    device_destroy(class, devno);

#if 0
    // Remove procfs entry
    remove_proc_entry("radaq_led", NULL);
    remove_proc_entry("radaq_test", NULL);
#endif

    // Release GPIOs
    gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));

#if 0
    // Free out struct
    kfree(rdq);
#endif

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

static int radaq_init(void)
{
    int res;

    res = alloc_chrdev_region(&devno, 0, 1, "radaq");
    if (res < 0) {
        printk("radaq: Unable to allocate major device number\n");

        i2c_del_driver(&radaq_driver);

        return res;
    }

    class = class_create(THIS_MODULE, "radaq");
    if (IS_ERR(class)) {
        printk("radaq: Unable to create device class\n");

        unregister_chrdev_region(devno, 1);
        i2c_del_driver(&radaq_driver);

        return PTR_ERR(class);
    }

    i2c_add_driver(&radaq_driver);

    return 0;
}

static void radaq_cleanup(void)
{
    i2c_del_driver(&radaq_driver);

    class_destroy(class);

    unregister_chrdev_region(devno, 1);
}

module_init(radaq_init);
module_exit(radaq_cleanup);

//module_i2c_driver(radaq_driver);

MODULE_AUTHOR("Fredrik Ahlberg <fredrik@etf.nu>");
MODULE_DESCRIPTION("Radaq ADC driver");
MODULE_LICENSE("GPL");
