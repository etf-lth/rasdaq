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
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <asm/fiq.h>
#include <mach/dma.h>

//#define RADAQ_USE_COMPLETION
#define RADAQ_DEBUG

/*
 * IOCTLs for userspace interface
 */
enum {
    RADAQ_IOCTL_SAMPLERATE      = 0xcafe0000,
    RADAQ_IOCTL_CHANNELS        = 0xcafe0001,
    RADAQ_IOCTL_BUFFER_SIZE     = 0xcafe0002,
    RADAQ_IOCTL_ARM             = 0xcafe0003,
    RADAQ_IOCTL_HALT            = 0xcafe0004,
    RADAQ_IOCTL_RESET           = 0xcafe0005,
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
#define GPIOEDS(x) (0x40+(x)*4)
#define GPIOREN(x) (0x4c+(x)*4)

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
    PROTO_FSDIVH    = 13,
    PROTO_FSDIVL    = 14,
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

    { 45,   GPIOF_OUT_INIT_LOW, "Radaq ISR Debug" }
};

/*
 * Some static data
 */
static dev_t devno;
static struct class *class;
void __iomem *gpio;

struct radaq {
    struct cdev *cdev;
    struct device *i2cdev, *ctrldev;

    uint32_t raw[2][BUFSIZE*CHANNELS];

    //int16_t buffer[2][BUFSIZE*CHANNELS];
    size_t hpage, tpage, idx, buflen;
#ifdef RADAQ_USE_COMPLETION
    struct completion comp;
#else
    struct semaphore sem;
#endif
    int channels;
    uint8_t leds;
    int running, overrun, armed;
} rdq;

static struct fiq_handler fh = {
    .name = "radaq_fiq",
};
static uint8_t fiqStack[1024];

struct bcm2708_dma_cb dmacb;
void __iomem *dma_cb_phys;

void __iomem *dma_base;

struct bcm2708_dma_cb *dma_cb_base;

#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP   (1<<3)
#define DMA_D_DREQ      (1<<6)
#define DMA_PER_MAP(x)  ((x)<<16)
#define DMA_END         (1<<1)
#define DMA_RESET       (1<<31)
#define DMA_INT         (1<<2)

#define DMA_CS          0x00
#define DMA_CONBLK_AD   0x04
#define DMA_DEBUG       0x20

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

/*
 * Toggle led and write register value
 */
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
 * Calculate sample rate divisor and write register values
 */
static void radaq_set_samplerate(unsigned int fs)
{
    unsigned short divisor = (20000000L / fs) - 1;

    radaq_write_i2c(rdq.i2cdev, divisor >> 8, PROTO_FSDIVH);
    radaq_write_i2c(rdq.i2cdev, divisor, PROTO_FSDIVL);
}

/*
 * Write the ADC config register
 */
static int radaq_write_reg(struct device *dev, uint32_t reg)
{
    int idx;

#ifdef RADAQ_DEBUG
    dev_notice(dev, "Writing ADC Config = %08x\n", reg);
#endif

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
    //int delay;

    // assert /cs
    writel(1 << 30, gpio + GPIOCLR(0));

    while (channels--) {
        // assert /rd
        writel(1 << 31, gpio + GPIOCLR(0));
        
        // sample data bus
        *data++ = readl(gpio + GPIOLEV(0));

        // deassert /rd
        writel(1 << 31, gpio + GPIOSET(0));
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

void __attribute__ ((naked)) radaq_handle_fiq(void)
{
	/* entry takes care to store registers we will be treading on here */
	asm __volatile__ (
		"mov     ip, sp ;"
		/* stash FIQ and normal regs */
		"stmdb	sp!, {r0-r12,  lr};"
		/* !! THIS SETS THE FRAME, adjust to > sizeof locals */
		"sub     fp, ip, #256 ;"
		);
    
    // ack interrupts on all banks
    writel(0xffffffff, __io_address(GPIO_BASE) + GPIOEDS(0));
    writel(0xffffffff, __io_address(GPIO_BASE) + GPIOEDS(1));

    radaq_read_result(&rdq.raw[rdq.hpage][rdq.idx], rdq.channels);
    rdq.idx += rdq.channels;

    if (rdq.idx == rdq.buflen) {
        rdq.idx = 0;
        rdq.hpage = !rdq.hpage;

        if (rdq.hpage == rdq.tpage) {
            rdq.overrun = 1;
            rdq.running = 0;
        }

        writel(1 << 13, __io_address(GPIO_BASE) + GPIOSET(1));
        writel(1 << 13, __io_address(GPIO_BASE) + GPIOCLR(1));

        bcm_dma_start(dma_base, (dma_addr_t)dma_cb_phys);
    }

    /* epilogue */
	mb();

	/* exit back to normal mode restoring everything */
	asm __volatile__ (
		/* return FIQ regs back to pristine state
		 * and get normal regs back
		 */
		"ldmia	sp!, {r0-r12, lr};"

		/* return */
		"subs	pc, lr, #4;"
	);
}

/*
 * Conversion Complete Interrupt Service Routine (tm)
 */
static irqreturn_t radaq_handle_irq(int irq, void *data)
{
#if 0
    uint32_t result[CHANNELS], idx;
    //struct radaq *rdq = data;

    if (!rdq.running) {
#ifdef RADAQ_DEBUG
        //printk("isr: not running!\n");
#endif
        return IRQ_HANDLED;
    }

    writel(1 << 13, gpio + GPIOSET(1));

    radaq_read_result(result, rdq.channels);

    for (idx=0; idx<rdq.channels; idx++) {
        rdq.buffer[rdq.hpage][rdq.idx++] = radaq_unswizzle(result[idx]);
        if (rdq.idx == BUFSIZE) {
            rdq.idx = 0;
            rdq.hpage = !rdq.hpage;

            if (rdq.hpage == rdq.tpage) {
                rdq.overrun = 1;
                rdq.running = 0;
#ifdef RADAQ_DEBUG
                //printk("isr: buffer overrun!\n");
#endif
            }
#endif

    writel(BCM2708_DMA_INT, dma_base + BCM2708_DMA_CS);

    printk("%s!\n", __FUNCTION__);

#ifdef RADAQ_USE_COMPLETION
            complete(&rdq.comp);
#else
            up(&rdq.sem);
#endif

#if 0
            //tasklet_schedule(&radaq_tasklet);
        }
    }

    writel(1 << 13, gpio + GPIOCLR(1));
#endif


    // clear pin
    //writel(1 << 13, __io_address(GPIO_BASE) + GPIOCLR(1));

    // ack bank 1
    //writel(0xffffffff, __io_address(GPIO_BASE) + GPIOEDS(1));

    return IRQ_HANDLED;
}

static struct irqaction radaq_irq = {
    .name = "Radaq buffer handler",
    .flags = IRQF_DISABLED | IRQF_IRQPOLL,
    .handler = radaq_handle_irq,
};

/*
 * Power up ADC and set configuration
 */
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

/*
 * Power down ADC
 */
static void radaq_shutdown(void)
{
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_RUN);
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_REFEN);
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_STANDBY);
    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_POWER);

    radaq_set_led(0, 0);
    radaq_set_led(1, 0);
}


/*
 * Start sampling!
 */
static void radaq_start(void)
{
#ifdef RADAQ_DEBUG
    printk("%s\n", __FUNCTION__);
#endif

    rdq.running = 1;
    radaq_set_led(0, 1);
    radaq_write_i2c(rdq.i2cdev, 0x01, PROTO_RUN);
}

/*
 * Stop pending operation
 */
static void radaq_stop(void)
{
#ifdef RADAQ_DEBUG
    printk("%s\n", __FUNCTION__);
#endif

    radaq_write_i2c(rdq.i2cdev, 0x00, PROTO_RUN);
    radaq_set_led(0, 0);
    rdq.running = 0;
}

/*
 * Reset ADC
 */
static void radaq_reset(void)
{
#ifdef RADAQ_DEBUG
    printk("%s\n", __FUNCTION__);
#endif

    radaq_stop();
    radaq_set_led(1, 0);
    mdelay(100);
    radaq_write_i2c(rdq.i2cdev, 0, PROTO_STRB_RST);
    radaq_write_reg(rdq.i2cdev, (1<<31)|(1<<27)|(1<<13)|0x000003ff);
    mdelay(100);
    radaq_set_led(1, 1);
}

static void radaq_adjust_buffer(void)
{
    size_t tuples = (BUFSIZE * CHANNELS) / rdq.channels;
    rdq.buflen = tuples * rdq.channels;

#ifdef RADAQ_DEBUG
    printk("%s: buffer=%d, channels=%d, tuples=%d, buflen=%d\n",
            __FUNCTION__, BUFSIZE*CHANNELS, rdq.channels, tuples, rdq.buflen);
#endif
}

/*
 * Userspace interface
 */
static int radaq_open(struct inode *inode, struct file *filp)
{
#ifdef RADAQ_DEBUG
    printk("%s\n", __FUNCTION__);
#endif
    radaq_reset();
    rdq.armed = 0;
    return 0;
}

static int radaq_release(struct inode *inode, struct file *filp)
{
#ifdef RADAQ_DEBUG
    printk("%s\n", __FUNCTION__);
#endif
    radaq_stop();
    return 0;
}

ssize_t radaq_read(struct file *filp, char *buf, size_t count, loff_t *offp)
{
    int idx;
    uint16_t *p;

    if (rdq.overrun) {
#ifdef RADAQ_DEBUG
        printk("%s: overrun\n", __FUNCTION__);
#endif
        return 0;
    }

    if (!rdq.running) {
        if (rdq.armed) {
            radaq_start();
            rdq.armed = 0;
        } else {
#ifdef RADAQ_DEBUG
            printk("%s: not armed\n", __FUNCTION__);
#endif
            return 0;
        }
    }

#ifdef RADAQ_USE_COMPLETION
    init_completion(&rdq.comp);
    wait_for_completion(&rdq.comp);
#else
    if (down_interruptible(&rdq.sem)) {
#ifdef RADAQ_DEBUG
        printk("%s: aborted\n", __FUNCTION__);
#endif
        radaq_stop();
        return 0;
    }
#endif

    if (rdq.overrun) {
#ifdef RADAQ_DEBUG
        printk("%s: overrun\n", __FUNCTION__);
#endif
        radaq_stop();
        return 0;
    }

    p = (uint16_t *)rdq.raw[rdq.tpage];
    for (idx=0; idx<rdq.buflen; idx++) {
        *p++ = radaq_unswizzle(rdq.raw[rdq.tpage][idx]);
    }

    if (copy_to_user((void *) buf, rdq.raw[rdq.tpage], count)) {
#ifdef RADAQ_DEBUG
        printk("%s: failed to copy to userspace\n", __FUNCTION__);
#endif
        return 0;
    }

    rdq.tpage = !rdq.tpage;

    return count;
}

ssize_t radaq_write(struct file *filp, const char *buf, size_t count, loff_t *offp)
{
#ifdef RADAQ_DEBUG
    printk("%s(%p, %u, %lld)\n",
            __FUNCTION__, buf, count, *offp);
#endif
    return 0;
}

static long radaq_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case RADAQ_IOCTL_SAMPLERATE:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_SAMPLERATE(fs=%luHz)\n", arg);
#endif
        if (arg < 1 || arg > SMPRATE_MAX) {
            return -EINVAL;
        }
        radaq_set_samplerate(arg);
        break;

    case RADAQ_IOCTL_CHANNELS:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_CHANNELS(n=%lu)\n", arg);
#endif
        if (arg < 1 || arg > CHANNELS) {
            return -EINVAL;
        }
        rdq.channels = arg;
        radaq_adjust_buffer();
        break;

    case RADAQ_IOCTL_BUFFER_SIZE:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_BUFFER_SIZE\n");
#endif
        if (copy_to_user((void *) arg, &rdq.buflen, sizeof(rdq.buflen))) {
            return -EINVAL;
        }
        break;

    case RADAQ_IOCTL_ARM:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_ARM\n");
#endif
        rdq.idx = 0;
        rdq.hpage = 0;
        rdq.tpage = 0;
        rdq.running = 0;
        rdq.overrun = 0;
        rdq.armed = 1;
        break;

    case RADAQ_IOCTL_HALT:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_HALT\n");
#endif
        radaq_stop();
        break;

    case RADAQ_IOCTL_RESET:
#ifdef RADAQ_DEBUG
        printk("RADAQ_IOCTL_RESET\n");
#endif
        radaq_reset();
        break;

    default:
#ifdef RADAQ_DEBUG
        printk("%s: Invalid call (%u, %08lx)\n", __FUNCTION__, cmd, arg);
#endif
        return -EINVAL;
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
    struct pt_regs regs;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    // Try communicate with the MCU and check protocol version
    if (radaq_read_i2c(i2cdev, &reg, PROTO_VERSION)) {
        dev_err(i2cdev, "Unable to read protocol version\n");
        return -ENODEV;
    }
    
#ifdef RADAQ_DEBUG
    dev_notice(i2cdev, "Protocol version = %02x\n", reg);
#endif

    if (reg != PROTOCOL_VERSION) {
        dev_err(i2cdev, "Protocol version %02x is not supported\n", reg);
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
#ifndef RADAQ_USE_COMPLETION
    sema_init(&rdq.sem, 0);
#endif
    rdq.i2cdev = i2cdev;
    memset(rdq.raw, 0, sizeof(rdq.raw));
    rdq.channels = CHANNELS;
    rdq.hpage = 0;
    rdq.tpage = 0;
    rdq.running = 0;
    rdq.overrun = 0;
    rdq.leds = 0;
    rdq.armed = 0;
    radaq_adjust_buffer();
    
    //dma_cb_phys = mem_virt_to_phys(&dmacb);
    dma_cb_base = dma_alloc_writecombine(NULL, SZ_4K, &dma_cb_phys, GFP_KERNEL);
#ifdef RADAQ_DEBUG
    printk("radaq: dma_cb_base=%08x, dma_cb_phys = %08x\n", dma_cb_base, dma_cb_phys);
#endif

    dma_cb_base->info = BCM2708_DMA_INT_EN | BCM2708_DMA_WAIT_RESP | (1<<7);
    dma_cb_base->src = 0;
    dma_cb_base->dst = 0;
    dma_cb_base->length = 1;
    dma_cb_base->stride = 0;
    dma_cb_base->next = 0;

    int dma_irq = 0;

    if (bcm_dma_chan_alloc(0, &dma_base, &dma_irq) < 0) {
        printk("%s: unable to alloc dma\n", __FUNCTION__);
    } else {
        printk("%s: dma_base = %08x, dma_irq = %d\n", __FUNCTION__, dma_base, dma_irq);
    }

    /*writel(DMA_RESET, __io_address(DMA_BASE) + DMA_CS);
    udelay(10);
    writel(DMA_INT|DMA_END, __io_address(DMA_BASE) + DMA_CS);
    writel(7, __io_address(DMA_BASE) + DMA_DEBUG);*/

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
#if 0
    if (request_irq(gpio_to_irq(2), radaq_isr, IRQ_TYPE_EDGE_RISING, "Radaq", NULL)) {
        dev_err(i2cdev, "Unable to register GPIO interrupt\n");

        gpio_free_array(radaq_gpios, ARRAY_SIZE(radaq_gpios));
        //kfree(rdq);

        return -ENODEV;
    }
#endif

    // Initialize FIQ
    claim_fiq(&fh);
    set_fiq_handler(__FIQ_Branch, 8);
    memset(&regs, 0, sizeof(regs));
    regs.ARM_r8 = (long)radaq_handle_fiq;
    regs.ARM_r9 = (long)0;
    regs.ARM_sp = (long)fiqStack + sizeof(fiqStack) - 4;
    set_fiq_regs(&regs);
    enable_fiq(INTERRUPT_GPIO3);

    writel(1 << 2, __io_address(GPIO_BASE)+GPIOEDS(0));
    writel(1 << 2, __io_address(GPIO_BASE)+GPIOREN(0));

    // Initialize IRQ
    /*writel(1 << 13, __io_address(GPIO_BASE)+GPIOEDS(1));
    writel(1 << 13, __io_address(GPIO_BASE)+GPIOREN(1));*/
    //setup_irq(IRQ_DMA0, &radaq_irq);
    if (dma_irq) {
        printk("radaq: registering irq\n");
        setup_irq(dma_irq, &radaq_irq);
    }

    radaq_initialize();

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

/*static inline void __iomem *UserVirtualToBus(void __user *pUser)
{
    int mapped;
    struct page *pPage;
    void *phys;

    mapped = get_user_pages(current, current->mm,
           (unsigned long)pUser, 1,
           1, 0,
           &pPage,
           0);

    if (mapped <= 0)
        return 0;

    phys = page_address(pPage) + offset_in_page(pUser);
    page_cache_release(pPage);

    return (void __iomem *) __virt_to_bus(phys);
}*/

static int radaq_init(void)
{
    int res;

    gpio = __io_address(GPIO_BASE);

    printk("radaq driver (c) 2013, Fredrik Ahlberg <fredrik@etf.nu>\n");

#ifdef RADAQ_DEBUG
#ifdef RADAQ_USE_COMPLETION
    printk("radaq: Using completion\n");
#else
    printk("radaq: Using semaphore\n");
#endif
#endif


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
