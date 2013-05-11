#include <fcntl.h>
#include "radaq.h"

static int fd;

int radaq_open(void)
{
    fd = open("/dev/radaq", O_RDWR|O_NOCTTY);
    
    if (fd == -1) {
        return -1;
    } else {
        return 0;
    }
}

int radaq_set_samplerate(unsigned int fs)
{
    if (ioctl(fd, RADAQ_SAMPLERATE, fs) < 0) {
        return -1;
    } else {
        return 0;
    }
}

int radaq_set_channels(unsigned int channels)
{
    if (ioctl(fd, RADAQ_CHANNELS, channels) < 0) {
        return -1;
    } else {
        return 0;
    }
}

size_t radaq_get_buffer_size(void)
{
    size_t res;

    if (ioctl(fd, RADAQ_BUFFER_SIZE, &res) < 0) {
        return 0;
    } else {
        return res;
    }
}

int radaq_read_buffer(unsigned short *data, unsigned int samples)
{
    size_t bytesRead = read(fd, data, samples * sizeof(unsigned short));

    if (bytesRead < samples * sizeof(unsigned short)) {
        return -bytesRead;
    } else {
        return bytesRead;
    }
}

int radaq_arm(void)
{
    if (ioctl(fd, RADAQ_ARM) < 0) {
        return -1;
    } else {
        return 0;
    }
}

int radaq_halt(void)
{
    if (ioctl(fd, RADAQ_HALT) < 0) {
        return -1;
    } else {
        return 0;
    }
}
