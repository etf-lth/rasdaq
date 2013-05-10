#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>

#define MAXBUFSZ 2048

enum {
    RADAQ_SAMPLERATE = 0xcafe0000,
    RADAQ_CHANNELS = 0xcafe0001,
    RADAQ_BUFFER_SIZE = 0xcafe0002,
    RADAQ_ARM = 0xcafe0003,
};

int fd;

void radaq_open(void)
{
    fd = open("/dev/radaq", O_RDWR|O_NOCTTY);
    
    if (fd == -1) {
        perror("radaq_open: Unable to open device: ");
        exit(-1);
    }
}

int radaq_set_samplerate(unsigned int fs)
{
    if (ioctl(fd, RADAQ_SAMPLERATE, fs) < 0) {
        fprintf(stderr, "%s: Unable to set samplerate fs=%luHz\n", __FUNCTION__, fs);
        return -1;
    } else {
        return 0;
    }
}

int radaq_set_channels(unsigned int channels)
{
    if (ioctl(fd, RADAQ_CHANNELS, channels) < 0) {
        fprintf(stderr, "%s: Unable to set number of channels n=%lu\n", __FUNCTION__, channels);
        return -1;
    } else {
        return 0;
    }
}

size_t radaq_get_buffer_size(void)
{
    size_t res;

    if (ioctl(fd, RADAQ_BUFFER_SIZE, &res) < 0) {
        fprintf(stderr, "%s: Unable to get buffer size\n", __FUNCTION__);
        return 0;
    } else {
        return res;
    }
}

int radaq_read_buffer(unsigned short *data, unsigned int samples)
{
    size_t bytesRead = read(fd, data, samples * sizeof(unsigned short));

    if (bytesRead < samples * sizeof(unsigned short)) {
        fprintf(stderr, "%s: Short read! (read %lu bytes, expected %lu bytes)\n",
                __FUNCTION__, bytesRead, samples * sizeof(unsigned short));
        return -bytesRead;
    } else {
        return bytesRead;
    }
}

int radaq_arm(void)
{
    if (ioctl(fd, RADAQ_ARM) < 0) {
        fprintf(stderr, "%s: Unable to arm device\n", __FUNCTION__);
        return -1;
    } else {
        return 0;
    }
}

int main(void)
{
    radaq_open();

    if (radaq_set_samplerate(20000) < 0) {
        return -1;
    }

    if (radaq_set_channels(1) < 0) {
        return -1;
    }

    size_t bufsz = radaq_get_buffer_size();
    if (!bufsz) {
        return -1;
    }

    if (bufsz > MAXBUFSZ || bufsz < 1) {
        fprintf(stderr, "Invaild buffer size! Must be 0 < %lu <= %lu\n",
                bufsz, MAXBUFSZ);
        return -1;
    }

    fprintf(stderr, "Using buffer page size %d samples\n", bufsz);

    if (radaq_arm() < 0) {
        return -1;
    }

    FILE *f = fopen("/tmp/radaq.bin", "w");
    unsigned short buffer[MAXBUFSZ];
    int pages = 0;

    while (1) {
        int read = radaq_read_buffer(buffer, bufsz);

        if (read > 0) {
            fwrite(buffer, read, 1, f);
            fflush(f);
        } else {
            break;
        }
        
        printf("\r%d pages read (last page=%d bytes)", ++pages, read);
    }

    fclose(f);
}

