/*
 * rdqtool - Controlling Radaq data acquisition
 *
 * (c) 2013, Fredrik Ahlberg <fredrik@etf.nu>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "radaq.h"

#define MAXBUFSZ 65536

int quiet = 0;

int prepare_device(unsigned int samplerate, unsigned int channels, size_t maxbufsz, size_t *bufsz)
{
    if (radaq_open() < 0) {
        perror("radaq_open: Unable to open device");
        return -1;
    }

    if (radaq_set_samplerate(samplerate) < 0) {
        fprintf(stderr, "Unable to set samplerate fs=%uHz\n", samplerate);
        return -1;
    }

    if (radaq_set_channels(channels) < 0) {
        fprintf(stderr, "Unable to set number of channels n=%u\n", channels);
        return -1;
    }

    if (radaq_set_max_buffer_size(maxbufsz) < 0) {
        fprintf(stderr, "Unable to set preferred buffer size n=%u\n", maxbufsz);
        return -1;
    }

    *bufsz = radaq_get_buffer_size();
    if (!*bufsz) {
        fprintf(stderr, "Unable to get buffer size\n");
        return -1;
    }

    if (*bufsz > MAXBUFSZ || *bufsz < 1) {
        fprintf(stderr, "Invalid buffer size! Must be 0 < %lu <= %lu\n",
                *bufsz, MAXBUFSZ);
        return -1;
    }

    if (!quiet) {
        fprintf(stderr, "Buffer page size = %d samples\n", *bufsz);
        fprintf(stderr, "Samplerate = %d Hz, %d channels\n", samplerate, channels);
    }

    if (radaq_arm() < 0) {
        fprintf(stderr, "Unable to arm device\n");
        return -1;
    }

    return 0;
}

int sample(size_t bufsz, unsigned int maxsamp, FILE *output)
{
    unsigned short buffer[MAXBUFSZ];
    unsigned pages = 0, samples = 0;

    while (!maxsamp || samples < maxsamp) {
        int read = radaq_read_buffer(buffer, bufsz);

        if (read > 0) {
            fwrite(buffer, read, 1, output);
            fflush(output);
        } else {
            if (!quiet) {
                fprintf(stderr, "\n%s: Buffer underrun! (read %u bytes, expected %u bytes)\n",
                    __FUNCTION__, read, bufsz * sizeof(unsigned short));
            }
            break;
        }
        
        samples += bufsz;

        if (!quiet) {
            fprintf(stderr, "\r%d pages read (last page=%d bytes)", ++pages, read);
        }
    }
    if (!quiet) {
        fprintf(stderr, "\n");
    }

    return 0;
}

int main(int argc, char **argv)
{
    int c;
    unsigned int samplerate = 10000, channels = 8, maxbufsz = 8192, maxsamp = 0;
    size_t bufsz;
    FILE *output = stdout;

    while ((c = getopt(argc, argv, "hqc:r:b:o:n:")) != -1) {
        switch (c) {
        case 'c':
            channels = atoi(optarg);
            break;

        case 'r':
            samplerate = atoi(optarg);
            break;

        case 'b':
            maxbufsz = atoi(optarg);
            break;

        case 'o':
            output = fopen(optarg, "wb");
            break;

        case 'n':
            maxsamp = atoi(optarg);
            break;

        case 'q':
            quiet = 1;
            break;

        case 'h':
            fprintf(stderr, "rdqtool (c) Fredrik Ahlberg, 2013 <fredrik@etf.nu>\n\n" \
                    "Options:\t-q\tquiet\n" \
                    "\t\t-c[n]\tnumber of channels, 1..8, default: 8\n" \
                    "\t\t-r[fs]\tsample rate in Hz, 1..500000, default: 10000\n" \
                    "\t\t-b[n]\tmaximum buffer size in tuples, 1..8192\n" \
                    "\t\t-o[fn]\toutput filename, default: stdout\n" \
                    "\t\t-n[ns]\tStop after n samples, default: run forever\n");
            return 0;

        case '?':
            //fprintf(stderr, "Unknown argument '-%c'!\n", optopt);
            return -1;
        }
    }

    if (prepare_device(samplerate, channels, maxbufsz, &bufsz) < 0) {
        return -1;
    }

    if (sample(bufsz, maxsamp, output) < 0) {
        return -1;
    }

    radaq_halt();

    if (output != stdout) {
        fclose(output);
    }
}

