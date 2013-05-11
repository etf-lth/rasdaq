#ifndef RADAQ_H
#define RADAQ_H

enum {
    RADAQ_SAMPLERATE = 0xcafe0000,
    RADAQ_CHANNELS = 0xcafe0001,
    RADAQ_BUFFER_SIZE = 0xcafe0002,
    RADAQ_ARM = 0xcafe0003,
    RADAQ_HALT = 0xcafe0004,
};

int radaq_open(void);
int radaq_set_samplerate(unsigned int fs);
int radaq_set_channels(unsigned int channels);
size_t radaq_get_buffer_size(void);
int radaq_read_buffer(unsigned short *data, unsigned int samples);
int radaq_arm(void);
int radaq_halt(void);

#endif
