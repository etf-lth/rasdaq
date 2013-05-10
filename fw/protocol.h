#ifndef PROTOCOL_H
#define PROTOCOL_H

#define PROTO_VERSION_BYTE 0x01

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

#endif
