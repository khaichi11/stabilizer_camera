// host mock: non-blocking send() onto a fake TCP connection (see WiFi.h)
#pragma once
#include <cstddef>
#define MSG_DONTWAIT 0x40
int send(int fd, const void *buf, size_t n, int flags);
