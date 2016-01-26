#pragma once

#include <inttypes.h>
#include <stdlib.h>

#define MAX_PAYLOAD_SIZE 10

typedef struct __attribute__((packed)) {
  uint8_t flags;
  uint8_t src;
  uint8_t dst;
  uint8_t type;
  uint8_t payload_sz;
} uber_header_t;

typedef struct __attribute__((packed)) {
  uber_header_t header;
  uint8_t payload[MAX_PAYLOAD_SIZE];
} uber_frame_t;

typedef struct __attribute__((packed)) {
  uint8_t radiohead_src;
  uint8_t radiohead_dst;
  uint8_t radiohead_id;
  uint8_t radiohead_flags;
  uber_frame_t frame;
} radiohead_frame_t;

int uber_dump_frame(radiohead_frame_t *f);
size_t uber_get_frame_size(radiohead_frame_t *t);
void uber_get_frame(radiohead_frame_t *f,  char* buf);
const char* uber_get_name(uint8_t id);
