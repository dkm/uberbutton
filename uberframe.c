#include "uberframe.h"
#include <stdio.h>

static const char *uber_events[] = {
  "payload",
  "pause",
  "rencontre",
  "repas",
  "clope",
  "café",
  "piscine",
  "épicuria",
  "vélo",
  "f*ck",
  "ack"
};

static const char *uber_ids[] = {
  "INVALID",
  "BenjM",
  "Clém",
  "Jérôme",
  "Marc",
  "GuillaumeS",
  "Dams",
  "GuillaumeL",
  [0xFF] "ALL"
};


int uber_dump_frame(radiohead_frame_t *f){
  printf ("discard : %" PRIx16 ", %" PRIx16 "\n", f->radiohead_src, f->radiohead_dst);
  printf ("flags : %x\n", f->frame.header.flags);

  printf ("src:    %x (%s)\n", f->frame.header.src, uber_ids[f->frame.header.src]);
  
  printf ("dst:    %x (%s)\n", f->frame.header.dst, uber_ids[f->frame.header.dst]);

  printf ("type:   %x (%s)\n", f->frame.header.type, uber_events[f->frame.header.type]);
  printf ("pl_sz:  %x\n", f->frame.header.payload_sz);
  return 0;
}

size_t uber_get_frame_size(radiohead_frame_t *t){
  return 4 /*radiohead header */ + sizeof(uber_header_t) + t->frame.header.payload_sz;
}
