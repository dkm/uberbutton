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
  "Clem",
  "Jérome",
  "Marc",
  "GuillaumeS",
  "Dams",
  "glager",
  [8 ... 0xFE]="none",
  [0xFF]="ALL"
};


const char* uber_get_type(uint8_t id){
  if(id > (sizeof(uber_events)/sizeof(char*)))
    return "INVALID!";
  return uber_events[id];
}

const char* uber_get_name(uint8_t id){
  if(id > (sizeof(uber_ids)/sizeof(char*)))
    return "INVALID!";
  return uber_ids[id > sizeof(uber_ids)/sizeof(char*) ? 0xFF : id];
}

void uber_get_frame(radiohead_frame_t *f,  char* buf){
  const char *src = (f->frame.header.src < sizeof(uber_ids)/sizeof(char*)) ?
    uber_ids[f->frame.header.src] :
    "INV";

  const char *dst = (f->frame.header.dst < sizeof(uber_ids)/sizeof(char*)) ?
    uber_ids[f->frame.header.dst] :
    "INV";

  const char *t = (f->frame.header.type < sizeof(uber_events)/sizeof(char*)) ?
    uber_events[f->frame.header.type] :
    "INV";  
  sprintf (buf,
	   "s:%s d:%s t:%s",
	   src,
	   dst,
	   t);
}


int uber_dump_frame(radiohead_frame_t *f){
  const char *src = f->frame.header.src < (sizeof(uber_ids)/sizeof(char*)) ?
    uber_ids[f->frame.header.src] :
    "INV";

  const char *dst = f->frame.header.dst < (sizeof(uber_ids)/sizeof(char*)) ?
    uber_ids[f->frame.header.dst] :
    "INV";

  const char *t = f->frame.header.type < (sizeof(uber_events)/sizeof(char*)) ?
    uber_events[f->frame.header.type] :
    "INV";  

  printf ("discard : %" PRIx16 ", %" PRIx16 "\n", f->radiohead_src, f->radiohead_dst);
  printf ("flags : %x\n", f->frame.header.flags);

  printf ("src:    %x (%s)\n", f->frame.header.src, src);
  
  printf ("dst:    %x (%s)\n", f->frame.header.dst, dst);

  printf ("type:   %x (%s)\n", f->frame.header.type, t);
  printf ("pl_sz:  %x\n", f->frame.header.payload_sz);
  return 0;
}

size_t uber_get_frame_size(radiohead_frame_t *t){
  return 4 /*radiohead header */ + sizeof(uber_header_t) + t->frame.header.payload_sz;
}
