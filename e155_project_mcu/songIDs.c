#include "songIDs.h"

uint8_t getSongID(uint64_t card_id){
  switch(card_id){
    case (0x7945dc11):
      return 0x01;
      break;
    case (0x1c2ae62e):
      return 0x02;
      break;
    case (0x984e62e):
      return 0x03;
      break;
    case (0xf966e62e):
      return 0x04;
      break;
    default: return 0x30;

  }
}
