#ifndef __ADNS_3080_H__
#define __ADNS_3080_H__

#include "spi.h"
#include <stdbool.h>


bool is_connect_ADNS3080(void);     // check spi comm, 3ms
void init_ADNS3080(bool ips_1600);  // reset, wait, set to 1600ips
bool update_ADNS3080(void);
int8_t get_DeltaX_ADNS3080(void);
int8_t get_DeltaY_ADNS3080(void);
uint8_t get_Qualty_ADNS3080(void);

void clear_XY_ADNS3080(void);
int32_t get_X_ADNS3080(void);
int32_t get_Y_ADNS3080(void);

void frame_print_ADNS3080(void);

#endif /* __ADNS_3080_H__ */
