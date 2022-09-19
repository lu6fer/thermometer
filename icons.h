#define loading_width 40
#define loading_height 40
static uint8_t loading_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
    0xff, 0xff, 0xff, 0x00, 0x00, 0xfe, 0xff, 0x7f, 0x00, 0x00, 0x0c, 0x00,
    0x30, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00,
    0x00, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x00, 0x88,
    0xff, 0x19, 0x00, 0x00, 0x98, 0xff, 0x19, 0x00, 0x00, 0x10, 0xff, 0x08,
    0x00, 0x00, 0x30, 0xfe, 0x0c, 0x00, 0x00, 0x60, 0x3c, 0x06, 0x00, 0x00,
    0xc0, 0x00, 0x03, 0x00, 0x00, 0xc0, 0x81, 0x03, 0x00, 0x00, 0xe0, 0x00,
    0x03, 0x00, 0x00, 0x20, 0x18, 0x06, 0x00, 0x00, 0x30, 0x3c, 0x0c, 0x00,
    0x00, 0x18, 0x7e, 0x08, 0x00, 0x00, 0x18, 0xff, 0x18, 0x00, 0x00, 0x88,
    0xff, 0x10, 0x00, 0x00, 0x8c, 0xff, 0x11, 0x00, 0x00, 0x8c, 0xff, 0x11,
    0x00, 0x00, 0xcc, 0xff, 0x31, 0x00, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00,
    0x0c, 0x00, 0x30, 0x00, 0x00, 0xff, 0xff, 0x7f, 0x00, 0x00, 0xff, 0xff,
    0xff, 0x00, 0x00, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define clock_width 16
#define clock_height 16
static uint8_t clock_bits[] = {
    0xe0, 0x07, 0xf8, 0x1f, 0x3c, 0x3c, 0x8e, 0x71, 0x86, 0x61, 0x87, 0xe1,
    0x83, 0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83, 0xc3, 0x07, 0xe3, 0x06, 0x60,
    0x0e, 0x70, 0x3c, 0x3c, 0xf8, 0x1f, 0xe0, 0x07};

#define temperature_width 16
#define temperature_height 16
static uint8_t temperature_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x30, 0x3b, 0x30, 0x0b, 0x30, 0x08, 0x30, 0x08,
    0x30, 0x38, 0x30, 0x00, 0x78, 0x00, 0xfc, 0x00, 0xfe, 0x01, 0xfe, 0x01,
    0xfe, 0x01, 0xfe, 0x01, 0xfc, 0x00, 0x78, 0x00};

#define humidity_width 16
#define humidity_height 16
static uint8_t humidity_bits[] = {
    0x80, 0x01, 0xc0, 0x03, 0xe0, 0x07, 0xe0, 0x07, 0x70, 0x0e, 0x38, 0x1c,
    0x1c, 0x38, 0x6e, 0x7a, 0x66, 0x73, 0x86, 0x61, 0xc6, 0x66, 0x6e, 0x76,
    0x0c, 0x30, 0x3c, 0x3c, 0xf8, 0x1f, 0xe0, 0x07};

#define pressure_width 16
#define pressure_height 16
static uint8_t pressure_bits[] = {
    0xe0, 0x07, 0xf8, 0x1f, 0xfc, 0x3f, 0xfe, 0x7f, 0xfe, 0x7f, 0x7b, 0xfc,
    0x7b, 0xfd, 0x63, 0x8d, 0x6b, 0xbc, 0x6b, 0x8f, 0x6b, 0x8f, 0xfe, 0x7f,
    0xfe, 0x7f, 0xfc, 0x3f, 0xf8, 0x1f, 0xe0, 0x07};