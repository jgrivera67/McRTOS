/**
 * @file mp3.h
 *
 * MP3 player device (VS1002 chip) abstraction layer interface
 *
 * @author German Rivera 
 */ 
#ifndef MP3_H
#define MP3_H

#include <stdint.h>
#include <stddef.h>

#define MP3_MIN_VOLUME      UINT8_C(0xff)   /* silence */ 
#define MP3_MAX_VOLUME      UINT8_C(0x00)
#define MP3_DEFAULT_VOLUME  UINT8_C(0x20)

struct ssp_controller;

void init_mp3(const struct ssp_controller *ssp_controller_p);

void mp3_soft_reset(void);

void mp3_play_sine_test(void);

void mp3_play_buffer(const uint8_t *buffer, size_t length);

void mp3_set_volume(uint_fast8_t volume);

#endif /* MP3_H */
