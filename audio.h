/**
 * @file audio.h
 * @brief Audio processing module
 */

#ifndef AUDIO_H_FILE
#define AUDIO_H_FILE

/**
 * @def AUDIO_BUFFER_SIZE
 * @brief Size of the audio buffer
 */
#define AUDIO_BUFFER_SIZE 1024

/**
 * @def AUDIO_MAX_SOURCES
 * @brief Maximum number of audio sources
 */
#define AUDIO_MAX_SOURCES 6

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize audio module
 * @param audio_pin Audio pin number
 * @param sample_freq Sample frequency
 */
void audio_init(int audio_pin, int sample_freq);

/**
 * @brief Get audio buffer
 * @return Audio buffer pointer or NULL if not available
 */
uint8_t *audio_get_buffer(void);

/**
 * @brief Play a single audio sample
 * @param samples Sample data
 * @param len Sample length
 * @return Index of the played source or -1 if none available
 */
int audio_play_once(const uint8_t *samples, int len);

/**
 * @brief Play a looping audio sample
 * @param samples Sample data
 * @param len Sample length
 * @param loop_start Loop start index
 * @return Index of the played source or -1 if none available
 */
int audio_play_loop(const uint8_t *samples, int len, int loop_start);

/**
 * @brief Stop an audio source
 * @param source_id Index of the source to stop
 */
void audio_source_stop(int source_id);

/**
 * @brief Set the volume of an audio source
 * @param source_id Index of the source to set volume for
 * @param volume Volume value (8.8 fixed point)
 */
void audio_source_set_volume(int source_id, uint16_t volume);

/**
 * @brief Perform a single step of the audio mixer
 */
void audio_mixer_step(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_H_FILE */

