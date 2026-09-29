/*
 * I2S Sample Player - ESP32 als I2S-Slave
 *
 * Sendet zwei 24-Bit-Signale als Stereo-Signal mit 96 kHz Abtastrate.
 * Der ESP32 arbeitet als SLAVE (BCLK/WS kommen vom externen Master, z. B. Zynq).
 *
 * Signalformen, Frequenz und Amplitude sind ueber die serielle Schnittstelle
 * (UART0, 115200 8N1) einstellbar. Zusaetzlich wird periodisch die gemessene
 * Abtastrate und der aktuelle Wert beider Kanaele geloggt.
 *
 * Pins:
 *   BCLK  = GPIO 4   (Eingang)
 *   WS    = GPIO 19  (Eingang)
 *   DOUT  = GPIO 18  (Ausgang)
 *
 * Datenformat (Philips/I2S Standard):
 *   24 Bit pro Sample, linksbuendig in einem 32-Bit-Wort abgelegt
 *   (Sample-Wert << 8), da der ESP32 bei 24 Bit die oberen 24 Bit sendet.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "i2s_player";

/* -------------------------------------------------------------------------
 * Feste I2S-Konfiguration
 * ---------------------------------------------------------------------- */
#define SAMPLE_RATE_HZ           96000        /* Soll-Abtastrate (Master gibt Takt vor) */
#define I2S_BCLK_IO             GPIO_NUM_4    /* Bit Clock (Eingang im Slave-Modus)     */
#define I2S_WS_IO               GPIO_NUM_19   /* Word Select (Eingang im Slave-Modus)   */
#define I2S_DOUT_IO             GPIO_NUM_18   /* Serial Data (Ausgang)                  */

#define FRAMES_PER_BUFFER        256           /* Stereo-Frames pro DMA-Puffer          */

/* Bytes pro Stereo-Frame: 4 Byte links + 4 Byte rechts = 8 Byte (24 Bit linksbuendig) */
#define BYTES_PER_FRAME         (2 * sizeof(int32_t))
#define BUFFER_SIZE_BYTES       (FRAMES_PER_BUFFER * BYTES_PER_FRAME)

/* 24-Bit-Vollaussteuerung: 2^23 - 1 */
#define MAX_AMPLITUDE           8388607

/* Bits pro Stereo-Frame (Philips, 24 Bit in 32-Bit-Slot, Stereo = 2 x 32 = 64 Bit).
 * BCLK-Frequenz = Sample-Rate x 64. */
#define BITS_PER_FRAME          64
#define BCLK_EXPECTED_HZ        (SAMPLE_RATE_HZ * BITS_PER_FRAME)   /* 6.144 MHz */

/* -------------------------------------------------------------------------
 * Signalformen
 * ---------------------------------------------------------------------- */
typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH,
    WAVE_DC,
    WAVE_COUNT
} wave_type_t;

/* Einstellungen eines Kanals (ohne Laufzeit-Phase, die haelt der TX-Task) */
typedef struct {
    wave_type_t type;        /* Signalform                                  */
    double      freq_hz;     /* Frequenz in Hz (ignoriert bei DC)           */
    double      amplitude;   /* Spitzenwert 0..8388607                      */
    double      dc_offset;   /* DC-Anteil -8388607..8388607                  */
} channel_cfg_t;

/* Globale, gemeinsam genutzte Konfiguration (TX-Task liest, Konsole schreibt) */
typedef struct {
    channel_cfg_t left;
    channel_cfg_t right;
} player_config_t;

static player_config_t g_cfg = {
    .left  = { .type = WAVE_SINE, .freq_hz = 1000.0, .amplitude = 8388607.0, .dc_offset = 0.0 },
    .right = { .type = WAVE_SINE, .freq_hz = 1500.0, .amplitude = 8388607.0, .dc_offset = 0.0 },
};

static SemaphoreHandle_t g_cfg_mutex = NULL;
static i2s_chan_handle_t tx_chan = NULL;

/* Statistik: Anzahl gesendeter Stereo-Frames (wird vom TX-Task hochgezaehlt) */
static volatile uint32_t g_frames_sent = 0;

/* Aktueller Sample-Wert pro Kanal (fuer das periodische Logging) */
static volatile int32_t g_last_left  = 0;
static volatile int32_t g_last_right = 0;

/* Logging ein-/ausschaltbar (Kommandos 'log on'/'log off') */
static volatile bool g_log_enabled = true;

/* -------------------------------------------------------------------------
 * 24-Bit-Sample (signed, -2^23 .. 2^23-1) -> 32-Bit-Wort linksbuendig packen
 * ---------------------------------------------------------------------- */
static inline int32_t i2s_pack_sample(int32_t sample)
{
    return sample << 8;   /* MSB des Samples auf Bit 31 */
}

static inline int32_t clamp_sample(double v)
{
    if (v >  MAX_AMPLITUDE) v =  MAX_AMPLITUDE;
    if (v < -MAX_AMPLITUDE) v = -MAX_AMPLITUDE;
    return (int32_t)v;
}

/* -------------------------------------------------------------------------
 * Signalgenerator: erzeugt den naechsten Sample-Wert eines Kanals.
 * Die Phase (0..2*PI) wird vom Aufrufer gehalten und fortgeschrieben, damit
 * Aenderungen der Frequenz/Ausformung klickfrei bleiben.
 * ---------------------------------------------------------------------- */
static inline int32_t gen_sample(const channel_cfg_t *cfg, double *phase)
{
    double v = 0.0;
    const double step = (2.0 * M_PI * cfg->freq_hz) / SAMPLE_RATE_HZ;

    switch (cfg->type) {
    case WAVE_SINE:
        v = sin(*phase) * cfg->amplitude;
        break;

    case WAVE_SQUARE:
        v = (sin(*phase) >= 0.0) ? cfg->amplitude : -cfg->amplitude;
        break;

    case WAVE_TRIANGLE: {
        double p = *phase / (2.0 * M_PI);            /* 0..1            */
        v = (4.0 * fabs(p - 0.5) - 1.0) * cfg->amplitude;
        break;
    }

    case WAVE_SAWTOOTH: {
        double p = *phase / (2.0 * M_PI);            /* 0..1            */
        v = (2.0 * p - 1.0) * cfg->amplitude;
        break;
    }

    case WAVE_DC:
    default:
        v = 0.0;                                     /* Pegel ueber dc_offset */
        break;
    }

    if (cfg->type != WAVE_DC) {
        *phase += step;
        if (*phase > 2.0 * M_PI) *phase -= 2.0 * M_PI;
    }

    v += cfg->dc_offset;
    return clamp_sample(v);
}

/* -------------------------------------------------------------------------
 * Fuellt den DMA-Puffer mit Stereo-Frames.
 * Snapshot der Kanal-Einstellungen unter Mutex -> klickfreie Uebernahme.
 * ---------------------------------------------------------------------- */
static void fill_buffer(int32_t *buf, uint32_t num_frames,
                        double *phase_l, double *phase_r)
{
    channel_cfg_t l, r;

    xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
    l = g_cfg.left;
    r = g_cfg.right;
    xSemaphoreGive(g_cfg_mutex);

    for (uint32_t i = 0; i < num_frames; i++) {
        int32_t left  = gen_sample(&l, phase_l);
        int32_t right = gen_sample(&r, phase_r);

        buf[2 * i]     = i2s_pack_sample(left);
        buf[2 * i + 1] = i2s_pack_sample(right);

        g_last_left  = left;
        g_last_right = right;
    }
}

/* -------------------------------------------------------------------------
 * TX-Task: fuellt den DMA-Puffer kontinuierlich mit Stereo-Samples.
 * Im Slave-Modus muss der Strom kontinuierlich laufen (jeder WS-Frame
 * erwartet ein Sample). "Pausen" = Amplitude/DC auf 0 setzen.
 * ---------------------------------------------------------------------- */
static void i2s_write_task(void *args)
{
    int32_t *buf = heap_caps_malloc(BUFFER_SIZE_BYTES, MALLOC_CAP_INTERNAL);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Speicher fuer TX-Puffer konnte nicht alloziert werden");
        vTaskDelete(NULL);
        return;
    }

    double phase_l = 0.0;
    double phase_r = 0.0;
    size_t bytes_written = 0;
    bool was_underflow = false;

    fill_buffer(buf, FRAMES_PER_BUFFER, &phase_l, &phase_r);

    /* DMA-Puffer vorladen */
    size_t preloaded = BUFFER_SIZE_BYTES;
    while (preloaded == BUFFER_SIZE_BYTES) {
        i2s_channel_preload_data(tx_chan, buf, BUFFER_SIZE_BYTES, &preloaded);
    }

    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_LOGI(TAG, "TX gestartet: Stereo, 24 Bit, %d Hz (Slave)", (int)SAMPLE_RATE_HZ);

    while (1) {
        fill_buffer(buf, FRAMES_PER_BUFFER, &phase_l, &phase_r);

        if (i2s_channel_write(tx_chan, buf, BUFFER_SIZE_BYTES,
                              &bytes_written, pdMS_TO_TICKS(200)) == ESP_OK) {
            g_frames_sent += (uint32_t)(bytes_written / BYTES_PER_FRAME);
            if (was_underflow) {
                ESP_LOGI(TAG, "TX laeuft wieder normal (Master-Takt liegt an)");
                was_underflow = false;
            }
        } else {
            if (!was_underflow) {
                ESP_LOGW(TAG, "TX-Timeout: kein Master-Takt (BCLK/WS) erkannt");
                was_underflow = true;
            }
        }
    }

    heap_caps_free(buf);
    vTaskDelete(NULL);
}


/* -------------------------------------------------------------------------
 * I2S-Kanal initialisieren (Standard-Modus, Slave, Stereo, 24 Bit)
 * ---------------------------------------------------------------------- */
static void i2s_init_slave(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE_HZ,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            /*
             * WICHTIG bei 24 Bit: mclk_multiple muss Vielfaches von 3 sein
             * (z. B. 384), sonst schlaegt die Initialisierung fehl.
             * Im Slave-Modus ist MCLK selbst ungenutzt.
             */
            .mclk_multiple  = I2S_MCLK_MULTIPLE_384,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   /* im Slave-Modus nicht benoetigt */
            .bclk = I2S_BCLK_IO,       /* Eingang */
            .ws   = I2S_WS_IO,         /* Eingang */
            .dout = I2S_DOUT_IO,       /* Ausgang */
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_LOGI(TAG, "I2S-Kanal initialisiert (Slave, Philips, Stereo, 24 Bit)");
}

/* -------------------------------------------------------------------------
 * Hilfsfunktionen fuer die serielle Konsole
 * ---------------------------------------------------------------------- */
static const char *wave_name(wave_type_t t)
{
    switch (t) {
    case WAVE_SINE:      return "sinus";
    case WAVE_SQUARE:    return "rechteck";
    case WAVE_TRIANGLE:  return "dreieck";
    case WAVE_SAWTOOTH:  return "saegezahn";
    case WAVE_DC:        return "dc";
    default:             return "unbekannt";
    }
}

static channel_cfg_t *channel_ptr(char lr)
{
    return (lr == 'r' || lr == 'R') ? &g_cfg.right : &g_cfg.left;
}



/* -------------------------------------------------------------------------
 * Periodisches Status-Logging:
 *   - gemessene Abtastrate (= WS-Frequenz) und daraus abgeleitete BCLK-Frequenz
 *   - Bewertung, ob WS und BCLK anliegen und zum Sollwert passen
 *   - aktuelle Sample-Werte (L/R)
 *   - aktive Einstellungen (Form, Frequenz, Amplitude, DC)
 * Mit 'log off' wird das Logging deaktiviert, mit 'log on' wieder aktiviert.
 * ---------------------------------------------------------------------- */
static void status_task(void *args)
{
    uint32_t last_frames = 0;
    int64_t  last_us     = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        uint32_t frames = g_frames_sent;
        int64_t  now    = esp_timer_get_time();
        int64_t  dt_us  = now - last_us;

        /* Rate (WS-Frequenz) und BCLK-Frequenz aus den gesendeten Frames ableiten */
        double ws_hz   = 0.0;
        double bclk_hz = 0.0;
        if (dt_us > 0 && frames >= last_frames) {
            ws_hz   = (double)(frames - last_frames) * 1000000.0 / (double)dt_us;
            bclk_hz = ws_hz * BITS_PER_FRAME;
        }

        /* Bewertung von WS und BCLK gegen den Sollwert */
        const double tol = 0.02;   /* 2 % Toleranz */
        const char *ws_state, *bclk_state;
        if (ws_hz < 100.0) {
            ws_state  = "FEHLT";
            bclk_state = "FEHLT";
        } else if (fabs(ws_hz - (double)SAMPLE_RATE_HZ) / (double)SAMPLE_RATE_HZ > tol) {
            ws_state  = "ABWEICHUNG";
            bclk_state = "ABWEICHUNG";
        } else {
            ws_state  = "OK";
            bclk_state = "OK";
        }

        if (g_log_enabled) {
            ESP_LOGI(TAG, "WS=%.0f Hz (%s, soll %d) | BCLK=%.0f Hz (%s, soll %d)",
                     ws_hz, ws_state, (int)SAMPLE_RATE_HZ,
                     bclk_hz, bclk_state, (int)BCLK_EXPECTED_HZ);
            ESP_LOGI(TAG, "Sample L=%+d  R=%+d",
                     (int)g_last_left, (int)g_last_right);

            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            ESP_LOGI(TAG, "L: %s %.0f Hz amp=%.0f dc=%.0f | R: %s %.0f Hz amp=%.0f dc=%.0f",
                     wave_name(g_cfg.left.type),  g_cfg.left.freq_hz,
                     g_cfg.left.amplitude,  g_cfg.left.dc_offset,
                     wave_name(g_cfg.right.type), g_cfg.right.freq_hz,
                     g_cfg.right.amplitude, g_cfg.right.dc_offset);
            xSemaphoreGive(g_cfg_mutex);
        }

        last_frames = frames;
        last_us     = now;
    }
}


/* -------------------------------------------------------------------------
 * Serielle Konsole: nimmt Kommandos entgegen und aendert die Signalparameter.
 * ---------------------------------------------------------------------- */
static void console_task(void *args)
{
    char line[64];

    printf("\n=== I2S Sample Player Konsole ===\n");
    printf("Kommandos:\n");
    printf("  help                                   - Hilfe\n");
    printf("  status                                 - Einstellungen anzeigen\n");
    printf("  wave <l|r> <sin|square|tri|saw|dc>     - Signalform\n");
    printf("  freq <l|r> <Hz>                        - Frequenz (0.1 .. 48000)\n");
    printf("  amp  <l|r> <0..8388607>                - Amplitude\n");
    printf("  dc   <l|r> <-8388607..8388607>         - DC-Offset\n");
    printf("  off                                   - beide Kanaele stumm (amp=0)\n");
    printf("  log <on|off>                          - Logging ein/aus\n");
    printf("==================================\n");

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Zeilenende entfernen */
        line[strcspn(line, "\r\n")] = '\0';

        char *cmd = line;
        while (*cmd && isspace((unsigned char)*cmd)) cmd++;
        if (*cmd == '\0') continue;

        char *tok1 = strtok(cmd, " \t");
        char *tok2 = strtok(NULL, " \t");
        char *tok3 = strtok(NULL, " \t");

        if (strcmp(tok1, "help") == 0) {
            printf("wave <l|r> <sin|square|tri|saw|dc>\n");
            printf("freq <l|r> <Hz>\n");
            printf("amp  <l|r> <0..8388607>\n");
            printf("dc   <l|r> <-8388607..8388607>\n");
            printf("off   (Stille)\n");
            printf("log <on|off>\n");
            printf("status\n");
            continue;
        }

        if (strcmp(tok1, "status") == 0) {
            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            printf("L: %s %.0f Hz amp=%.0f dc=%.0f\n",
                   wave_name(g_cfg.left.type), g_cfg.left.freq_hz,
                   g_cfg.left.amplitude, g_cfg.left.dc_offset);
            printf("R: %s %.0f Hz amp=%.0f dc=%.0f\n",
                   wave_name(g_cfg.right.type), g_cfg.right.freq_hz,
                   g_cfg.right.amplitude, g_cfg.right.dc_offset);
            xSemaphoreGive(g_cfg_mutex);
            continue;
        }

        if (strcmp(tok1, "off") == 0) {
            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            g_cfg.left.amplitude  = 0.0;
            g_cfg.right.amplitude = 0.0;
            g_cfg.left.dc_offset  = 0.0;
            g_cfg.right.dc_offset = 0.0;
            xSemaphoreGive(g_cfg_mutex);
            printf("OK: beide Kanaele stumm (Amplitude=0)\n");
            continue;
        }

        if (strcmp(tok1, "log") == 0) {
            if (tok2 == NULL) {
                printf("Fehler: 'log on' oder 'log off' erwartet.\n");
            } else if (strcmp(tok2, "on") == 0) {
                g_log_enabled = true;
                printf("OK: Logging eingeschaltet\n");
            } else if (strcmp(tok2, "off") == 0) {
                g_log_enabled = false;
                printf("OK: Logging ausgeschaltet\n");
            } else {
                printf("Fehler: 'log on' oder 'log off' erwartet.\n");
            }
            continue;
        }

        if (tok2 == NULL || tok3 == NULL) {
            printf("Fehler: fehlende Parameter. 'help' fuer Hilfe.\n");
            continue;
        }

        if (tok2[1] != '\0') {
            printf("Fehler: Kanal muss 'l' oder 'r' sein.\n");
            continue;
        }

        channel_cfg_t *ch = channel_ptr(tok2[0]);

        if (strcmp(tok1, "wave") == 0) {
            wave_type_t t;
            if      (strcmp(tok3, "sin")   == 0) t = WAVE_SINE;
            else if (strcmp(tok3, "square") == 0) t = WAVE_SQUARE;
            else if (strcmp(tok3, "tri")   == 0) t = WAVE_TRIANGLE;
            else if (strcmp(tok3, "saw")   == 0) t = WAVE_SAWTOOTH;
            else if (strcmp(tok3, "dc")    == 0) t = WAVE_DC;
            else { printf("Fehler: unbekannte Signalform.\n"); continue; }

            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            ch->type = t;
            xSemaphoreGive(g_cfg_mutex);
            printf("OK: Kanal '%c' -> %s\n", tok2[0], wave_name(t));
            continue;
        }

        double val = strtod(tok3, NULL);

        if (strcmp(tok1, "freq") == 0) {
            if (val < 0.1 || val > 48000.0) {
                printf("Fehler: Frequenz muss 0.1..48000 Hz sein.\n");
                continue;
            }
            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            ch->freq_hz = val;
            xSemaphoreGive(g_cfg_mutex);
            printf("OK: Kanal '%c' Frequenz = %.1f Hz\n", tok2[0], val);
            continue;
        }

        if (strcmp(tok1, "amp") == 0) {
            if (val < 0.0 || val > (double)MAX_AMPLITUDE) {
                printf("Fehler: Amplitude muss 0..%d sein.\n", MAX_AMPLITUDE);
                continue;
            }
            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            ch->amplitude = val;
            xSemaphoreGive(g_cfg_mutex);
            printf("OK: Kanal '%c' Amplitude = %.0f\n", tok2[0], val);
            continue;
        }

        if (strcmp(tok1, "dc") == 0) {
            if (val < -(double)MAX_AMPLITUDE || val > (double)MAX_AMPLITUDE) {
                printf("Fehler: DC-Offset muss -%d..%d sein.\n", MAX_AMPLITUDE, MAX_AMPLITUDE);
                continue;
            }
            xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
            ch->dc_offset = val;
            xSemaphoreGive(g_cfg_mutex);
            printf("OK: Kanal '%c' DC-Offset = %.0f\n", tok2[0], val);
            continue;
        }

        printf("Fehler: unbekanntes Kommando '%s'. 'help' fuer Hilfe.\n", tok1);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starte I2S Sample Player (Slave)");

    g_cfg_mutex = xSemaphoreCreateMutex();
    assert(g_cfg_mutex != NULL);

    i2s_init_slave();

    xTaskCreate(i2s_write_task, "i2s_write_task", 4096, NULL, 5, NULL);
    xTaskCreate(status_task,    "status_task",    4096, NULL, 3, NULL);
    xTaskCreate(console_task,   "console_task",   4096, NULL, 4, NULL);
}
