/*
 * A camera image sensor's I2C control interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SENSOR_IMAGE_SENSOR_H
#define HW_SENSOR_IMAGE_SENSOR_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IMAGE_SENSOR "image-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(ImageSensorState, IMAGE_SENSOR)

/* The video inputs sensors can feed: VICs, or a MIPI switch's ports */
#define IMAGE_SENSOR_INPUTS 4

typedef struct ImageSensorModel ImageSensorModel;
typedef struct ImageSensorRaw ImageSensorRaw;

/*
 * A grey's raw R and B against its G, in 1/256ths, for a model with no
 * response of its own: an OS04D10's in daylight, and for the Apical parts
 * (T10, T20, T30) a JXF23's in a room lit by a white LED
 */
#define IMAGE_SENSOR_RAW_R 144
#define IMAGE_SENSOR_RAW_B 105
#define IMAGE_SENSOR_RAW_APICAL_R 129
#define IMAGE_SENSOR_RAW_APICAL_B 142

/*
 * The parts whose tuning files give a model its raw response; the same
 * sensor's differs from part to part
 */
typedef enum ImageSensorRawPart {
    IMAGE_SENSOR_PART_NONE,
    IMAGE_SENSOR_PART_T21,
    IMAGE_SENSOR_PART_T23,
    IMAGE_SENSOR_PART_T31,
    IMAGE_SENSOR_PART_T32,
    IMAGE_SENSOR_PART_T40,
    IMAGE_SENSOR_PART_T41,
    IMAGE_SENSOR_PART_T33,
    IMAGE_SENSOR_PART_T32PRO,
    IMAGE_SENSOR_PART_APICAL,       /* the T10, T20 and T30's files */
} ImageSensorRawPart;

/* Which of a WDR sensor's exposures a picture shows */
typedef enum ImageSensorExposureSel {
    IMAGE_SENSOR_FUSED,         /* both, as the ISP fuses them */
    IMAGE_SENSOR_LONG,
    IMAGE_SENSOR_SHORT,
} ImageSensorExposureSel;

struct ImageSensorState {
    I2CSlave parent_obj;

    char *model_name;
    const ImageSensorModel *model;
    uint32_t fps;
    uint32_t input;             /* the video input it feeds */
    bool noise;                 /* over the scene's luma, each frame anew */
    bool sensor_noise;          /* per Bayer sample, as a sensor's */
    double read_noise;          /* its sigma at 1x gain, 0: measured */
    double shot_noise;          /* its variance per level, 0: measured */
    uint32_t light;             /* the scene's brightness, percent */
    uint32_t light_to;          /* where a ramp from light goes */
    uint32_t light_secs;        /* in that many seconds of frames */
    uint32_t kelvin;            /* the light's colour, 0: as rendered */
    uint32_t raw_r;             /* a grey's raw R/G and B/G, 1/256ths, */
    uint32_t raw_b;             /* whatever the light: 0, the model's */
    uint32_t ambient_ir;        /* its IR, percent of its visible light */
    uint32_t ir;                /* the IR LEDs' at full power, percent */
    bool object;                /* a square crossing the ramp */
    bool motion;                /* the stripe and the square move */
    bool stamp;                 /* the frame number in cells, top left */
    char *ircut;                /* thingino.json's pins: one, or two */
    char *irled;                /* GPIOs, or pwmN channels */

    uint8_t regs[65536];        /* by address, or page and address */
    uint16_t pointer;           /* the register the next byte goes to */
    uint8_t addr_bytes;         /* address bytes taken in this write */
    uint8_t page;               /* GalaxyCore's 0xfe */
    uint32_t traced_lines;      /* the exposure as last traced */
    uint32_t traced_gain;
    bool traced_filter;         /* the IR state as last traced */
    long traced_leds;
    long traced_light;

    ImageSensorExposureSel exposure;    /* what render puts out */
    uint64_t frame;             /* the last frame rendered */
    uint64_t ramp_frame;        /* the frame a ramp started at */
    uint32_t ramp_from;
    bool ramping;
    bool filter_in;             /* the IR-cut filter blocks IR */
    Object *gpio;               /* the SoC's, for the pins' levels */
    Object *pwm;
    uint8_t ircut_pins;         /* 1: its level, 2: pulses, latched */
    int16_t ircut_pin[2];
    bool ircut_low[2];          /* active low */
    uint8_t irleds;
    int16_t irled_pin[4];       /* a GPIO, or -1 - a PWM channel */
    bool irled_low[4];
};

/* The sensor on video input @input, or NULL */
ImageSensorState *image_sensor_find(unsigned input);

/* Out of standby: sending frames */
bool image_sensor_streaming(ImageSensorState *s);

/*
 * Lines a second the sensor sends in the mode its T31 driver starts in
 * (total lines times the nominal frame rate; blanking stretches for a lower
 * rate, the line rate stays): a GC2053's at 1080p30 for a model whose mode
 * is not known
 */
#define IMAGE_SENSOR_LINE_RATE_REF (1418 * 30)
uint32_t image_sensor_line_rate(ImageSensorState *s);

/* In its two-exposure WDR mode */
bool image_sensor_wdr(ImageSensorState *s);

/*
 * Frame @frame of the scene as the ISP outputs it: NV12, @w x @h, packed;
 * in WDR mode its two exposures fused
 */
void image_sensor_render(ImageSensorState *s, uint64_t frame, uint32_t w,
                         uint32_t h, uint8_t *nv12);

/* The same, of one exposure alone in WDR mode */
void image_sensor_render_exposure(ImageSensorState *s, uint64_t frame,
                                  uint32_t w, uint32_t h, uint8_t *nv12,
                                  ImageSensorExposureSel exposure);

/* The same, its luma at @luma and chroma at @uv, both @w apart */
void image_sensor_render_planes(ImageSensorState *s, uint64_t frame,
                                uint32_t w, uint32_t h, uint8_t *luma,
                                uint8_t *uv);

/*
 * The factors that take the linear light of the picture @s renders to the
 * raw signal its sensor gives for it on @part (R, G, B; G is 1): the
 * sensor's raw response in the scene's light, with the tint the light puts
 * on the picture taken out. False, @f untouched, for a model with no
 * response of its own on @part and no raw-r or raw-b: the caller's default.
 */
bool image_sensor_raw_factors(ImageSensorState *s, ImageSensorRawPart part,
                              double f[3]);

/*
 * The same as the sensor sends it, before any ISP: one colour a pixel in
 * an RGGB Bayer pattern, each full scale at 0xffff
 */
void image_sensor_render_bayer(ImageSensorState *s, uint64_t frame,
                               uint32_t w, uint32_t h, uint16_t *raw);

#endif
