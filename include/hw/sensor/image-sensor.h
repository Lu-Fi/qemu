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

struct ImageSensorState {
    I2CSlave parent_obj;

    char *model_name;
    const ImageSensorModel *model;
    uint32_t fps;
    uint32_t input;             /* the video input it feeds */
    bool noise;                 /* over the scene's luma, each frame anew */
    uint32_t light;             /* the scene's brightness, percent */
    uint32_t light_to;          /* where a ramp from light goes */
    uint32_t light_secs;        /* in that many seconds of frames */
    uint32_t kelvin;            /* the light's colour, 0: as rendered */
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

/* Frame @frame of the scene as the ISP outputs it: NV12, @w x @h, packed */
void image_sensor_render(ImageSensorState *s, uint64_t frame, uint32_t w,
                         uint32_t h, uint8_t *nv12);

/* The same, its luma at @luma and chroma at @uv, both @w apart */
void image_sensor_render_planes(ImageSensorState *s, uint64_t frame,
                                uint32_t w, uint32_t h, uint8_t *luma,
                                uint8_t *uv);

#endif
