/*
 * A camera image sensor's I2C control interface
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SENSOR_IMAGE_SENSOR_H
#define HW_SENSOR_IMAGE_SENSOR_H

#include "hw/i2c/i2c.h"
#include "qemu/thread.h"
#include "qom/object.h"

#define TYPE_IMAGE_SENSOR "image-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(ImageSensorState, IMAGE_SENSOR)

/* The video inputs sensors can feed: VICs, or a MIPI switch's ports */
#define IMAGE_SENSOR_INPUTS 4

typedef struct ImageSensorModel ImageSensorModel;
typedef struct ImageSensorRaw ImageSensorRaw;

#define IMAGE_SENSOR_SCENE_CACHE 4

/* A scene picture scaled once: NV12 planes of w x h, of source frame serial */
typedef struct ImageSensorSceneCache {
    uint8_t *buf;
    uint64_t serial;
    uint32_t w, h;
    bool mirror, flip;
} ImageSensorSceneCache;

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
    uint32_t grey;              /* a flat grey of this luma, 0: the scene */
    uint32_t ambient_ir;        /* its IR, percent of its visible light */
    uint32_t ir;                /* the IR LEDs' at full power, percent */
    bool object;                /* a square crossing the ramp */
    bool motion;                /* the stripe and the square move */
    bool stamp;                 /* the frame number in cells, top left */
    char *ircut;                /* thingino.json's pins: one, or two */
    char *irled;                /* GPIOs, or pwmN channels */
    char *scene;                /* a YUV4MPEG2 file or FIFO, else the bars */

    int scene_fd;
    bool scene_stream;          /* a FIFO: the latest frame, else a loop */
    uint32_t scene_w, scene_h;  /* of the stream or file */
    size_t scene_fsize;         /* a frame's bytes, 4:2:0 planar */
    off_t scene_off;            /* a file's first frame */
    uint64_t scene_frames;      /* a file's frames */
    uint8_t *scene_cur;         /* the frame in use, or NULL */
    uint8_t *scene_acc;         /* a stream's frame filling up (its thread) */
    uint8_t *scene_ready;       /* the latest complete one, under scene_lock */
    uint32_t scene_ready_w, scene_ready_h;
    bool scene_ready_new;
    QemuMutex scene_lock;
    QemuThread scene_thread;
    bool scene_thread_up;
    bool scene_quit;
    uint32_t scene_cur_w, scene_cur_h;
    uint64_t scene_cur_idx;     /* a file's frame in scene_cur */
    uint64_t scene_polled;      /* the frame a stream was last read at */
    bool scene_polled_valid;
    int scene_state;            /* a stream's parser */
    size_t scene_fill;          /* bytes of the line or the body so far */
    char scene_line[256];
    uint64_t scene_serial;      /* counts the frames scene_cur has held */
    ImageSensorSceneCache scene_cache[IMAGE_SENSOR_SCENE_CACHE];
    unsigned scene_cache_next;
    /* the exposure tables last made, and the light (vis, ir, fuse) of them */
    double expose_key[3];
    uint8_t expose_lut[256], expose_clut[256];
    bool expose_valid;

    uint8_t regs[65536];        /* by address, or page and address */
    unsigned hold;              /* image_sensor_hold()s in force */
    GArray *held;               /* the writes they keep back, in order */
    uint16_t pointer;           /* the register the next byte goes to */
    uint8_t addr_bytes;         /* address bytes taken in this write */
    uint8_t page;               /* GalaxyCore's 0xfe */
    uint32_t traced_lines;      /* the exposure as last traced */
    uint32_t traced_gain;
    bool traced_filter;         /* the IR state as last traced */
    long traced_leds;
    long traced_light;

    ImageSensorExposureSel exposure;    /* what render puts out */
    bool untinted;                      /* without the light's tint */
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

/*
 * Register writes over I2C wait while the sensor is held, reads see them,
 * and image_sensor_release() of the last hold stores them in order: a frame
 * drawn between the two sees one set of registers however the guest runs
 * meanwhile. Under the BQL, as the I2C bus is.
 */
void image_sensor_hold(ImageSensorState *s);
void image_sensor_release(ImageSensorState *s);

/* Out of standby: sending frames */
bool image_sensor_streaming(ImageSensorState *s);

/*
 * Lines a second the sensor sends in the mode the driver of @part (the T21,
 * T23, else the T31) starts in: total lines times the nominal frame rate;
 * blanking stretches for a lower rate, the line rate stays. 0 for a model
 * whose mode is not known
 */
uint32_t image_sensor_line_rate(ImageSensorState *s, ImageSensorRawPart part);

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
 * The raw signal of a grey in the scene's light, as factors on the linear
 * light of the picture image_sensor_render_untinted() renders (R, G, B).
 * False, @f untouched, where image_sensor_raw_factors() is.
 */
bool image_sensor_raw_response(ImageSensorState *s, ImageSensorRawPart part,
                               double f[3]);

/* @n bytes of @src through the 256-byte table @lut into @dst */
void image_sensor_lookup(uint8_t *dst, const uint8_t *src, size_t n,
                         const uint8_t *lut);

/*
 * Frame @frame of one exposure with the light's tint left out, which the
 * 8-bit chroma of a dim picture cannot hold
 */
bool image_sensor_tinted(const ImageSensorState *s);
void image_sensor_render_untinted(ImageSensorState *s, uint64_t frame,
                                  uint32_t w, uint32_t h, uint8_t *nv12,
                                  ImageSensorExposureSel exposure);

/*
 * The same as the sensor sends it, before any ISP: one colour a pixel in
 * an RGGB Bayer pattern, each full scale at 0xffff
 */
void image_sensor_render_bayer(ImageSensorState *s, uint64_t frame,
                               uint32_t w, uint32_t h, uint16_t *raw);

#endif
