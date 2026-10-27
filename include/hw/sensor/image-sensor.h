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

    uint8_t regs[65536];        /* by address, or page and address */
    uint16_t pointer;           /* the register the next byte goes to */
    uint8_t addr_bytes;         /* address bytes taken in this write */
    uint8_t page;               /* GalaxyCore's 0xfe */
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
