/*
 * The image sensor models' line rate by mode: the registers that fix the
 * pixel clock and the line length each driver's table writes, see
 * ImageSensorLineTime.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

static const ImageSensorLineMode image_sensor_gc2053_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 1100, 33750, { 0x44, 0x01, 0x63, 0x40 } },
    { IMAGE_SENSOR_PART_NONE, 1100, 16875, { 0x44, 0x03, 0x63, 0x40 } },
    { IMAGE_SENSOR_PART_NONE, 1100, 42540, { 0x44, 0x01, 0x68, 0x40 },
      .frame_rate = 35430 },
    { IMAGE_SENSOR_PART_NONE, 1100, 32700, { 0x84, 0x11, 0x3c, 0x82 } },
    { IMAGE_SENSOR_PART_NONE, 1100, 17715, { 0x44, 0x03, 0x68, 0x40 } },
    { IMAGE_SENSOR_PART_NONE, 1100, 45000, { 0x44, 0x01, 0x84, 0x40 } },
};

static const ImageSensorLineTime image_sensor_gc2053_line = {
    .hts = { 0x5, 0x6 }, .vts = { 0x41, 0x42 },
    .sig = { 0xf6, 0xf7, 0xf8, 0xf9 }, .nsig = 4,
    .modes = image_sensor_gc2053_line_modes,
    .n = ARRAY_SIZE(image_sensor_gc2053_line_modes),
};

static const ImageSensorLineMode image_sensor_gc4653_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 1500, 48000, { 0xc8, 0x60, 0x00, 0x60 } },
    { IMAGE_SENSOR_PART_NONE, 1333, 54000, { 0xc8, 0x60, 0x00, 0x70 } },
    { IMAGE_SENSOR_PART_NONE, 1501, 96000, { 0xc8, 0x60, 0x55, 0x60 } },
    { IMAGE_SENSOR_PART_NONE, 1250, 72000, { 0xd0, 0x87, 0x51, 0x87 } },
    { IMAGE_SENSOR_PART_NONE, 1600, 45000, { 0xc8, 0x6c, 0x00, 0x6c } },
};

static const ImageSensorLineTime image_sensor_gc4653_line = {
    .hts = { 0x342, 0x343 },
    .sig = { 0x324, 0x326, 0x335, 0x336 }, .nsig = 4,
    .modes = image_sensor_gc4653_line_modes,
    .n = ARRAY_SIZE(image_sensor_gc4653_line_modes),
};

static const ImageSensorLineMode image_sensor_gc4023_line_modes[] = {
    { IMAGE_SENSOR_PART_T41, 1200, 37500, { 0 } },
    { IMAGE_SENSOR_PART_NONE, 1200, 45000, { 0 } },
};

static const ImageSensorLineTime image_sensor_gc4023_line = {
    .hts = { 0x342, 0x343 }, .vts = { 0x340, 0x341 },
    .sig = { 0 }, .nsig = 0,
    .modes = image_sensor_gc4023_line_modes,
    .n = ARRAY_SIZE(image_sensor_gc4023_line_modes),
};

static const ImageSensorLineMode image_sensor_gc5603_line_modes[] = {
    { IMAGE_SENSOR_PART_T41, 1192, 52500, { 0 } },
    { IMAGE_SENSOR_PART_T40, 1192, 52500, { 0 } },
    { IMAGE_SENSOR_PART_NONE, 1200, 52500, { 0 } },
};

static const ImageSensorLineTime image_sensor_gc5603_line = {
    .hts = { 0x342, 0x343 }, .vts = { 0x340, 0x341 },
    .sig = { 0 }, .nsig = 0,
    .modes = image_sensor_gc5603_line_modes,
    .n = ARRAY_SIZE(image_sensor_gc5603_line_modes),
};

static const ImageSensorLineMode image_sensor_sc2336_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 2250, 36000, { 0x53, 0x09, 0x0c, 0x1c, 0x28 } },
};

static const ImageSensorLineTime image_sensor_sc2336_line = {
    .hts = { 0x320c, 0x320d }, .vts = { 0x320e, 0x320f },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed }, .nsig = 5,
    .modes = image_sensor_sc2336_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc2336_line_modes),
};

static const ImageSensorLineMode image_sensor_sc2336p_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 0, 36000, { 0x53, 0x0b, 0x0c, 0x1c, 0x18 } },
};

static const ImageSensorLineTime image_sensor_sc2336p_line = {
    .hts = { 0x320c, 0x320d }, .vts = { 0x320e, 0x320f },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed }, .nsig = 5,
    .modes = image_sensor_sc2336p_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc2336p_line_modes),
};

static const ImageSensorLineMode image_sensor_sc4336p_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 2800, 45000, { 0x53 } },
};

static const ImageSensorLineTime image_sensor_sc4336p_line = {
    .hts = { 0x320c, 0x320d },
    .sig = { 0x36e9 }, .nsig = 1,
    .modes = image_sensor_sc4336p_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc4336p_line_modes),
};

static const ImageSensorLineMode image_sensor_sc3338_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 0, 40800, { 0x54, 0x11, 0x0d, 0x1c, 0x26 } },
    { IMAGE_SENSOR_PART_NONE, 0, 39600, { 0x53, 0x0b, 0x0d, 0x1c, 0x26 } },
    { IMAGE_SENSOR_PART_NONE, 0, 40800, { 0x54, 0x11, 0x0d, 0x2c, 0x26 } },
    { IMAGE_SENSOR_PART_NONE, 1250, 40500, { 0x54, 0x00, 0x00, 0x00, 0x00 } },
};

static const ImageSensorLineTime image_sensor_sc3338_line = {
    .hts = { 0x320c, 0x320d },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed }, .nsig = 5,
    .modes = image_sensor_sc3338_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc3338_line_modes),
};

static const ImageSensorLineMode image_sensor_sc401ai_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 0, 45000,
      { 0x23, 0x39, 0x0d, 0x1c, 0x24, 0x23, 0x39, 0x33, 0x10 } },
    { IMAGE_SENSOR_PART_NONE, 0, 45000,
      { 0x23, 0x39, 0x0d, 0x2c, 0x24, 0x23, 0x39, 0x33, 0x10 } },
    { IMAGE_SENSOR_PART_NONE, 0, 30000,
      { 0x43, 0x39, 0x0d, 0x1c, 0x24, 0x44, 0x39, 0x33, 0x10 } },
};

static const ImageSensorLineTime image_sensor_sc401ai_line = {
    .hts = { 0x320c, 0x320d },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed, 0x36f9, 0x36fa, 0x36fb,
      0x36fc }, .nsig = 9,
    .modes = image_sensor_sc401ai_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc401ai_line_modes),
};

static const ImageSensorLineMode image_sensor_sc830ai_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 2100, 67500, { 0x24, 0x0a, 0x05, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2025, 90000, { 0x00, 0x09, 0x05, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2100, 67500, { 0x53, 0x07, 0x04, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2025, 90000, { 0x53, 0x0a, 0x04, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2100, 34260, { 0x53, 0x08, 0x14, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2100, 68520, { 0x53, 0x08, 0x04, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 2400, 33750, { 0x53, 0x09, 0x04, 0x03, 0x22 } },
    { IMAGE_SENSOR_PART_NONE, 4200, 45000, { 0x53, 0x0c, 0x04, 0x03, 0x22 } },
};

static const ImageSensorLineTime image_sensor_sc830ai_line = {
    .hts = { 0x320c, 0x320d },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed }, .nsig = 5,
    .modes = image_sensor_sc830ai_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc830ai_line_modes),
};

static const ImageSensorLineMode image_sensor_sc4236_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 2500, 48000,
      { 0x02, 0x36, 0x06, 0x0e, 0x03, 0x01, 0x8a, 0x00 } },
    { IMAGE_SENSOR_PART_NONE, 2600, 30000,
      { 0x56, 0x33, 0x06, 0x0e, 0x03, 0x01, 0x9e, 0x00 } },
};

static const ImageSensorLineTime image_sensor_sc4236_line = {
    .hts = { 0x320c, 0x320d }, .vts = { 0x320e, 0x320f },
    .sig = { 0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed, 0x36f9, 0x36fa, 0x36fb },
    .nsig = 8,
    .modes = image_sensor_sc4236_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc4236_line_modes),
};

static const ImageSensorLineMode image_sensor_sc2232h_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 2080, 37500, { 0x08, 0x20 } },
    { IMAGE_SENSOR_PART_NONE, 2400, 22500, { 0x09, 0x60 } },
};

static const ImageSensorLineTime image_sensor_sc2232h_line = {
    .hts = { 0x320c, 0x320d },
    .sig = { 0x320c, 0x320d }, .nsig = 2,
    .modes = image_sensor_sc2232h_line_modes,
    .n = ARRAY_SIZE(image_sensor_sc2232h_line_modes),
};

static const ImageSensorLineMode image_sensor_imx307_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 4400, 33750, { 0x12, 0x00 } },
    { IMAGE_SENSOR_PART_NONE, 4400, 33750, { 0x02, 0x00 } },
};

static const ImageSensorLineTime image_sensor_imx307_line = {
    .hts = { 0x301d, 0x301c }, .vts = { 0x3019, 0x3018 },
    .sig = { 0x3009, 0x3007 }, .nsig = 2,
    .modes = image_sensor_imx307_line_modes,
    .n = ARRAY_SIZE(image_sensor_imx307_line_modes),
};

static const ImageSensorLineMode image_sensor_jxf23_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 1280, 33750, { 0x11, 0x14, 0x48, 0x80, 0x00 } },
    { IMAGE_SENSOR_PART_NONE, 1280, 16875, { 0x11, 0x14, 0x24, 0x80, 0x40 } },
    { IMAGE_SENSOR_PART_NONE, 1280, 33750, { 0x11, 0x14, 0x24, 0x80, 0x00 } },
    { IMAGE_SENSOR_PART_NONE, 1280, 16875, { 0x12, 0x1c, 0x36, 0x80, 0x00 } },
    { IMAGE_SENSOR_PART_NONE, 1280, 33750, { 0x11, 0x14, 0x24, 0x80, 0x08 } },
};

static const ImageSensorLineTime image_sensor_jxf23_line = {
    .hts = { 0x21, 0x20 }, .vts = { 0x23, 0x22 },
    .sig = { 0xe, 0xf, 0x10, 0x11, 0x12 }, .nsig = 5,
    .modes = image_sensor_jxf23_line_modes,
    .n = ARRAY_SIZE(image_sensor_jxf23_line_modes),
};

static const ImageSensorLineMode image_sensor_jxf22_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 1200, 33750, { 0x11, 0x00, 0x36, 0x80, 0x40 } },
};

static const ImageSensorLineTime image_sensor_jxf22_line = {
    .hts = { 0x21, 0x20 },
    .sig = { 0xe, 0xf, 0x10, 0x11, 0x12 }, .nsig = 5,
    .modes = image_sensor_jxf22_line_modes,
    .n = ARRAY_SIZE(image_sensor_jxf22_line_modes),
};

static const ImageSensorLineMode image_sensor_jxh42_line_modes[] = {
    { IMAGE_SENSOR_PART_NONE, 1600, 22500, { 0x1d, 0x09, 0x1e, 0x80, 0x40 } },
};

static const ImageSensorLineTime image_sensor_jxh42_line = {
    .hts = { 0x21, 0x20 },
    .sig = { 0xe, 0xf, 0x10, 0x11, 0x12 }, .nsig = 5,
    .modes = image_sensor_jxh42_line_modes,
    .n = ARRAY_SIZE(image_sensor_jxh42_line_modes),
};
