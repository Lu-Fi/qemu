/*
 * The image sensor models' raw colour response: a grey's raw R/G and B/G
 * (1/1024ths) in light of the temperatures of image_sensor_raw_kelvin[], as
 * the AWB calibration in the sensor's stock T31 tuning file has it.
 *
 * The file holds, in each of its day and night parameter blocks, a 15x15
 * mesh over the AWB's calibrated ratios rg = 256 * (R/G) * wb_static[0] / 1024
 * and bg = 256 * (B/G) * wb_static[1] / 1024 (the mesh's axes rg_pos and
 * bg_pos, wb_static the sensor's static gains in 1/1024ths): the mesh gives
 * each cell its colour temperature as 1e6 / K (color_temp_mesh, word 0x15b0
 * of the block) and the weight a grey there gets, 0..256 (rgbg_weight,
 * 0x122c), whose ridge is the locus of the sensor's greys. The AWB's gains
 * settle on the inverse of a grey's R/G and B/G, so the model puts a grey
 * where the ridge is for the light's temperature. The values here are the
 * day block's ridge at each temperature (a weighted mean along the cells
 * of equal 1e6 / K, smoothed over 12 mired), as raw R/G and B/G; a light
 * outside the range the calibration covers holds the end's value. Models
 * whose files carry the same calibration share a table.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

static const uint16_t image_sensor_raw_kelvin[IMAGE_SENSOR_RAW_POINTS] = {
    2500, 3000, 3500, 4000, 4500, 5000, 5500, 6500, 8000, 10000,
};

/* gc1084 */
static const ImageSensorRaw image_sensor_raw_gc1084 = {
    .r = { 914, 844, 767, 704, 650, 612, 587, 551, 496, 465 },
    .b = { 335, 377, 406, 455, 502, 539, 565, 607, 662, 707 },
};

/* gc2053, gc2063 */
static const ImageSensorRaw image_sensor_raw_gc2053 = {
    .r = { 882, 780, 747, 699, 628, 569, 536, 495, 458, 428 },
    .b = { 313, 357, 391, 427, 474, 514, 550, 610, 678, 736 },
};

/* gc2083 */
static const ImageSensorRaw image_sensor_raw_gc2083 = {
    .r = { 982, 839, 775, 719, 649, 560, 513, 475, 439, 411 },
    .b = { 334, 362, 402, 431, 489, 587, 647, 712, 782, 854 },
};

/* gc3003a */
static const ImageSensorRaw image_sensor_raw_gc3003a = {
    .r = { 969, 942, 839, 762, 700, 648, 601, 533, 530, 530 },
    .b = { 473, 480, 518, 548, 590, 629, 671, 730, 733, 733 },
};

/* gc4023 */
static const ImageSensorRaw image_sensor_raw_gc4023 = {
    .r = { 1070, 923, 842, 769, 701, 646, 605, 567, 522, 484 },
    .b = { 430, 468, 493, 532, 591, 638, 676, 735, 790, 831 },
};

/* gc4653 */
static const ImageSensorRaw image_sensor_raw_gc4653 = {
    .r = { 955, 806, 738, 673, 606, 565, 527, 481, 449, 416 },
    .b = { 265, 324, 371, 414, 459, 509, 542, 599, 670, 736 },
};

/* imx307, imx327, os05a10 */
static const ImageSensorRaw image_sensor_raw_imx307 = {
    .r = { 1061, 903, 793, 718, 671, 633, 602, 561, 526, 503 },
    .b = { 306, 347, 379, 417, 458, 495, 530, 587, 645, 701 },
};

/* jxf23, jxh62 */
static const ImageSensorRaw image_sensor_raw_jxf23 = {
    .r = { 971, 824, 758, 693, 630, 582, 543, 494, 460, 441 },
    .b = { 397, 433, 457, 493, 538, 578, 606, 654, 711, 761 },
};

/* jxf37 */
static const ImageSensorRaw image_sensor_raw_jxf37 = {
    .r = { 901, 782, 730, 667, 607, 556, 533, 499, 458, 432 },
    .b = { 422, 473, 511, 550, 586, 610, 645, 699, 753, 805 },
};

/* jxk04 */
static const ImageSensorRaw image_sensor_raw_jxk04 = {
    .r = { 944, 831, 775, 721, 662, 620, 601, 575, 534, 500 },
    .b = { 528, 539, 572, 602, 636, 665, 692, 738, 789, 837 },
};

/* jxq03 */
static const ImageSensorRaw image_sensor_raw_jxq03 = {
    .r = { 851, 774, 743, 703, 649, 605, 582, 556, 525, 504 },
    .b = { 491, 529, 549, 585, 633, 669, 702, 751, 802, 852 },
};

/* jxq03p */
static const ImageSensorRaw image_sensor_raw_jxq03p = {
    .r = { 901, 839, 809, 779, 750, 726, 712, 697, 688, 673 },
    .b = { 527, 559, 581, 606, 644, 677, 708, 756, 801, 846 },
};

/* mis2008 */
static const ImageSensorRaw image_sensor_raw_mis2008 = {
    .r = { 1000, 856, 707, 615, 558, 527, 502, 463, 423, 392 },
    .b = { 249, 296, 346, 409, 471, 527, 574, 638, 710, 777 },
};

/* os02g10 */
static const ImageSensorRaw image_sensor_raw_os02g10 = {
    .r = { 1061, 903, 793, 718, 671, 633, 602, 561, 528, 506 },
    .b = { 306, 347, 379, 417, 458, 495, 530, 587, 645, 702 },
};

/* os03b10 */
static const ImageSensorRaw image_sensor_raw_os03b10 = {
    .r = { 878, 835, 729, 647, 594, 558, 529, 482, 428, 427 },
    .b = { 328, 351, 423, 483, 535, 575, 596, 642, 724, 727 },
};

/* sc2232h */
static const ImageSensorRaw image_sensor_raw_sc2232h = {
    .r = { 930, 817, 753, 697, 651, 613, 594, 566, 549, 533 },
    .b = { 400, 438, 461, 496, 536, 568, 599, 638, 666, 692 },
};

/* sc223a */
static const ImageSensorRaw image_sensor_raw_sc223a = {
    .r = { 869, 750, 703, 650, 588, 535, 503, 462, 429, 403 },
    .b = { 312, 358, 380, 423, 474, 512, 551, 611, 676, 733 },
};

/* sc2310 */
static const ImageSensorRaw image_sensor_raw_sc2310 = {
    .r = { 931, 811, 760, 699, 625, 574, 553, 520, 484, 460 },
    .b = { 391, 430, 465, 506, 555, 595, 623, 651, 689, 734 },
};

/* sc2331 */
static const ImageSensorRaw image_sensor_raw_sc2331 = {
    .r = { 838, 754, 666, 618, 582, 546, 504, 460, 450, 445 },
    .b = { 298, 322, 365, 391, 413, 453, 525, 612, 648, 670 },
};

/* sc2332 */
static const ImageSensorRaw image_sensor_raw_sc2332 = {
    .r = { 1031, 934, 843, 783, 732, 688, 657, 612, 563, 528 },
    .b = { 401, 426, 466, 490, 544, 590, 626, 682, 752, 802 },
};

/* sc2336 */
static const ImageSensorRaw image_sensor_raw_sc2336 = {
    .r = { 988, 894, 793, 714, 659, 608, 565, 524, 508, 499 },
    .b = { 249, 273, 311, 342, 379, 428, 489, 565, 623, 666 },
};

/* sc2336p */
static const ImageSensorRaw image_sensor_raw_sc2336p = {
    .r = { 920, 825, 712, 666, 639, 601, 558, 508, 480, 461 },
    .b = { 310, 339, 388, 414, 441, 487, 551, 629, 692, 743 },
};

/* sc301iot */
static const ImageSensorRaw image_sensor_raw_sc301iot = {
    .r = { 981, 849, 778, 721, 655, 602, 571, 524, 483, 448 },
    .b = { 293, 339, 371, 402, 452, 515, 543, 589, 662, 734 },
};

/* sc3235 */
static const ImageSensorRaw image_sensor_raw_sc3235 = {
    .r = { 892, 797, 766, 722, 667, 624, 596, 560, 528, 500 },
    .b = { 441, 479, 510, 543, 581, 614, 640, 685, 739, 792 },
};

/* sc3335 */
static const ImageSensorRaw image_sensor_raw_sc3335 = {
    .r = { 868, 771, 748, 700, 635, 588, 561, 527, 494, 468 },
    .b = { 471, 510, 555, 596, 638, 674, 709, 760, 814, 872 },
};

/* sc3336 */
static const ImageSensorRaw image_sensor_raw_sc3336 = {
    .r = { 1096, 929, 882, 811, 722, 654, 621, 577, 533, 499 },
    .b = { 346, 397, 453, 504, 566, 616, 660, 726, 803, 865 },
};

/* sc3338 */
static const ImageSensorRaw image_sensor_raw_sc3338 = {
    .r = { 983, 983, 909, 811, 735, 678, 637, 577, 547, 547 },
    .b = { 364, 364, 426, 492, 543, 582, 610, 653, 692, 692 },
};

/* sc401ai */
static const ImageSensorRaw image_sensor_raw_sc401ai = {
    .r = { 989, 815, 718, 645, 594, 559, 526, 481, 444, 414 },
    .b = { 270, 323, 357, 400, 456, 505, 540, 595, 659, 721 },
};

/* sc4236 */
static const ImageSensorRaw image_sensor_raw_sc4236 = {
    .r = { 1088, 954, 874, 806, 758, 718, 701, 669, 629, 597 },
    .b = { 497, 519, 522, 538, 582, 615, 650, 691, 728, 767 },
};

/* sc4336 */
static const ImageSensorRaw image_sensor_raw_sc4336 = {
    .r = { 1029, 1010, 907, 834, 772, 715, 664, 608, 584, 582 },
    .b = { 422, 428, 474, 508, 530, 568, 640, 730, 776, 781 },
};

/* sc4336p */
static const ImageSensorRaw image_sensor_raw_sc4336p = {
    .r = { 875, 805, 743, 693, 655, 617, 578, 535, 520, 513 },
    .b = { 355, 372, 404, 437, 463, 500, 564, 637, 672, 695 },
};

/* sc500ai */
static const ImageSensorRaw image_sensor_raw_sc500ai = {
    .r = { 984, 811, 724, 649, 593, 550, 520, 479, 441, 409 },
    .b = { 269, 309, 337, 382, 443, 490, 525, 580, 646, 711 },
};

/* sc5235 */
static const ImageSensorRaw image_sensor_raw_sc5235 = {
    .r = { 1015, 875, 791, 719, 654, 606, 568, 522, 484, 466 },
    .b = { 391, 426, 458, 491, 541, 584, 615, 665, 717, 763 },
};

static const struct {
    const char *model;
    const ImageSensorRaw *raw;
} image_sensor_raw_models[] = {
    { "gc1084", &image_sensor_raw_gc1084 },
    { "gc2053", &image_sensor_raw_gc2053 },
    { "gc2063", &image_sensor_raw_gc2053 },
    { "gc2083", &image_sensor_raw_gc2083 },
    { "gc3003a", &image_sensor_raw_gc3003a },
    { "gc4023", &image_sensor_raw_gc4023 },
    { "gc4653", &image_sensor_raw_gc4653 },
    { "imx307", &image_sensor_raw_imx307 },
    { "imx327", &image_sensor_raw_imx307 },
    { "jxf23", &image_sensor_raw_jxf23 },
    { "jxf37", &image_sensor_raw_jxf37 },
    { "jxh62", &image_sensor_raw_jxf23 },
    { "jxk04", &image_sensor_raw_jxk04 },
    { "jxq03", &image_sensor_raw_jxq03 },
    { "jxq03p", &image_sensor_raw_jxq03p },
    { "mis2008", &image_sensor_raw_mis2008 },
    { "os02g10", &image_sensor_raw_os02g10 },
    { "os03b10", &image_sensor_raw_os03b10 },
    { "os05a10", &image_sensor_raw_imx307 },
    { "sc2232h", &image_sensor_raw_sc2232h },
    { "sc223a", &image_sensor_raw_sc223a },
    { "sc2310", &image_sensor_raw_sc2310 },
    { "sc2331", &image_sensor_raw_sc2331 },
    { "sc2332", &image_sensor_raw_sc2332 },
    { "sc2336", &image_sensor_raw_sc2336 },
    { "sc2336p", &image_sensor_raw_sc2336p },
    { "sc301iot", &image_sensor_raw_sc301iot },
    { "sc3235", &image_sensor_raw_sc3235 },
    { "sc3335", &image_sensor_raw_sc3335 },
    { "sc3336", &image_sensor_raw_sc3336 },
    { "sc3338", &image_sensor_raw_sc3338 },
    { "sc401ai", &image_sensor_raw_sc401ai },
    { "sc4236", &image_sensor_raw_sc4236 },
    { "sc4336", &image_sensor_raw_sc4336 },
    { "sc4336p", &image_sensor_raw_sc4336p },
    { "sc500ai", &image_sensor_raw_sc500ai },
    { "sc5235", &image_sensor_raw_sc5235 },
};
