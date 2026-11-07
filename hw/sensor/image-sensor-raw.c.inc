/*
 * The image sensor models' raw colour response: a grey's raw R/G and B/G
 * (1/1024ths) in light of the temperatures of image_sensor_raw_kelvin[], as
 * the AWB calibration in the sensor's stock tuning file for a part has it
 * (the T21, T23, T31, T32, T40 and T41, and the Apical files of the T10,
 * T20 and T30; a part keeps its own table for a model, see
 * image_sensor_raw_factors()).
 *
 * A T21 to T41 file holds a 15x15 mesh over the AWB's calibrated ratios
 * rg = 256 * (R/G) * wb_static[0] / 1024 and bg = 256 * (B/G) * wb_static[1]
 * / 1024 (the mesh's axes rg_pos and bg_pos, wb_static the sensor's static
 * gains in 1/1024ths): the mesh gives each cell its colour temperature as
 * 1e6 / K and the weight a grey there gets, 0..256, whose ridge is the locus
 * of the sensor's greys. The AWB's gains settle on the inverse of a grey's
 * R/G and B/G, so the model puts a grey where the ridge is for the light's
 * temperature. The values here are the day parameters' ridge at each
 * temperature (a weighted mean along the cells of equal 1e6 / K, smoothed
 * over 12 mired), as raw R/G and B/G; a light outside the range the
 * calibration covers holds the end's value. Models whose files carry the
 * same calibration share a table.
 *
 * The T10, T20 and T30 run the Apical firmware, whose tuning file has, beside
 * a mesh of its own, a colour temperature curve: STATIC_WB (the sensor's
 * static gains, 1/256ths), COLOR_TEMP (mired, 7 to 10 points) and, at each,
 * the coordinate of a grey in the AWB's plane, CT_RG_POS_CALC and
 * CT_BG_POS_CALC. A zone's coordinates are rg = R/G * STATIC_WB R and bg =
 * B/G * STATIC_WB B, so the curve divided by the static gains is the same
 * locus (the mesh's weights peak along it, and its colour temperatures match
 * COLOR_TEMP there). The image_sensor_raw_ap_* tables read that curve at the
 * same temperatures; a sensor whose file thingino ships differently from the
 * SDK's gets the mean of the copies. The settled AWB of a bench T30X (SC4236)
 * and T10L (OV9712) sits within 1% and 4% of their curves at the colour
 * temperature the firmware reports; a T20X's JXF23 is 10% below its curve in
 * R and within 3% of the T31 table, so that model's Apical entry is its T31
 * table.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

static const uint16_t image_sensor_raw_kelvin[IMAGE_SENSOR_RAW_POINTS] = {
    2500, 3000, 3500, 4000, 4500, 5000, 5500, 6500, 8000, 10000,
};

/* cv2003 t32 */
static const ImageSensorRaw image_sensor_raw_cv2003_t32 = {
    .r = { 879, 749, 680, 627, 584, 549, 532, 507, 475, 446 },
    .b = { 275, 323, 352, 400, 450, 488, 518, 558, 601, 638 },
};

/* cv8001 t32 */
static const ImageSensorRaw image_sensor_raw_cv8001_t32 = {
    .r = { 849, 747, 709, 651, 587, 536, 508, 464, 427, 398 },
    .b = { 297, 336, 363, 394, 429, 461, 502, 567, 634, 675 },
};

/* gc1084 t23 */
static const ImageSensorRaw image_sensor_raw_gc1084_t23 = {
    .r = { 944, 821, 758, 698, 645, 608, 588, 564, 529, 498 },
    .b = { 347, 393, 417, 448, 482, 512, 537, 573, 606, 640 },
};

/* gc1084 t31 */
static const ImageSensorRaw image_sensor_raw_gc1084_t31 = {
    .r = { 914, 844, 767, 704, 650, 612, 587, 551, 496, 465 },
    .b = { 335, 377, 406, 455, 502, 539, 565, 607, 662, 707 },
};

/* gc2053 t21 */
static const ImageSensorRaw image_sensor_raw_gc2053_t21 = {
    .r = { 1064, 905, 794, 718, 668, 628, 599, 556, 522, 499 },
    .b = { 307, 344, 375, 412, 454, 490, 525, 581, 637, 692 },
};

/* gc2053 t23, gc2063 t23, os02g10 t23 */
static const ImageSensorRaw image_sensor_raw_gc2053_t23 = {
    .r = { 1008, 868, 781, 708, 647, 609, 575, 535, 499, 468 },
    .b = { 337, 378, 402, 443, 496, 542, 571, 618, 675, 722 },
};

/* gc2053 t31, gc2063 t31 */
static const ImageSensorRaw image_sensor_raw_gc2053_t31 = {
    .r = { 882, 780, 747, 699, 628, 569, 536, 495, 458, 428 },
    .b = { 313, 357, 391, 427, 474, 514, 550, 610, 678, 736 },
};

/* gc2053 t40, imx327 t40, jxf23 t40, sc2310 t40 */
static const ImageSensorRaw image_sensor_raw_gc2053_t40 = {
    .r = { 859, 759, 725, 668, 596, 545, 525, 507, 480, 455 },
    .b = { 286, 339, 371, 414, 469, 511, 536, 567, 605, 645 },
};

/* gc2063 t32 */
static const ImageSensorRaw image_sensor_raw_gc2063_t32 = {
    .r = { 957, 826, 761, 700, 646, 608, 577, 532, 487, 459 },
    .b = { 325, 368, 395, 432, 472, 511, 548, 617, 689, 735 },
};

/* gc2063 t41 */
static const ImageSensorRaw image_sensor_raw_gc2063_t41 = {
    .r = { 888, 774, 737, 686, 613, 559, 529, 495, 465, 443 },
    .b = { 337, 373, 404, 440, 485, 526, 558, 608, 667, 717 },
};

/* gc2083 t23 */
static const ImageSensorRaw image_sensor_raw_gc2083_t23 = {
    .r = { 878, 854, 773, 710, 663, 628, 604, 561, 503, 455 },
    .b = { 405, 410, 437, 483, 538, 580, 604, 638, 708, 771 },
};

/* gc2083 t31 */
static const ImageSensorRaw image_sensor_raw_gc2083_t31 = {
    .r = { 982, 839, 775, 719, 649, 560, 513, 475, 439, 411 },
    .b = { 334, 362, 402, 431, 489, 587, 647, 712, 782, 854 },
};

/* gc2083 t32 */
static const ImageSensorRaw image_sensor_raw_gc2083_t32 = {
    .r = { 882, 784, 744, 689, 633, 598, 570, 536, 509, 480 },
    .b = { 413, 447, 466, 505, 554, 604, 652, 732, 813, 869 },
};

/* gc2083 t41 */
static const ImageSensorRaw image_sensor_raw_gc2083_t41 = {
    .r = { 962, 843, 768, 701, 649, 614, 590, 554, 517, 481 },
    .b = { 344, 378, 406, 442, 497, 541, 570, 609, 653, 701 },
};

/* gc3003a t31 */
static const ImageSensorRaw image_sensor_raw_gc3003a_t31 = {
    .r = { 969, 942, 839, 762, 700, 648, 601, 533, 530, 530 },
    .b = { 473, 480, 518, 548, 590, 629, 671, 730, 733, 733 },
};

/* gc3003a t32 */
static const ImageSensorRaw image_sensor_raw_gc3003a_t32 = {
    .r = { 984, 867, 819, 756, 689, 641, 622, 600, 559, 526 },
    .b = { 434, 468, 486, 519, 556, 586, 615, 653, 682, 711 },
};

/* gc3003a t41 */
static const ImageSensorRaw image_sensor_raw_gc3003a_t41 = {
    .r = { 1048, 903, 811, 745, 686, 645, 620, 571, 533, 496 },
    .b = { 415, 448, 481, 511, 535, 569, 596, 635, 674, 729 },
};

/* gc4023 t31 */
static const ImageSensorRaw image_sensor_raw_gc4023_t31 = {
    .r = { 1070, 923, 842, 769, 701, 646, 605, 567, 522, 484 },
    .b = { 430, 468, 493, 532, 591, 638, 676, 735, 790, 831 },
};

/* gc4023 t32 */
static const ImageSensorRaw image_sensor_raw_gc4023_t32 = {
    .r = { 861, 777, 749, 696, 626, 572, 547, 506, 467, 439 },
    .b = { 497, 555, 587, 621, 654, 686, 735, 815, 891, 931 },
};

/* gc4023 t41 */
static const ImageSensorRaw image_sensor_raw_gc4023_t41 = {
    .r = { 1012, 873, 796, 729, 667, 618, 592, 557, 514, 474 },
    .b = { 417, 453, 488, 523, 565, 590, 613, 659, 697, 755 },
};

/* gc4653 t31 */
static const ImageSensorRaw image_sensor_raw_gc4653_t31 = {
    .r = { 955, 806, 738, 673, 606, 565, 527, 481, 449, 416 },
    .b = { 265, 324, 371, 414, 459, 509, 542, 599, 670, 736 },
};

/* gc4653 t32 */
static const ImageSensorRaw image_sensor_raw_gc4653_t32 = {
    .r = { 847, 791, 703, 658, 632, 603, 568, 517, 477, 460 },
    .b = { 206, 233, 273, 297, 315, 341, 392, 485, 564, 625 },
};

/* gc4653 t40 */
static const ImageSensorRaw image_sensor_raw_gc4653_t40 = {
    .r = { 852, 737, 671, 606, 545, 495, 464, 417, 380, 347 },
    .b = { 247, 306, 335, 364, 410, 451, 499, 564, 626, 685 },
};

/* gc4653 t41 */
static const ImageSensorRaw image_sensor_raw_gc4653_t41 = {
    .r = { 879, 744, 705, 645, 573, 521, 485, 452, 420, 390 },
    .b = { 270, 310, 356, 390, 443, 496, 542, 609, 665, 719 },
};

/* gc4663 t41 */
static const ImageSensorRaw image_sensor_raw_gc4663_t41 = {
    .r = { 882, 748, 699, 637, 566, 509, 475, 443, 410, 380 },
    .b = { 261, 297, 329, 379, 451, 504, 544, 608, 670, 723 },
};

/* gc5603 t32, gc5613 t32 */
static const ImageSensorRaw image_sensor_raw_gc5603_t32 = {
    .r = { 915, 793, 737, 680, 626, 589, 557, 513, 471, 448 },
    .b = { 304, 367, 406, 449, 493, 538, 584, 667, 749, 803 },
};

/* gc5603 t40 */
static const ImageSensorRaw image_sensor_raw_gc5603_t40 = {
    .r = { 905, 782, 729, 674, 619, 574, 543, 496, 461, 435 },
    .b = { 295, 345, 380, 422, 469, 510, 558, 624, 679, 732 },
};

/* gc5603 t41 */
static const ImageSensorRaw image_sensor_raw_gc5603_t41 = {
    .r = { 951, 823, 749, 687, 630, 592, 562, 525, 494, 460 },
    .b = { 277, 327, 369, 412, 466, 512, 540, 584, 638, 687 },
};

/* imx307 t31, imx327 t31, os05a10 t31 */
static const ImageSensorRaw image_sensor_raw_imx307_t31 = {
    .r = { 1061, 903, 793, 718, 671, 633, 602, 561, 526, 503 },
    .b = { 306, 347, 379, 417, 458, 495, 530, 587, 645, 701 },
};

/* imx307 t40 */
static const ImageSensorRaw image_sensor_raw_imx307_t40 = {
    .r = { 904, 777, 715, 652, 579, 537, 507, 464, 422, 395 },
    .b = { 294, 350, 389, 441, 511, 586, 636, 707, 782, 843 },
};

/* imx327 t23 */
static const ImageSensorRaw image_sensor_raw_imx327_t23 = {
    .r = { 815, 713, 682, 635, 562, 512, 480, 444, 414, 387 },
    .b = { 282, 336, 377, 419, 477, 529, 565, 625, 703, 771 },
};

/* imx327 t41 */
static const ImageSensorRaw image_sensor_raw_imx327_t41 = {
    .r = { 957, 828, 763, 697, 634, 586, 554, 507, 473, 442 },
    .b = { 275, 327, 367, 415, 464, 504, 539, 591, 649, 696 },
};

/* jxf23 t21 */
static const ImageSensorRaw image_sensor_raw_jxf23_t21 = {
    .r = { 963, 822, 762, 699, 631, 583, 547, 496, 459, 427 },
    .b = { 404, 445, 476, 510, 553, 594, 621, 673, 739, 769 },
};

/* jxf23 t23 */
static const ImageSensorRaw image_sensor_raw_jxf23_t23 = {
    .r = { 952, 815, 761, 696, 619, 565, 535, 493, 452, 417 },
    .b = { 362, 401, 425, 464, 510, 545, 575, 619, 671, 719 },
};

/* jxf23 t31, jxh62 t31 */
static const ImageSensorRaw image_sensor_raw_jxf23_t31 = {
    .r = { 971, 824, 758, 693, 630, 582, 543, 494, 460, 441 },
    .b = { 397, 433, 457, 493, 538, 578, 606, 654, 711, 761 },
};

/* jxf28 t21 */
static const ImageSensorRaw image_sensor_raw_jxf28_t21 = {
    .r = { 998, 879, 807, 745, 689, 644, 621, 585, 554, 529 },
    .b = { 528, 560, 591, 620, 654, 682, 715, 760, 806, 849 },
};

/* jxf37 t21 */
static const ImageSensorRaw image_sensor_raw_jxf37_t21 = {
    .r = { 1046, 898, 816, 754, 716, 683, 642, 582, 542, 509 },
    .b = { 451, 485, 514, 556, 613, 657, 680, 718, 765, 809 },
};

/* jxf37 t31 */
static const ImageSensorRaw image_sensor_raw_jxf37_t31 = {
    .r = { 901, 782, 730, 667, 607, 556, 533, 499, 458, 432 },
    .b = { 422, 473, 511, 550, 586, 610, 645, 699, 753, 805 },
};

/* jxf37 t40 */
static const ImageSensorRaw image_sensor_raw_jxf37_t40 = {
    .r = { 869, 765, 728, 668, 604, 556, 527, 490, 458, 435 },
    .b = { 431, 474, 511, 542, 571, 595, 620, 661, 710, 754 },
};

/* jxh62 t21 */
static const ImageSensorRaw image_sensor_raw_jxh62_t21 = {
    .r = { 1108, 938, 820, 733, 677, 645, 617, 572, 524, 477 },
    .b = { 379, 412, 454, 490, 531, 569, 590, 625, 685, 742 },
};

/* jxh63p t23 */
static const ImageSensorRaw image_sensor_raw_jxh63p_t23 = {
    .r = { 1028, 882, 803, 734, 677, 636, 607, 572, 530, 496 },
    .b = { 428, 462, 488, 524, 564, 593, 609, 641, 688, 744 },
};

/* jxk04 t31 */
static const ImageSensorRaw image_sensor_raw_jxk04_t31 = {
    .r = { 944, 831, 775, 721, 662, 620, 601, 575, 534, 500 },
    .b = { 528, 539, 572, 602, 636, 665, 692, 738, 789, 837 },
};

/* jxk04 t40 */
static const ImageSensorRaw image_sensor_raw_jxk04_t40 = {
    .r = { 928, 829, 772, 714, 653, 603, 575, 543, 511, 477 },
    .b = { 502, 530, 566, 601, 642, 675, 710, 765, 820, 862 },
};

/* jxq03 t31 */
static const ImageSensorRaw image_sensor_raw_jxq03_t31 = {
    .r = { 851, 774, 743, 703, 649, 605, 582, 556, 525, 504 },
    .b = { 491, 529, 549, 585, 633, 669, 702, 751, 802, 852 },
};

/* jxq03 t40 */
static const ImageSensorRaw image_sensor_raw_jxq03_t40 = {
    .r = { 903, 805, 746, 706, 659, 606, 578, 546, 514, 487 },
    .b = { 467, 494, 519, 543, 578, 632, 676, 729, 786, 839 },
};

/* jxq03p t23 */
static const ImageSensorRaw image_sensor_raw_jxq03p_t23 = {
    .r = { 901, 844, 814, 790, 766, 746, 737, 723, 706, 687 },
    .b = { 537, 576, 594, 630, 668, 696, 717, 747, 782, 814 },
};

/* jxq03p t31 */
static const ImageSensorRaw image_sensor_raw_jxq03p_t31 = {
    .r = { 901, 839, 809, 779, 750, 726, 712, 697, 688, 673 },
    .b = { 527, 559, 581, 606, 644, 677, 708, 756, 801, 846 },
};

/* jxq03p t41 */
static const ImageSensorRaw image_sensor_raw_jxq03p_t41 = {
    .r = { 896, 830, 790, 757, 734, 723, 719, 714, 704, 687 },
    .b = { 571, 608, 629, 676, 744, 786, 813, 845, 886, 936 },
};

/* mis2008 t31 */
static const ImageSensorRaw image_sensor_raw_mis2008_t31 = {
    .r = { 1000, 856, 707, 615, 558, 527, 502, 463, 423, 392 },
    .b = { 249, 296, 346, 409, 471, 527, 574, 638, 710, 777 },
};

/* mis2008 t40 */
static const ImageSensorRaw image_sensor_raw_mis2008_t40 = {
    .r = { 780, 744, 658, 575, 522, 482, 454, 409, 352, 338 },
    .b = { 215, 239, 309, 364, 439, 494, 522, 574, 684, 719 },
};

/* mis20c1 t23 */
static const ImageSensorRaw image_sensor_raw_mis20c1_t23 = {
    .r = { 786, 691, 653, 608, 562, 524, 501, 462, 447, 447 },
    .b = { 335, 368, 381, 403, 430, 453, 485, 540, 563, 563 },
};

/* os02b10 t21 */
static const ImageSensorRaw image_sensor_raw_os02b10_t21 = {
    .r = { 891, 787, 742, 702, 650, 583, 551, 514, 480, 459 },
    .b = { 375, 415, 442, 475, 519, 566, 608, 664, 715, 769 },
};

/* os02g10 t21 */
static const ImageSensorRaw image_sensor_raw_os02g10_t21 = {
    .r = { 865, 790, 718, 664, 624, 585, 549, 509, 489, 481 },
    .b = { 431, 449, 479, 506, 534, 573, 645, 726, 756, 777 },
};

/* os02g10 t31 */
static const ImageSensorRaw image_sensor_raw_os02g10_t31 = {
    .r = { 1061, 903, 793, 718, 671, 633, 602, 561, 528, 506 },
    .b = { 306, 347, 379, 417, 458, 495, 530, 587, 645, 702 },
};

/* os02g10 t32 */
static const ImageSensorRaw image_sensor_raw_os02g10_t32 = {
    .r = { 894, 782, 724, 671, 616, 591, 574, 539, 498, 468 },
    .b = { 404, 445, 470, 488, 508, 546, 574, 606, 646, 695 },
};

/* os02g10 t40 */
static const ImageSensorRaw image_sensor_raw_os02g10_t40 = {
    .r = { 1018, 848, 734, 653, 601, 569, 543, 490, 447, 446 },
    .b = { 368, 416, 472, 520, 575, 623, 654, 700, 787, 789 },
};

/* os02n10 t23 */
static const ImageSensorRaw image_sensor_raw_os02n10_t23 = {
    .r = { 957, 818, 733, 678, 654, 634, 614, 585, 551, 520 },
    .b = { 369, 396, 404, 439, 484, 518, 541, 570, 606, 640 },
};

/* os03b10 t31 */
static const ImageSensorRaw image_sensor_raw_os03b10_t31 = {
    .r = { 878, 835, 729, 647, 594, 558, 529, 482, 428, 427 },
    .b = { 328, 351, 423, 483, 535, 575, 596, 642, 724, 727 },
};

/* os04d10 t32 */
static const ImageSensorRaw image_sensor_raw_os04d10_t32 = {
    .r = { 845, 719, 674, 617, 560, 516, 484, 444, 410, 392 },
    .b = { 251, 299, 336, 381, 433, 478, 519, 597, 679, 735 },
};

/* os04d10 t41 */
static const ImageSensorRaw image_sensor_raw_os04d10_t41 = {
    .r = { 784, 730, 692, 645, 585, 543, 506, 462, 434, 404 },
    .b = { 297, 322, 341, 370, 423, 475, 516, 573, 632, 686 },
};

/* os05a10 t40 */
static const ImageSensorRaw image_sensor_raw_os05a10_t40 = {
    .r = { 859, 727, 649, 590, 547, 519, 490, 452, 427, 402 },
    .b = { 299, 348, 386, 429, 469, 508, 529, 571, 629, 679 },
};

/* os08a10 t40, sc8238 t40 */
static const ImageSensorRaw image_sensor_raw_os08a10_t40 = {
    .r = { 813, 704, 648, 594, 539, 502, 483, 459, 436, 411 },
    .b = { 310, 352, 374, 419, 473, 514, 540, 573, 628, 683 },
};

/* os08c10 t41 */
static const ImageSensorRaw image_sensor_raw_os08c10_t41 = {
    .r = { 994, 830, 766, 695, 605, 543, 502, 448, 403, 370 },
    .b = { 315, 359, 393, 434, 493, 564, 617, 702, 800, 861 },
};

/* ov2735b t21 */
static const ImageSensorRaw image_sensor_raw_ov2735b_t21 = {
    .r = { 894, 757, 673, 615, 569, 542, 518, 478, 423, 383 },
    .b = { 374, 424, 475, 523, 566, 614, 656, 732, 825, 894 },
};

/* ov9732 t32 */
static const ImageSensorRaw image_sensor_raw_ov9732_t32 = {
    .r = { 851, 780, 761, 720, 671, 631, 596, 551, 517, 498 },
    .b = { 464, 505, 532, 557, 579, 609, 654, 733, 809, 864 },
};

/* ps5260 t21 */
static const ImageSensorRaw image_sensor_raw_ps5260_t21 = {
    .r = { 1030, 867, 796, 717, 649, 605, 574, 531, 485, 450 },
    .b = { 378, 432, 479, 515, 556, 597, 633, 687, 746, 802 },
};

/* sc1235 t21 */
static const ImageSensorRaw image_sensor_raw_sc1235_t21 = {
    .r = { 1083, 945, 880, 820, 742, 680, 654, 609, 560, 525 },
    .b = { 487, 526, 551, 583, 647, 704, 747, 812, 898, 967 },
};

/* sc1a4t t23 */
static const ImageSensorRaw image_sensor_raw_sc1a4t_t23 = {
    .r = { 947, 809, 741, 687, 642, 601, 570, 512, 462, 461 },
    .b = { 308, 357, 389, 442, 501, 543, 572, 626, 708, 711 },
};

/* sc2232 t21 */
static const ImageSensorRaw image_sensor_raw_sc2232_t21 = {
    .r = { 993, 895, 852, 796, 743, 699, 669, 624, 585, 550 },
    .b = { 478, 520, 546, 573, 601, 624, 655, 708, 764, 817 },
};

/* sc2232h t21 */
static const ImageSensorRaw image_sensor_raw_sc2232h_t21 = {
    .r = { 936, 824, 765, 712, 665, 624, 598, 557, 525, 500 },
    .b = { 380, 418, 447, 479, 517, 545, 575, 628, 689, 733 },
};

/* sc2232h t31 */
static const ImageSensorRaw image_sensor_raw_sc2232h_t31 = {
    .r = { 930, 817, 753, 697, 651, 613, 594, 566, 549, 533 },
    .b = { 400, 438, 461, 496, 536, 568, 599, 638, 666, 692 },
};

/* sc2235 t21 */
static const ImageSensorRaw image_sensor_raw_sc2235_t21 = {
    .r = { 1082, 937, 844, 751, 676, 634, 605, 570, 550, 526 },
    .b = { 480, 535, 574, 615, 669, 731, 783, 852, 908, 962 },
};

/* sc223a t23 */
static const ImageSensorRaw image_sensor_raw_sc223a_t23 = {
    .r = { 924, 855, 756, 687, 643, 608, 583, 551, 516, 482 },
    .b = { 316, 342, 381, 428, 476, 517, 552, 596, 636, 677 },
};

/* sc223a t31 */
static const ImageSensorRaw image_sensor_raw_sc223a_t31 = {
    .r = { 869, 750, 703, 650, 588, 535, 503, 462, 429, 403 },
    .b = { 312, 358, 380, 423, 474, 512, 551, 611, 676, 733 },
};

/* sc223a t40 */
static const ImageSensorRaw image_sensor_raw_sc223a_t40 = {
    .r = { 868, 755, 679, 610, 548, 495, 471, 436, 398, 371 },
    .b = { 252, 291, 334, 385, 448, 494, 543, 611, 676, 733 },
};

/* sc2300 t21 */
static const ImageSensorRaw image_sensor_raw_sc2300_t21 = {
    .r = { 964, 803, 719, 644, 572, 524, 506, 473, 422, 386 },
    .b = { 331, 377, 422, 466, 521, 562, 596, 646, 709, 763 },
};

/* sc2310 t23 */
static const ImageSensorRaw image_sensor_raw_sc2310_t23 = {
    .r = { 975, 833, 783, 723, 643, 586, 552, 504, 466, 435 },
    .b = { 367, 426, 467, 510, 569, 619, 658, 707, 766, 825 },
};

/* sc2310 t31 */
static const ImageSensorRaw image_sensor_raw_sc2310_t31 = {
    .r = { 931, 811, 760, 699, 625, 574, 553, 520, 484, 460 },
    .b = { 391, 430, 465, 506, 555, 595, 623, 651, 689, 734 },
};

/* sc231hai t32 */
static const ImageSensorRaw image_sensor_raw_sc231hai_t32 = {
    .r = { 853, 734, 680, 623, 569, 530, 511, 486, 455, 426 },
    .b = { 234, 277, 302, 337, 375, 406, 430, 459, 489, 518 },
};

/* sc2331 t23, sc2332 t31 */
static const ImageSensorRaw image_sensor_raw_sc2331_t23 = {
    .r = { 1031, 934, 843, 783, 732, 688, 657, 612, 563, 528 },
    .b = { 401, 426, 466, 490, 544, 590, 626, 682, 752, 802 },
};

/* sc2331 t31 */
static const ImageSensorRaw image_sensor_raw_sc2331_t31 = {
    .r = { 838, 754, 666, 618, 582, 546, 504, 460, 450, 445 },
    .b = { 298, 322, 365, 391, 413, 453, 525, 612, 648, 670 },
};

/* sc2332 t21 */
static const ImageSensorRaw image_sensor_raw_sc2332_t21 = {
    .r = { 993, 894, 851, 796, 740, 688, 645, 590, 562, 538 },
    .b = { 476, 522, 551, 574, 590, 615, 665, 745, 790, 825 },
};

/* sc2336 t23 */
static const ImageSensorRaw image_sensor_raw_sc2336_t23 = {
    .r = { 991, 896, 795, 715, 665, 619, 577, 533, 516, 511 },
    .b = { 333, 356, 395, 427, 471, 522, 581, 652, 694, 714 },
};

/* sc2336 t31 */
static const ImageSensorRaw image_sensor_raw_sc2336_t31 = {
    .r = { 988, 894, 793, 714, 659, 608, 565, 524, 508, 499 },
    .b = { 249, 273, 311, 342, 379, 428, 489, 565, 623, 666 },
};

/* sc2336 t41 */
static const ImageSensorRaw image_sensor_raw_sc2336_t41 = {
    .r = { 943, 810, 739, 680, 625, 583, 570, 552, 515, 481 },
    .b = { 306, 348, 368, 409, 451, 488, 528, 579, 615, 654 },
};

/* sc2336p t23 */
static const ImageSensorRaw image_sensor_raw_sc2336p_t23 = {
    .r = { 945, 810, 732, 676, 632, 593, 565, 526, 498, 480 },
    .b = { 323, 364, 381, 427, 477, 516, 555, 612, 673, 736 },
};

/* sc2336p t31 */
static const ImageSensorRaw image_sensor_raw_sc2336p_t31 = {
    .r = { 920, 825, 712, 666, 639, 601, 558, 508, 480, 461 },
    .b = { 310, 339, 388, 414, 441, 487, 551, 629, 692, 743 },
};

/* sc301iot t23 */
static const ImageSensorRaw image_sensor_raw_sc301iot_t23 = {
    .r = { 964, 826, 753, 689, 635, 603, 577, 538, 496, 470 },
    .b = { 324, 370, 406, 444, 479, 514, 540, 577, 622, 676 },
};

/* sc301iot t31 */
static const ImageSensorRaw image_sensor_raw_sc301iot_t31 = {
    .r = { 981, 849, 778, 721, 655, 602, 571, 524, 483, 448 },
    .b = { 293, 339, 371, 402, 452, 515, 543, 589, 662, 734 },
};

/* sc301iot t32 */
static const ImageSensorRaw image_sensor_raw_sc301iot_t32 = {
    .r = { 893, 787, 741, 683, 622, 574, 549, 504, 470, 442 },
    .b = { 342, 382, 402, 441, 487, 527, 573, 648, 735, 785 },
};

/* sc301iot t41 */
static const ImageSensorRaw image_sensor_raw_sc301iot_t41 = {
    .r = { 1029, 880, 789, 712, 658, 622, 583, 542, 510, 479 },
    .b = { 348, 384, 405, 452, 524, 584, 621, 678, 734, 784 },
};

/* sc3235 t31 */
static const ImageSensorRaw image_sensor_raw_sc3235_t31 = {
    .r = { 892, 797, 766, 722, 667, 624, 596, 560, 528, 500 },
    .b = { 441, 479, 510, 543, 581, 614, 640, 685, 739, 792 },
};

/* sc3332 t23 */
static const ImageSensorRaw image_sensor_raw_sc3332_t23 = {
    .r = { 1179, 1047, 939, 866, 812, 760, 715, 647, 610, 591 },
    .b = { 342, 383, 420, 447, 465, 492, 553, 646, 703, 746 },
};

/* sc3332 t41 */
static const ImageSensorRaw image_sensor_raw_sc3332_t41 = {
    .r = { 1088, 935, 869, 797, 724, 672, 644, 616, 573, 536 },
    .b = { 332, 374, 406, 449, 493, 531, 560, 601, 641, 680 },
};

/* sc3335 t31 */
static const ImageSensorRaw image_sensor_raw_sc3335_t31 = {
    .r = { 868, 771, 748, 700, 635, 588, 561, 527, 494, 468 },
    .b = { 471, 510, 555, 596, 638, 674, 709, 760, 814, 872 },
};

/* sc3335 t40 */
static const ImageSensorRaw image_sensor_raw_sc3335_t40 = {
    .r = { 907, 822, 788, 742, 686, 643, 619, 586, 564, 539 },
    .b = { 550, 594, 627, 664, 697, 727, 760, 811, 872, 937 },
};

/* sc3336 t23 */
static const ImageSensorRaw image_sensor_raw_sc3336_t23 = {
    .r = { 1212, 1020, 921, 845, 785, 739, 697, 644, 610, 567 },
    .b = { 360, 403, 424, 468, 522, 572, 619, 691, 745, 782 },
};

/* sc3336 t31 */
static const ImageSensorRaw image_sensor_raw_sc3336_t31 = {
    .r = { 1096, 929, 882, 811, 722, 654, 621, 577, 533, 499 },
    .b = { 346, 397, 453, 504, 566, 616, 660, 726, 803, 865 },
};

/* sc3336 t32 */
static const ImageSensorRaw image_sensor_raw_sc3336_t32 = {
    .r = { 957, 825, 759, 697, 643, 605, 572, 531, 486, 459 },
    .b = { 325, 367, 394, 430, 470, 509, 546, 616, 688, 735 },
};

/* sc3336 t41 */
static const ImageSensorRaw image_sensor_raw_sc3336_t41 = {
    .r = { 1181, 1006, 903, 825, 764, 707, 666, 615, 577, 529 },
    .b = { 328, 367, 391, 415, 454, 503, 533, 575, 639, 693 },
};

/* sc3338 t31 */
static const ImageSensorRaw image_sensor_raw_sc3338_t31 = {
    .r = { 983, 983, 909, 811, 735, 678, 637, 577, 547, 547 },
    .b = { 364, 364, 426, 492, 543, 582, 610, 653, 692, 692 },
};

/* sc3338 t40 */
static const ImageSensorRaw image_sensor_raw_sc3338_t40 = {
    .r = { 1092, 950, 892, 820, 739, 677, 635, 584, 550, 515 },
    .b = { 367, 411, 433, 478, 523, 565, 609, 692, 781, 830 },
};

/* sc3338 t41 */
static const ImageSensorRaw image_sensor_raw_sc3338_t41 = {
    .r = { 1152, 1054, 961, 893, 815, 756, 712, 638, 595, 562 },
    .b = { 359, 411, 434, 459, 483, 527, 582, 671, 735, 773 },
};

/* sc401ai t31 */
static const ImageSensorRaw image_sensor_raw_sc401ai_t31 = {
    .r = { 989, 815, 718, 645, 594, 559, 526, 481, 444, 414 },
    .b = { 270, 323, 357, 400, 456, 505, 540, 595, 659, 721 },
};

/* sc401ai t32 */
static const ImageSensorRaw image_sensor_raw_sc401ai_t32 = {
    .r = { 871, 746, 700, 639, 577, 534, 513, 476, 445, 414 },
    .b = { 300, 343, 379, 428, 487, 533, 568, 610, 665, 718 },
};

/* sc401ai t41 */
static const ImageSensorRaw image_sensor_raw_sc401ai_t41 = {
    .r = { 908, 783, 734, 674, 599, 543, 505, 464, 425, 404 },
    .b = { 275, 322, 356, 391, 439, 484, 527, 601, 679, 725 },
};

/* sc4236 t31 */
static const ImageSensorRaw image_sensor_raw_sc4236_t31 = {
    .r = { 1088, 954, 874, 806, 758, 718, 701, 669, 629, 597 },
    .b = { 497, 519, 522, 538, 582, 615, 650, 691, 728, 767 },
};

/* sc4336 t31 */
static const ImageSensorRaw image_sensor_raw_sc4336_t31 = {
    .r = { 1029, 1010, 907, 834, 772, 715, 664, 608, 584, 582 },
    .b = { 422, 428, 474, 508, 530, 568, 640, 730, 776, 781 },
};

/* sc4336 t32 */
static const ImageSensorRaw image_sensor_raw_sc4336_t32 = {
    .r = { 1151, 992, 912, 842, 775, 725, 694, 646, 597, 555 },
    .b = { 374, 406, 420, 459, 500, 535, 561, 592, 630, 686 },
};

/* sc4336 t41 */
static const ImageSensorRaw image_sensor_raw_sc4336_t41 = {
    .r = { 1107, 959, 890, 825, 742, 683, 648, 607, 565, 517 },
    .b = { 370, 413, 439, 471, 516, 567, 600, 648, 711, 779 },
};

/* sc4336p t31 */
static const ImageSensorRaw image_sensor_raw_sc4336p_t31 = {
    .r = { 875, 805, 743, 693, 655, 617, 578, 535, 520, 513 },
    .b = { 355, 372, 404, 437, 463, 500, 564, 637, 672, 695 },
};

/* sc4336p t32 */
static const ImageSensorRaw image_sensor_raw_sc4336p_t32 = {
    .r = { 946, 845, 796, 742, 688, 653, 624, 582, 547, 523 },
    .b = { 374, 407, 425, 456, 491, 528, 561, 621, 687, 734 },
};

/* sc4336p t41 */
static const ImageSensorRaw image_sensor_raw_sc4336p_t41 = {
    .r = { 969, 864, 805, 754, 712, 662, 634, 597, 553, 520 },
    .b = { 359, 386, 404, 423, 459, 497, 524, 559, 608, 663 },
};

/* sc500ai t31 */
static const ImageSensorRaw image_sensor_raw_sc500ai_t31 = {
    .r = { 984, 811, 724, 649, 593, 550, 520, 479, 441, 409 },
    .b = { 269, 309, 337, 382, 443, 490, 525, 580, 646, 711 },
};

/* sc500ai t40 */
static const ImageSensorRaw image_sensor_raw_sc500ai_t40 = {
    .r = { 876, 746, 687, 619, 556, 514, 480, 444, 412, 391 },
    .b = { 277, 334, 377, 423, 467, 510, 541, 593, 652, 710 },
};

/* sc500ai t41 */
static const ImageSensorRaw image_sensor_raw_sc500ai_t41 = {
    .r = { 853, 736, 685, 620, 550, 498, 466, 425, 403, 380 },
    .b = { 294, 343, 384, 434, 493, 543, 586, 646, 706, 753 },
};

/* sc5235 t31 */
static const ImageSensorRaw image_sensor_raw_sc5235_t31 = {
    .r = { 1015, 875, 791, 719, 654, 606, 568, 522, 484, 466 },
    .b = { 391, 426, 458, 491, 541, 584, 615, 665, 717, 763 },
};

/* sc5235 t40 */
static const ImageSensorRaw image_sensor_raw_sc5235_t40 = {
    .r = { 954, 834, 790, 736, 654, 589, 553, 513, 484, 452 },
    .b = { 458, 503, 544, 583, 634, 677, 713, 766, 827, 877 },
};

/* sc5336 t41 */
static const ImageSensorRaw image_sensor_raw_sc5336_t41 = {
    .r = { 946, 835, 780, 726, 675, 638, 621, 591, 559, 529 },
    .b = { 369, 403, 426, 457, 491, 520, 546, 578, 619, 654 },
};

/* sc8238 t41 */
static const ImageSensorRaw image_sensor_raw_sc8238_t41 = {
    .r = { 1043, 926, 869, 808, 743, 688, 665, 643, 608, 574 },
    .b = { 471, 508, 545, 577, 613, 640, 672, 705, 737, 776 },
};

/* sc830ai t40 */
static const ImageSensorRaw image_sensor_raw_sc830ai_t40 = {
    .r = { 1107, 968, 913, 839, 757, 697, 656, 605, 553, 523 },
    .b = { 354, 387, 403, 436, 474, 510, 547, 613, 675, 714 },
};

/* sc830ai t41 */
static const ImageSensorRaw image_sensor_raw_sc830ai_t41 = {
    .r = { 1112, 960, 903, 831, 755, 698, 672, 646, 613, 580 },
    .b = { 359, 394, 432, 467, 507, 539, 565, 599, 632, 668 },
};

/* sc835hai t32 */
static const ImageSensorRaw image_sensor_raw_sc835hai_t32 = {
    .r = { 964, 810, 722, 653, 601, 556, 532, 487, 449, 419 },
    .b = { 330, 355, 372, 414, 465, 502, 535, 581, 625, 673 },
};

/* gc2023 */
static const ImageSensorRaw image_sensor_raw_ap_gc2023 = {
    .r = { 1628, 1349, 1183, 1068, 988, 923, 869, 794, 759, 696 },
    .b = { 485, 552, 624, 691, 755, 807, 892, 1009, 1065, 1169 },
};

/* gc2053 */
static const ImageSensorRaw image_sensor_raw_ap_gc2053 = {
    .r = { 944, 821, 748, 686, 631, 586, 559, 523, 493, 468 },
    .b = { 314, 346, 373, 411, 455, 491, 526, 573, 626, 682 },
};

/* imx307 */
static const ImageSensorRaw image_sensor_raw_ap_imx307 = {
    .r = { 972, 825, 726, 648, 582, 550, 525, 486, 455, 434 },
    .b = { 284, 341, 387, 430, 474, 522, 562, 623, 694, 735 },
};

/* jxf22 */
static const ImageSensorRaw image_sensor_raw_ap_jxf22 = {
    .r = { 1054, 893, 800, 726, 667, 622, 586, 532, 480, 460 },
    .b = { 399, 444, 484, 520, 550, 588, 631, 697, 758, 784 },
};

/* jxh42 */
static const ImageSensorRaw image_sensor_raw_ap_jxh42 = {
    .r = { 909, 875, 821, 721, 641, 576, 553, 518, 477, 450 },
    .b = { 393, 408, 433, 490, 545, 592, 626, 678, 744, 800 },
};

/* jxh62 */
static const ImageSensorRaw image_sensor_raw_ap_jxh62 = {
    .r = { 978, 844, 772, 700, 631, 576, 548, 504, 472, 466 },
    .b = { 394, 439, 484, 525, 562, 592, 631, 691, 737, 747 },
};

/* os02b10 */
static const ImageSensorRaw image_sensor_raw_ap_os02b10 = {
    .r = { 910, 810, 765, 693, 624, 588, 561, 523, 498, 476 },
    .b = { 404, 441, 465, 493, 518, 565, 605, 662, 699, 732 },
};

/* os05a10 */
static const ImageSensorRaw image_sensor_raw_ap_os05a10 = {
    .r = { 869, 741, 667, 598, 538, 505, 480, 443, 413, 390 },
    .b = { 301, 363, 413, 452, 482, 549, 607, 693, 745, 771 },
};

/* ov2710 */
static const ImageSensorRaw image_sensor_raw_ap_ov2710 = {
    .r = { 1100, 903, 834, 755, 674, 614, 583, 535, 493, 459 },
    .b = { 354, 406, 439, 472, 503, 533, 577, 644, 695, 746 },
};

/* ov2735b */
static const ImageSensorRaw image_sensor_raw_ap_ov2735b = {
    .r = { 894, 782, 707, 646, 596, 565, 540, 505, 477, 454 },
    .b = { 428, 472, 500, 535, 568, 626, 676, 747, 796, 838 },
};

/* ov9712 */
static const ImageSensorRaw image_sensor_raw_ap_ov9712 = {
    .r = { 1029, 918, 854, 766, 664, 582, 543, 500, 469, 426 },
    .b = { 352, 395, 427, 464, 497, 523, 569, 622, 663, 731 },
};

/* ov9732 */
static const ImageSensorRaw image_sensor_raw_ap_ov9732 = {
    .r = { 930, 813, 741, 684, 636, 597, 570, 529, 519, 496 },
    .b = { 407, 451, 477, 514, 569, 612, 657, 725, 783, 836 },
};

/* ov9750 */
static const ImageSensorRaw image_sensor_raw_ap_ov9750 = {
    .r = { 926, 788, 719, 656, 600, 556, 524, 477, 468, 438 },
    .b = { 338, 401, 453, 503, 548, 583, 623, 689, 733, 788 },
};

/* ps5250 */
static const ImageSensorRaw image_sensor_raw_ap_ps5250 = {
    .r = { 968, 842, 792, 732, 654, 595, 566, 524, 491, 464 },
    .b = { 373, 416, 446, 477, 513, 546, 592, 658, 707, 741 },
};

/* sc1235 */
static const ImageSensorRaw image_sensor_raw_ap_sc1235 = {
    .r = { 1136, 972, 872, 785, 714, 670, 635, 582, 528, 504 },
    .b = { 516, 571, 609, 651, 689, 751, 805, 884, 943, 970 },
};

/* sc2135 */
static const ImageSensorRaw image_sensor_raw_ap_sc2135 = {
    .r = { 1085, 958, 885, 812, 729, 664, 628, 577, 564, 545 },
    .b = { 492, 548, 582, 630, 697, 751, 812, 901, 937, 978 },
};

/* sc2232 */
static const ImageSensorRaw image_sensor_raw_ap_sc2232 = {
    .r = { 999, 903, 864, 786, 707, 668, 638, 596, 568, 547 },
    .b = { 486, 521, 551, 578, 601, 644, 682, 739, 786, 822 },
};

/* sc2235 */
static const ImageSensorRaw image_sensor_raw_ap_sc2235 = {
    .r = { 1096, 971, 873, 782, 703, 659, 626, 577, 543, 517 },
    .b = { 528, 584, 635, 677, 713, 770, 822, 898, 964, 1010 },
};

/* sc4236 */
static const ImageSensorRaw image_sensor_raw_ap_sc4236 = {
    .r = { 974, 875, 829, 759, 690, 654, 627, 587, 562, 545 },
    .b = { 528, 567, 599, 623, 643, 688, 727, 787, 832, 860 },
};

/* sc5235 */
static const ImageSensorRaw image_sensor_raw_ap_sc5235 = {
    .r = { 932, 780, 711, 634, 552, 515, 489, 449, 415, 396 },
    .b = { 338, 407, 463, 502, 530, 583, 622, 683, 742, 781 },
};

static const struct {
    const char *model;
    ImageSensorRawPart part;
    const ImageSensorRaw *raw;
} image_sensor_raw_models[] = {
    { "cv2003", IMAGE_SENSOR_PART_T32, &image_sensor_raw_cv2003_t32 },
    { "cv8001", IMAGE_SENSOR_PART_T32, &image_sensor_raw_cv8001_t32 },
    { "gc1084", IMAGE_SENSOR_PART_T23, &image_sensor_raw_gc1084_t23 },
    { "gc1084", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc1084_t31 },
    { "gc2023", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_gc2023 },
    { "gc2053", IMAGE_SENSOR_PART_T21, &image_sensor_raw_gc2053_t21 },
    { "gc2053", IMAGE_SENSOR_PART_T23, &image_sensor_raw_gc2053_t23 },
    { "gc2053", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc2053_t31 },
    { "gc2053", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc2053_t40 },
    { "gc2053", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_gc2053 },
    { "gc2063", IMAGE_SENSOR_PART_T23, &image_sensor_raw_gc2053_t23 },
    { "gc2063", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc2053_t31 },
    { "gc2063", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc2063_t32 },
    { "gc2063", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc2063_t41 },
    { "gc2083", IMAGE_SENSOR_PART_T23, &image_sensor_raw_gc2083_t23 },
    { "gc2083", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc2083_t31 },
    { "gc2083", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc2083_t32 },
    { "gc2083", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc2083_t41 },
    { "gc3003a", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc3003a_t31 },
    { "gc3003a", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc3003a_t32 },
    { "gc3003a", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc3003a_t41 },
    { "gc4023", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc4023_t31 },
    { "gc4023", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc4023_t32 },
    { "gc4023", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc4023_t41 },
    { "gc4653", IMAGE_SENSOR_PART_T31, &image_sensor_raw_gc4653_t31 },
    { "gc4653", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc4653_t32 },
    { "gc4653", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc4653_t40 },
    { "gc4653", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc4653_t41 },
    { "gc4663", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc4663_t41 },
    { "gc5603", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc5603_t32 },
    { "gc5603", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc5603_t40 },
    { "gc5603", IMAGE_SENSOR_PART_T41, &image_sensor_raw_gc5603_t41 },
    { "gc5613", IMAGE_SENSOR_PART_T32, &image_sensor_raw_gc5603_t32 },
    { "imx307", IMAGE_SENSOR_PART_T31, &image_sensor_raw_imx307_t31 },
    { "imx307", IMAGE_SENSOR_PART_T40, &image_sensor_raw_imx307_t40 },
    { "imx307", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_imx307 },
    { "imx327", IMAGE_SENSOR_PART_T23, &image_sensor_raw_imx327_t23 },
    { "imx327", IMAGE_SENSOR_PART_T31, &image_sensor_raw_imx307_t31 },
    { "imx327", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc2053_t40 },
    { "imx327", IMAGE_SENSOR_PART_T41, &image_sensor_raw_imx327_t41 },
    { "jxf22", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_jxf22 },
    { "jxf23", IMAGE_SENSOR_PART_T21, &image_sensor_raw_jxf23_t21 },
    { "jxf23", IMAGE_SENSOR_PART_T23, &image_sensor_raw_jxf23_t23 },
    { "jxf23", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxf23_t31 },
    { "jxf23", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc2053_t40 },
    { "jxf23", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_jxf23_t31 },
    { "jxf28", IMAGE_SENSOR_PART_T21, &image_sensor_raw_jxf28_t21 },
    { "jxf37", IMAGE_SENSOR_PART_T21, &image_sensor_raw_jxf37_t21 },
    { "jxf37", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxf37_t31 },
    { "jxf37", IMAGE_SENSOR_PART_T40, &image_sensor_raw_jxf37_t40 },
    { "jxh42", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_jxh42 },
    { "jxh62", IMAGE_SENSOR_PART_T21, &image_sensor_raw_jxh62_t21 },
    { "jxh62", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxf23_t31 },
    { "jxh62", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_jxh62 },
    { "jxh63p", IMAGE_SENSOR_PART_T23, &image_sensor_raw_jxh63p_t23 },
    { "jxk04", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxk04_t31 },
    { "jxk04", IMAGE_SENSOR_PART_T40, &image_sensor_raw_jxk04_t40 },
    { "jxq03", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxq03_t31 },
    { "jxq03", IMAGE_SENSOR_PART_T40, &image_sensor_raw_jxq03_t40 },
    { "jxq03p", IMAGE_SENSOR_PART_T23, &image_sensor_raw_jxq03p_t23 },
    { "jxq03p", IMAGE_SENSOR_PART_T31, &image_sensor_raw_jxq03p_t31 },
    { "jxq03p", IMAGE_SENSOR_PART_T41, &image_sensor_raw_jxq03p_t41 },
    { "mis2008", IMAGE_SENSOR_PART_T31, &image_sensor_raw_mis2008_t31 },
    { "mis2008", IMAGE_SENSOR_PART_T40, &image_sensor_raw_mis2008_t40 },
    { "mis20c1", IMAGE_SENSOR_PART_T23, &image_sensor_raw_mis20c1_t23 },
    { "os02b10", IMAGE_SENSOR_PART_T21, &image_sensor_raw_os02b10_t21 },
    { "os02b10", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_os02b10 },
    { "os02g10", IMAGE_SENSOR_PART_T21, &image_sensor_raw_os02g10_t21 },
    { "os02g10", IMAGE_SENSOR_PART_T23, &image_sensor_raw_gc2053_t23 },
    { "os02g10", IMAGE_SENSOR_PART_T31, &image_sensor_raw_os02g10_t31 },
    { "os02g10", IMAGE_SENSOR_PART_T32, &image_sensor_raw_os02g10_t32 },
    { "os02g10", IMAGE_SENSOR_PART_T40, &image_sensor_raw_os02g10_t40 },
    { "os02n10", IMAGE_SENSOR_PART_T23, &image_sensor_raw_os02n10_t23 },
    { "os03b10", IMAGE_SENSOR_PART_T31, &image_sensor_raw_os03b10_t31 },
    { "os04d10", IMAGE_SENSOR_PART_T32, &image_sensor_raw_os04d10_t32 },
    { "os04d10", IMAGE_SENSOR_PART_T41, &image_sensor_raw_os04d10_t41 },
    { "os05a10", IMAGE_SENSOR_PART_T31, &image_sensor_raw_imx307_t31 },
    { "os05a10", IMAGE_SENSOR_PART_T40, &image_sensor_raw_os05a10_t40 },
    { "os05a10", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_os05a10 },
    { "os08a10", IMAGE_SENSOR_PART_T40, &image_sensor_raw_os08a10_t40 },
    { "os08c10", IMAGE_SENSOR_PART_T41, &image_sensor_raw_os08c10_t41 },
    { "ov2710", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ov2710 },
    { "ov2735b", IMAGE_SENSOR_PART_T21, &image_sensor_raw_ov2735b_t21 },
    { "ov2735b", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ov2735b },
    { "ov9712", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ov9712 },
    { "ov9732", IMAGE_SENSOR_PART_T32, &image_sensor_raw_ov9732_t32 },
    { "ov9732", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ov9732 },
    { "ov9750", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ov9750 },
    { "ps5250", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_ps5250 },
    { "ps5260", IMAGE_SENSOR_PART_T21, &image_sensor_raw_ps5260_t21 },
    { "sc1235", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc1235_t21 },
    { "sc1235", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc1235 },
    { "sc1a4t", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc1a4t_t23 },
    { "sc2135", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc2135 },
    { "sc2232", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc2232_t21 },
    { "sc2232", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc2232 },
    { "sc2232h", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc2232h_t21 },
    { "sc2232h", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2232h_t31 },
    { "sc2235", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc2235_t21 },
    { "sc2235", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc2235 },
    { "sc223a", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc223a_t23 },
    { "sc223a", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc223a_t31 },
    { "sc223a", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc223a_t40 },
    { "sc2300", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc2300_t21 },
    { "sc2310", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc2310_t23 },
    { "sc2310", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2310_t31 },
    { "sc2310", IMAGE_SENSOR_PART_T40, &image_sensor_raw_gc2053_t40 },
    { "sc231hai", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc231hai_t32 },
    { "sc2331", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc2331_t23 },
    { "sc2331", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2331_t31 },
    { "sc2332", IMAGE_SENSOR_PART_T21, &image_sensor_raw_sc2332_t21 },
    { "sc2332", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2331_t23 },
    { "sc2336", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc2336_t23 },
    { "sc2336", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2336_t31 },
    { "sc2336", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc2336_t41 },
    { "sc2336p", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc2336p_t23 },
    { "sc2336p", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc2336p_t31 },
    { "sc301iot", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc301iot_t23 },
    { "sc301iot", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc301iot_t31 },
    { "sc301iot", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc301iot_t32 },
    { "sc301iot", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc301iot_t41 },
    { "sc3235", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc3235_t31 },
    { "sc3332", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc3332_t23 },
    { "sc3332", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc3332_t41 },
    { "sc3335", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc3335_t31 },
    { "sc3335", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc3335_t40 },
    { "sc3336", IMAGE_SENSOR_PART_T23, &image_sensor_raw_sc3336_t23 },
    { "sc3336", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc3336_t31 },
    { "sc3336", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc3336_t32 },
    { "sc3336", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc3336_t41 },
    { "sc3338", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc3338_t31 },
    { "sc3338", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc3338_t40 },
    { "sc3338", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc3338_t41 },
    { "sc401ai", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc401ai_t31 },
    { "sc401ai", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc401ai_t32 },
    { "sc401ai", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc401ai_t41 },
    { "sc4236", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc4236_t31 },
    { "sc4236", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc4236 },
    { "sc4336", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc4336_t31 },
    { "sc4336", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc4336_t32 },
    { "sc4336", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc4336_t41 },
    { "sc4336p", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc4336p_t31 },
    { "sc4336p", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc4336p_t32 },
    { "sc4336p", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc4336p_t41 },
    { "sc500ai", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc500ai_t31 },
    { "sc500ai", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc500ai_t40 },
    { "sc500ai", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc500ai_t41 },
    { "sc5235", IMAGE_SENSOR_PART_T31, &image_sensor_raw_sc5235_t31 },
    { "sc5235", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc5235_t40 },
    { "sc5235", IMAGE_SENSOR_PART_APICAL, &image_sensor_raw_ap_sc5235 },
    { "sc5336", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc5336_t41 },
    { "sc8238", IMAGE_SENSOR_PART_T40, &image_sensor_raw_os08a10_t40 },
    { "sc8238", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc8238_t41 },
    { "sc830ai", IMAGE_SENSOR_PART_T40, &image_sensor_raw_sc830ai_t40 },
    { "sc830ai", IMAGE_SENSOR_PART_T41, &image_sensor_raw_sc830ai_t41 },
    { "sc835hai", IMAGE_SENSOR_PART_T32, &image_sensor_raw_sc835hai_t32 },
};
