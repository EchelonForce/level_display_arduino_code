#!/usr/bin/env python

# Generates the two look up tables needed for fast LED updates.
num_leds = 16

# 8 bit ADC, but the signal is centered and then rectified to half range.
max_sens = 128

thresholds = [5, 6, 7, 9, 11, 13, 17, 21, 26, 33, 41, 51, 65, 81, 102, 115]
#            -30,-28,-26,-24,-22,-20,-18,-16,-14,-12,-10,-8,-6, -4, -2, -1 dB  | 127=0dB
sense_led = [(1 << (i + 1)) - 1 for i in range(num_leds)]
peak_led = [1 << i for i in range(num_leds)]
vals = [i for i in range(max_sens)]

# print(sense_led)
# print(peak_led)
# print(vals)

out_lut_sense = ""
out_lut_peak = ""

prev_out_sense = ""
prev_val = 0
for val in vals:
    out_sense = "0,"
    out_peak = "0,"
    thresh_comment = ""
    for idx, threshold in enumerate(reversed(thresholds)):
        if val >= threshold:
            out_sense = hex(sense_led[num_leds - idx - 1]) + ","
            out_peak = hex(peak_led[num_leds - idx - 1]) + ","
            # print(
            #     val,
            #     threshold,
            #     hex(sense_led[num_leds - idx - 1]),
            #     hex(peak_led[num_leds - idx - 1]),
            # )
            thresh_comment = " // {a}-{b}".format(a=prev_val, b=val - 1)
            break
    # newline/comment handling
    if prev_out_sense != out_sense:
        prev_out_sense = out_sense
        out_lut_sense += thresh_comment + "\n"
        out_lut_peak += thresh_comment + "\n"
        prev_val = val

    out_lut_sense += out_sense
    out_lut_peak += out_peak

print(
    "static const uint16_t lookup_sample[] = {"
    + out_lut_sense[:-1]
    + thresh_comment
    + "\n};"
)
print(
    "static const uint16_t lookup_peak[] = {"
    + out_lut_peak[:-1]
    + thresh_comment
    + "\n};"
)
