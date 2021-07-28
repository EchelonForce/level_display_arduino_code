num_leds = 16

# 8 bit ADC, but the signal is centered and then rectified to half range.
max_sens = 128

thresholds = [5, 6, 7, 9, 11, 13, 17, 21, 26, 33, 41, 51, 65, 81, 102, 115]
sense_led = [(1 << (i + 1)) - 1 for i in range(num_leds)]
peak_led = [1 << i for i in range(num_leds)]
vals = [i for i in range(max_sens)]

# print(sense_led)
# print(peak_led)
# print(vals)

out_lut_sense = ""
out_lut_peak = ""
for val in vals:
    out_sense = "0,"
    out_peak = "0,"
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
            break
    out_lut_sense += out_sense
    out_lut_peak += out_peak

print(out_lut_sense)
print(out_lut_peak)
