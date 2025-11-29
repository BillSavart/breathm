import sys
import time
import csv
from enum import Enum
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from matplotlib.ticker import MultipleLocator


def butter_lowpass(cutoff=2, fs=60, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def lowpass_filter(data, cutoff=2, fs=60, order=4):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

df = pd.read_csv("raw_data.csv")
time_arr = df["time"].to_numpy()
raw = df["pressure"].to_numpy()

filtered = lowpass_filter(raw, cutoff=2, fs=60, order=4)

end_time = time_arr[-1]
mask = (time_arr >= 30) & (time_arr <= end_time - 5)

time_arr = time_arr[mask]-30
raw = raw[mask]
filtered = filtered[mask]

plt.figure(figsize=(10,4))
plt.plot(time_arr, raw, label="Raw", alpha=0.7)
#plt.plot(time_arr, filtered, label="Filtered", linewidth=2)
plt.gca().yaxis.set_major_locator(MultipleLocator(0.5))
plt.ylabel("Pressure")
plt.legend()
plt.tight_layout()
plt.savefig("waveform_csv_raw.png", dpi=150)