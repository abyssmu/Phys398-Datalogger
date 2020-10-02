import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks
#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("audio06.wav")

data = np.frombuffer(audio[:], dtype=np.int16)[100:-100]
#data = np.frombuffer(audio[:sampleRate*10], dtype=np.int16)[100:-100]

#filter the peaks along the audio signal
peaks = find_peaks(data, height=np.mean(data)+6*np.std(data), distance=5000)[0]
print("the first peak is at", peaks[0]/sampleRate, "seconds") # the first peak location
peaks_3 = find_peaks(data, height=np.mean(data)+40*np.std(data), distance=15000)[0]
print("the first peak_3 is at", peaks_3[0]/sampleRate, "seconds")

#plots the audio signal
plt.plot(data)
#plots the peaks of signal_1
y = [data[i] for i in peaks]
plt.plot(peaks, y, 'ro',color='r')

#plots the peaks of signal_3
y_3 = [data[i] for i in peaks_3] 
plt.plot(peaks_3,y_3,'ro',color='g')

plt.ylabel("Amplitude")
plt.xlabel("Samples")
plt.savefig('0.png')
plt.show()

