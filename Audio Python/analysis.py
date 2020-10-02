import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks

#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("audio06.wav")

#range of data you want to look at in multiples of the sampleRate
#you can set this to whatever you want
rateRange = int(sampleRate / 10)
#the centered time you want to look at
#for t = 1, you go from 1 - rateRange to 1 + rateRange
time = 1
#distance to centered time
dist = sampleRate * time
#convert from audio buffer to cleaner numpy data array
#data = np.frombuffer(audio[dist - rateRange : dist + rateRange], dtype=np.int16)
data = np.frombuffer(audio, dtype=np.int16)[100:-100]
#calculate the peaks along the audio signal
#peaks = arg(data, np.greater)[0]
peaks = find_peaks(data, height=np.mean(data)+8*np.std(data), distance=5000)[0]
print("the first peak is at", peaks[0]/sampleRate, "seconds") # the first peak location
y = [data[i] for i in peaks] #not quite sure what this does

#plots the audio signal
plt.plot(data)
#plots the peaks
plt.plot(peaks, y, 'ro')
plt.ylabel("Amplitude")
plt.xlabel("Samples")
plt.savefig('0.png')
plt.show()