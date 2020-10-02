import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg

#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("test.wav")

#range of data you want to look at in multiples of the sampleRate
#you can set this to whatever you want
rateRange = int(sampleRate / 10)

#the centered time you want to look at
#for t = 1, you go from 1 - rateRange to 1 + rateRange
time = 1

#distance to centered time
dist = sampleRate * time

#convert from audio buffer to cleaner numpy data array
data = np.frombuffer(audio[dist - rateRange : dist + rateRange], dtype=np.int16)

#calculate the peaks along the audio signal
peaks = arg(data, np.greater)[0]
y = [data[i] for i in peaks] #not quite sure what this does

#plots the audio signal
plt.plot(data)

#plots the peaks
plt.plot(peaks, y, 'rs')
plt.ylabel("Amplitude")
plt.xlabel("Samples")
plt.savefig('0.png')

plt.show()