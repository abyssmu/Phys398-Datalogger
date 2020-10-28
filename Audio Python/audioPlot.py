import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg

#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("audio02.wav")
sampleRate2, audio2 = wave.read("audio03.wav")

#convert from audio buffer to cleaner numpy data array
data = np.frombuffer(audio, dtype=np.int16)
data2 = np.frombuffer(audio2, dtype=np.int16)

#plots the audio signal
plt.plot(data)
plt.plot(data2, alpha = 0.5)

#plots the peaks
#plt.plot(peaks, y, 'rs')
plt.ylabel("Amplitude")
plt.xlabel("Samples")
plt.savefig('0.png')

plt.show()