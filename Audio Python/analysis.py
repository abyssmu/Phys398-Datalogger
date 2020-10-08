import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks
import p398dlp_read_audio_function as RA

"""
num="20"
textName = "audio"+num
filename = textName + '.bin'
max_buffers = 5000
audio= RA.read_audio(filename, max_buffers)
"""

#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("audio20.wav")
sampleRate=32000
#filter first 15 sec
data = np.frombuffer(audio[:sampleRate*15], dtype=np.int16)[100:-100]
#data = audio[:sampleRate*15]


#filter the peaks along the audio signal
peaks = find_peaks(data, height=np.mean(data)+6*np.std(data), distance=5000)[0]
print("the first peak is at", peaks[0]/sampleRate, "seconds") # the first peak location


fig=plt.figure(figsize=(8,10))
ax1=plt.subplot(2,1,1)
ax2=plt.subplot(2,1,2)
#plots the audio signal
plt.sca(ax1)
plt.plot(data)
plt.ylabel("Amplitude")
ticks = ax1.get_xticks()/sampleRate
ax1.set_xticklabels(ticks)
plt.xlabel("Time [sec]")
#plots the peaks of signal_1
y = [data[i] for i in peaks]
plt.plot(peaks, y, 'ro',color='r')


#zoom in starting point
plt.sca(ax2)
plt.ylabel("Amplitude")
plt.xlabel("Samples")
sampleRange=int(sampleRate/100)
sample=data[peaks[0]-sampleRange:peaks[0]+sampleRange*2]
plt.plot(sample)
plt.plot(sampleRange,data[peaks[0]],'ro',label="starting peak at {} sec.".format(peaks[0]/sampleRate))
plt.legend()

plt.tight_layout()
plt.legend()
plt.savefig('0.png')
plt.show()

