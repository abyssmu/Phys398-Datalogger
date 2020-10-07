import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks
#reads in the sample rate and data from the wave file as a buffer array
sampleRate, audio = wave.read("audio09.wav")
print(sampleRate)
#filter first 15 sec
data = np.frombuffer(audio[:sampleRate*15], dtype=np.int16)[100:-100]

#filter the peaks along the audio signal
peaks = find_peaks(data, height=np.mean(data)+6*np.std(data), distance=5000)[0]
print("the first peak is at", peaks[0]/sampleRate, "seconds") # the first peak location
peaks_3 = find_peaks(data, height=np.mean(data)+40*np.std(data), distance=15000)[0]
print("the first peak_3 is at", peaks_3[0]/sampleRate, "seconds")

fig=plt.figure(figsize=(8,14))
ax1=plt.subplot(3,1,1)
ax2=plt.subplot(3,1,2)
ax3=plt.subplot(3,1,3)
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
#plots the peaks of signal_3
y_3 = [data[i] for i in peaks_3] 
plt.plot(peaks_3,y_3,'ro',color='g')

#zoom in starting point
plt.sca(ax2)
plt.ylabel("Amplitude")
plt.xlabel("Samples")
sampleRange=int(sampleRate/100)
sample=data[peaks[0]-sampleRange:peaks[0]+sampleRange*5]
plt.plot(sample)
plt.plot(sampleRange,data[peaks[0]],'ro',label="starting peak at {} sec.".format(peaks[0]/sampleRate))
plt.legend()

#zoom in another peak
plt.sca(ax3)
plt.ylabel("Amplitude")
plt.xlabel("Samples")
sampleRange=int(sampleRate/10)
sample_3=data[peaks_3[0]-sampleRange*3:peaks_3[0]+sampleRange]
plt.plot(sample_3)
plt.plot(sampleRange*3-(peaks_3[0]-peaks[2]),data[peaks[0]],'ro',color='r')
plt.plot(sampleRange*3,data[peaks_3[0]],'ro',label="first peak_3 at {} sec.".format(peaks_3[0]/sampleRate),color='g')

plt.tight_layout()
plt.legend()
plt.savefig('0.png')
plt.show()

