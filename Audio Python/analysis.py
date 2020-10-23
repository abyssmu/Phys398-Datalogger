import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks
import p398dlp_read_audio_function as RA

def getAudioMicros(filename):
	text = open(filename, 'r').read().split("\n")[3].split(',')

	hours = int(text[0])
	minutes = int(text[1])
	seconds = int(text[2])
	micros = int(text[3])

	microsInSec = 1e6
	microsInHour = 3600 * microsInSec
	microsInMin = 60 * microsInSec

	time = microsInHour * hours + microsInMin * minutes + microsInSec * seconds + micros

	return time

def adjustMicros(filename):
	time = open(filename, 'r').read().split("\n")
	time = [x for x in time if x != '']

	baseline = int(time[0])
	for t in range(len(time)):
		time[t] = int(time[t]) - baseline

	return time[1]

folder = "AudioDistanceTest\\"
receiverName = folder + "close_"
senderName = folder + "dist_"

num = "001"
num2 = "01"

audioTimeText = "aud-" + num + ".txt"
timeText = "met-" + num + ".txt"

receiverMicros = getAudioMicros(receiverName + timeText)
senderMicros = getAudioMicros(senderName + timeText)

receiverAdjust = adjustMicros(receiverName + audioTimeText)
senderAdjust = adjustMicros(senderName + audioTimeText)

timeAdjust = int(((receiverMicros - senderMicros) + (receiverAdjust - senderAdjust)) / 1e6 * 32000)

receiverFile = receiverName + "audio" + num2 + ".wav"
sampleRate, audio = wave.read(receiverFile)
receiverData = np.frombuffer(audio, dtype = np.int16)

# receiverStart = int(dT * sampleRate)
# receiverData = receiverData[receiverStart :]

senderFile = senderName + "audio" + num2 + ".wav"
sampleRate, audio = wave.read(senderFile)
senderData = np.frombuffer(audio, dtype = np.int16)

senderData = senderData.tolist()

for i in range(timeAdjust):
	senderData.insert(0, 0)

# senderStart = int(dT * sampleRate)
# senderData = senderData[senderStart :]

# fig, (a1, a2) = plt.subplots(2, 1)

# a1.plot(receiverData)
# a1.set_title("Receiver")

# a2.plot(senderData)
# a2.set_title("Sender")

plt.plot(receiverData[100000:], 'r', label = "close")
plt.plot(senderData[100000:], 'b', label = "dist", alpha = 0.5)

plt.legend(loc = 'upper right')

plt.show()