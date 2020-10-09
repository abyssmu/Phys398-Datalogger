import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile as wave
from scipy.signal import argrelextrema as arg
from scipy.signal import find_peaks
import p398dlp_read_audio_function as RA

def getMicros(filename):
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

	return time

def calculateTimeAdjust(senderMicros, receiverMicros, senderAjust):
	

	return 

folder = "Sean_Shunyue_Data\\"
receiverName = "receiver"
senderName = "sender"

audioTimeText = "AudioTime.txt"
timeText = "Time.txt"

receiverMicros = getMicros(folder + receiverName + timeText)
senderMicros = getMicros(folder + senderName + timeText)

senderAdjust = adjustMicros(folder + senderName + audioTimeText)

dT = calculateTimeAdjust(senderMicros, receiverMicros, senderAdjust)

# receiverFile = folder + receiverName + ".wav"
# sampleRate, audio = wave.read(receiverFile)
# receiverData = np.frombuffer(audio, dtype = np.int16)

# receiverStart = int(dT * sampleRate)
# receiverData = receiverData[receiverStart :]

# senderFile = folder + senderName + ".wav"
# sampleRate, audio = wave.read(senderFile)
# senderData = np.frombuffer(audio, dtype = np.int16)

# senderStart = int(dT * sampleRate)
# senderData = senderData[senderStart :]

# fig, (a1, a2) = plt.subplots(2, 1)

# a1.plot(receiverData)
# a1.set_title("Receiver")

# a2.plot(senderData)
# a2.set_title("Sender")

# plt.show()