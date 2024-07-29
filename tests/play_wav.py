from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

script_dir = Path(__file__).parent

# Construct the path to the WAV file
file_path = script_dir / "../build/output.wav"

# Read a WAV file
# file_path = '../build/output.wav'
framerate, data = wavfile.read(file_path)

print(f"Frame Rate: {framerate}, Data Shape: {data.shape}")

plt.figure(figsize=(6, 4))
plt.specgram(data[0:48000])

plt.figure(figsize=(6, 4))
plt.plot(data[3000 : 2 * 4800])

plt.show()
