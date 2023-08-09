
import os
from dotenv import load_dotenv
load_dotenv('/home/ubuntu/ws/src/.env')

import time
import math

def capture_photo(filename):
    os.system(f"streamer -o { os.environ.get('DATA_PATH') }images/{ filename }.jpg")
    print(f"image saved in images/{ filename }.jpg")

# needs heavy adjustments
def record_video(filename, duration=5, framerate=24):
    frames = duration*framerate
    downtime = 1/framerate
    num_digits = int(math.log10(frames))+1
    start = time.time()

    os.system(f"mkdir { os.environ.get('DATA_PATH') }videos/{ filename }")
    os.system(f"streamer -t { frames } -r { framerate } -o { os.environ.get('DATA_PATH') }videos/{ filename }/{ filename }{ '0'*num_digits }.jpeg")

    end = time.time()
    print(f"captured {frames} frames in {end-start} seconds")


def record_audio(filename, duration=5):
    os.system(f"arecord -d {duration} -D hw:webcam,0 -f S16_LE { os.environ.get('DATA_PATH') }audio/{ filename }.wav")
    print(f"audio saved in audio/{ filename }.wav")

if __name__ == '__main__':
    record_video('example')
    #capture_photo('example')
    #record_audio('example')
    pass
