
import os
from dotenv import load_dotenv
load_dotenv('/home/ubuntu/ws/src/.env')

import time

def capture_photo(filename):
    os.system(f"fswebcam -r 640x480 -D 0.01 --no-banner { os.environ.get('DATA_PATH') }images/{ filename }.jpg")
    print(f"image saved in images/{ filename }.jpg")

# needs heavy adjustments
def record_video(filename, duration=5, framerate=24):
    frames = duration*framerate
    downtime = 1/framerate
    start = time.time()

    os.system(f"mkdir { os.environ.get('DATA_PATH') }videos/{ filename }")
    for i in range(frames):
        os.system(f"fswebcam -r 640x480 --no-banner { os.environ.get('DATA_PATH') }videos/{ filename }/{ filename }{i}.jpg")
        time.sleep(downtime)
    end = time.time()
    print(f"captured {frames} frames in {end-start} seconds")


def record_audio(filename, duration=5):
    os.system(f"arecord -d {duration} -D hw:webcam,0 -f S16_LE { os.environ.get('DATA_PATH') }audio/{ filename }.wav")
    print(f"audio saved in audio/{ filename }.wav")

if __name__ == '__main__':
    #record_video('example')
    #capture_photo('example')
    #record_audio('example')
    pass