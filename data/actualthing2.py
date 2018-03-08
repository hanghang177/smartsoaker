"""
Listen to serial, return most recent numeric values
Lots of help from here:
http://stackoverflow.com/questions/1093598/pyserial-how-to-read-last-line-sent-from-serial-device
"""
from threading import Thread
import time
import serial
import numpy as np
import cv2
import cv2.cv as cv
from video import create_capture
from common import clock, draw_str


last_received = ''


def receiving(serial_port):
    global last_received
    buffer = ''
    while True:
        buffer += serial_port.read_all()
        if '\n' in buffer:
            lines = buffer.split('\n')  # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            # If the Arduino sends lots of empty lines, you'll lose the last
            # filled line, so you could make the above statement conditional
            # like so: if lines[-2]: last_received = lines[-2]
            buffer = lines[-1]


class SerialData(object):
    def __init__(self):
        try:
            self.serial_port = serial.Serial('com5',115200)
        except serial.serialutil.SerialException:
            # no serial connection
            self.serial_port = None
        else:
            Thread(target=receiving, args=(self.serial_port,)).start()

    def send(self, data):
        self.serial_port.write(data)

    def next(self):
        if self.serial_port is None:
            # return anything so we can test when Arduino isn't connected
            return 100
        # return a float value or try a few times until we get one
        for i in range(40):
            raw_line = last_received
            try:
                return float(raw_line.strip())
            except ValueError:
                print 'bogus data', raw_line
                time.sleep(.005)
        return 0.

    def __del__(self):
        if self.serial_port is not None:
            self.serial_port.close()

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)




if __name__ == '__main__':
    s = SerialData()
    import sys, getopt
    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try:
        video_src = video_src[1]
    # try: video_rec = cv2.VideoCapture(0);
    except:
        video_src = 0
    args = dict(args)
    cascade_fn = args.get('--cascade', "data/haarcascades_GPU/haarcascade_frontalface_alt.xml")
    nested_fn = args.get('--nested-cascade', "data/haarcascades_GPU/haarcascade_eye.xml")

    cascade = cv2.CascadeClassifier(cascade_fn)
    nested = cv2.CascadeClassifier(nested_fn)

    #cam = create_capture(video_src, fallback='synth:bg=../cpp/lena.jpg:noise=0.05')
    cam = cv2.VideoCapture(1)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,1280)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,720)

    while True:
        max = -1
        maxpos = -1
        ret, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        t = clock()
        rects = detect(gray, cascade)
        vis = img.copy()
        draw_rects(vis, rects, (0, 255, 0))
        if(len(rects) > 0):
            max = -1
            maxpos = -1
            for x1,y1,x2,y2 in rects:
                if(x2 - x1 > max):
                    max = x2 - x1
                    maxpos = (x1 + x2)/2
            s.send(bytes(maxpos))
            print(maxpos)
        else:
            s.send("-1")
        for x1, y1, x2, y2 in rects:
            roi = gray[y1:y2, x1:x2]
            vis_roi = vis[y1:y2, x1:x2]
            subrects = detect(roi.copy(), nested)
            draw_rects(vis_roi, subrects, (255, 0, 0))
        dt = clock() - t

        draw_str(vis, (20, 20), 'time: %.1f ms' % (dt * 1000))
        cv2.imshow('facedetect', vis)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
