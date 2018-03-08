from __future__ import division
import Leap, sys, thread, time
import cv2
from threading import Thread

import math

import serial

from pyaudio import PyAudio # sudo apt-get install python{,3}-pyaudio

COMport = 'com14'

class SerialData(object):
    def __init__(self):
        try:
            self.serial_port = serial.Serial(COMport,115200)
        except serial.serialutil.SerialException:
            # no serial connection
            self.serial_port = None
        else:
            pass
    def send(self, data):
        self.serial_port.write(data + ",")

    def __del__(self):
        if self.serial_port is not None:
            self.serial_port.close()



try:
    from itertools import izip
except ImportError: # Python 3
    izip = zip
    xrange = range

convertedkey = -1

def sine_tone(frequency, duration, volume=1, sample_rate=22050):
    n_samples = int(sample_rate * duration)
    restframes = n_samples % sample_rate

    p = PyAudio()
    stream = p.open(format=p.get_format_from_width(1), # 8bit
                    channels=1, # mono
                    rate=sample_rate,
                    output=True)
    s = lambda t: volume * math.sin(2 * math.pi * frequency * t / sample_rate)

    samples = (int(s(t) * 0x7f + 0x80) for t in xrange(n_samples))
    stream.write(bytes(bytearray(samples)))

    # samples = (int(s(t) * 0x7f + 0x80) for t in xrange(n_samples))
    # for buf in izip(*[samples]*sample_rate): # write several samples at a time
    #     stream.write(bytes(bytearray(buf)))

    # fill remainder of frameset with silence
    stream.write(b'\x80' * restframes)

    stream.stop_stream()
    stream.close()
    p.terminate()

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    movekey = -1
    convertedkey = -1

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        for hand in frame.hands:
            self.movekey = int((hand.palm_position[0]+200)/400*14)
            if(self.movekey>13):
                self.movekey = 13
            if(self.movekey<0):
                self.movekey = 0
            break

        for gesture in frame.gestures():
            if gesture.type is Leap.Gesture.TYPE_KEY_TAP:
                key_tap = Leap.KeyTapGesture(gesture)
                printedid = []
                for hand in key_tap.hands:
                    handType = "Left hand" if hand.is_left else "Right hand"
                    if(hand.id not in printedid):
                        print "  %s, position: %s" % (
                                 handType, hand.palm_position[0])
                        self.convertedkey = int((hand.palm_position[0]+200)/400*14)
                        if(self.convertedkey>13):
                            self.convertedkey = 13
                        if(self.convertedkey<0):
                            self.convertedkey = 0
                        print self.convertedkey

                    printedid.append(hand.id)

def main():
    # Create a sample listener and controller
    s = SerialData()
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
    controller.add_listener(listener)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        cv2.flip(gray,1,gray)
        cv2.rectangle(gray, (int((listener.movekey)*640/13), 0), (int((listener.movekey+1)*640/13), 480), (0, 255, 0), 3)
        # Display the resulting frame
        cv2.imshow('frame', gray)

        notes = [261.626, 293.665, 329.628, 349.228, 391.995, 440, 493.883, 523.251, 587.330, 659.255, 698.456, 783.991, 880, 987.767, 1046.50, 1174.66]

        if(listener.convertedkey == -1):
            s.send(str(14))
        if(listener.convertedkey!= -1):
            s.send(str(listener.convertedkey+1))
            # Thread(target=sine_tone, args=(
            #     notes[listener.convertedkey],  # Hz, waves per second A4
            #     0.2,  # seconds to play sound
            #     1,  # 0..1 how loud it is
            #     # see http://en.wikipedia.org/wiki/Bit_rate#Audio
            #     22050  # number of samples per second
            # )).start()
            # sine_tone(
            #     # see http://www.phy.mtu.edu/~suits/notefreqs.html
            #     frequency=261.63+listener.convertedkey*32.03,  # Hz, waves per second A4
            #     duration=1,  # seconds to play sound
            #     volume=1,  # 0..1 how loud it is
            #     # see http://en.wikipedia.org/wiki/Bit_rate#Audio
            #     sample_rate=22050  # number of samples per second
            # )
            time.sleep(0.05)
            listener.convertedkey = -1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
