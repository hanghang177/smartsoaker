import Leap, sys, thread, time
import serialdata
import cv2
from threading import Thread

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
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

        for gesture in frame.gestures():
            if gesture.type is Leap.Gesture.TYPE_KEY_TAP:
                key_tap = Leap.KeyTapGesture(gesture)
                printedid = []
                for hand in key_tap.hands:
                    handType = "Left hand" if hand.is_left else "Right hand"
                    if(hand.id not in printedid):
                        print "  %s, position: %s" % (
                                 handType, hand.palm_position[0])
                        self.convertedkey = int((hand.palm_position[0]+200)/400*16)
                        if(self.convertedkey>15):
                            self.convertedkey = 15
                        if(self.convertedkey<0):
                            self.convertedkey = 0
                        print self.convertedkey

                    printedid.append(hand.id)

        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
        #       frame.id, frame.timestamp, len(frame.hands), len(frame.fingers))
        #
        # # Get hands
        # for hand in frame.hands:
        #
        #     handType = "Left hand" if hand.is_left else "Right hand"
        #
        #     print "  %s, id %d, position: %s" % (
        #         handType, hand.id, hand.palm_position)
        #
        #     # Get the hand's normal vector and direction
        #     normal = hand.palm_normal
        #     direction = hand.direction
        #
        #     # Calculate the hand's pitch, roll, and yaw angles
        #     print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
        #         direction.pitch * Leap.RAD_TO_DEG,
        #         normal.roll * Leap.RAD_TO_DEG,
        #         direction.yaw * Leap.RAD_TO_DEG)
        #
        #     # Get arm bone
        #     arm = hand.arm
        #     print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
        #         arm.direction,
        #         arm.wrist_position,
        #         arm.elbow_position)
        #
        #     # Get fingers
        #     for finger in hand.fingers:
        #
        #         print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
        #             self.finger_names[finger.type],
        #             finger.id,
        #             finger.length,
        #             finger.width)
        #
        #         # Get bones
        #         for b in range(0, 4):
        #             bone = finger.bone(b)
        #             print "      Bone: %s, start: %s, end: %s, direction: %s" % (
        #                 self.bone_names[bone.type],
        #                 bone.prev_joint,
        #                 bone.next_joint,
        #                 bone.direction)
        #
        # if not frame.hands.is_empty:
        #     print ""

if __name__ == "__main__":
    listener = SampleListener()
    controller = Leap.Controller()

    s = serialdata.SerialData()

    # Have the sample listener receive events from the controller
    controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
    controller.add_listener(listener)

    cap = cv2.VideoCapture(1)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        cv2.flip(gray,1,gray)
        cv2.rectangle(gray, ((listener.convertedkey)*40, 0), ((listener.convertedkey+1)*40, 480), (0, 255, 0), 3)
        s.send(listener.convertedkey)
        # Display the resulting frame
        cv2.imshow('frame', gray)
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