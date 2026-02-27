#!/usr/bin/env python3

"""
for capturing individual rgb frames from the digit tactile sensors
"""

from digit_interface import Digit
import cv2
import numpy as np
import time
import os

def main():
    # Initialize the DIGIT device
    dl = Digit("D20233")  # Replace with your sensor's serial number or use Digit("")
    dr = Digit("D20237")

    dl.connect()
    dl.set_resolution(Digit.STREAMS["VGA"])
    dl.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])

    dr.connect()
    dr.set_resolution(Digit.STREAMS["VGA"])
    dr.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])

    print("[INFO] Capturing frames from DIGIT. Press Ctrl+C to stop.")

    save_dir = "data/digit_frames"
    os.makedirs(save_dir, exist_ok=True)

    frame_count = 0
    try:
        while True:
            lframe = dl.get_frame()  # This returns a BGR image as a NumPy array
            rframe = dr.get_frame()  # This returns a BGR image as a NumPy array

            # Optional: Display the frame in a window
            cv2.imshow("DIGIT Left Frame", lframe)
            cv2.imshow("DIGIT Right Frame", rframe)

            # Optional: Save frame to disk
            lfilename = os.path.join(save_dir, f"frame_{frame_count:04d}_left.png")
            rfilename = os.path.join(save_dir, f"frame_{frame_count:04d}_right.png")
            # cv2.imwrite(lfilename, lframe)
            # cv2.imwrite(rfilename, rframe)
            frame_count += 1

            # Break if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(.5)  # Sleep to control frame rate

    except KeyboardInterrupt:
        print("\n[INFO] Capture stopped.")

    finally:
        dl.disconnect()
        dr.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
