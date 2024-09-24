# ----------------------------------------------------
# Color Detection Robot interfaced in Arduino
# by Logic Laboratories
# ----------------------------------------------------
# To simply use the program run it in the terminal and 
# use the shortcut "q" to quit.
# ----------------------------------------------------

import cv2
import numpy as np
import serial
import time
import threading
import gc

class CameraDetector:
    def __init__(self, port, baudrate):
        self.serial_connection = serial.Serial(port, baudrate)
        time.sleep(2)

        # HSV Color Calibrated from treshold.py
        self.lower_color = np.array([25, 140, 30])
        self.upper_color = np.array([80, 255, 255])

        # Control variables
        self.frame_skip = 2
        self.command = None 
        self.frame_width = 320 
        self.frame_height = 240

        self.distance_update_interval = 10
        self.last_distance_update_frame = 0

        self.last_command_time = 0
        self.command_interval = 1

        # Define max and min speed limits
        self.MAX_PWM = 255
        self.MIN_PWM = 200

        # Initialize video capture in a separate thread
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.ret = False
        self.frame = None
        self.stopped = False 
        self.lock = threading.Lock()

        # Threading to reduce lag (lmao still lags)
        self._start_video_thread()

    def _start_video_thread(self):
        def update_frame():
            while not self.stopped:
                ret, frame = self.cap.read()
                if ret:
                    with self.lock:
                        self.ret = ret
                        self.frame = frame
                else:
                    self.ret = False
                time.sleep(0.1)

        video_thread = threading.Thread(target=update_frame)
        video_thread.daemon = True
        video_thread.start()

    def calculate_distance(self, radius):
        return 500 / radius if radius != 0 else float('inf')

    def calculate_pwm(self, distance_error):
        gain = 0
        pwm_value = int(self.MAX_PWM - (gain * abs(distance_error)))

        return max(self.MIN_PWM, min(self.MAX_PWM, pwm_value))

    # Masking and Frame Processing
    def process_frame(self, frame):
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Make sures that the stuff being captured are only the things needed by adding a limit
            if area > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                radius = w // 2
                distance = self.calculate_distance(radius)

                ball_center_x = x + w // 2

                distance_error = distance - 10

                frame_center_x = self.frame_width // 2 
                horizontal_error = ball_center_x - frame_center_x

                return distance_error, horizontal_error, distance, frame

        return None, None, None, frame

    def run(self):
        frame_count = 0
        last_displayed_distance = None  

        while not self.stopped:
            with self.lock:
                if self.ret:
                    frame_count += 1

                    if frame_count % self.frame_skip == 0:
                        distance_error, horizontal_error, distance, processed_frame = self.process_frame(self.frame)

                        if distance_error is not None:
                            pwm_value = self.calculate_pwm(distance_error)

                            movement_command = None
                            if distance_error > 1:
                                movement_command = f"F{pwm_value}"
                            elif distance_error < -1:
                                movement_command = f"B{pwm_value}"
                            else:
                                movement_command = "S"  # Stop

                            turning_command = None
                            if horizontal_error > 20:
                                turning_command = f"L{pwm_value}"
                            elif horizontal_error < -20:
                                turning_command = f"R{pwm_value}"
                            else:
                                turning_command = "S"

                            current_time = time.time()

                            if current_time - self.last_command_time >= self.command_interval:
                                self.serial_connection.write(movement_command.encode())
                                self.serial_connection.write(turning_command.encode())
                                self.last_command_time = current_time

                            if frame_count - self.last_distance_update_frame >= self.distance_update_interval:
                                last_displayed_distance = distance
                                self.last_distance_update_frame = frame_count

                        if last_displayed_distance is not None:
                            distance_text = f'Distance: {last_displayed_distance:.2f}m'
                            cv2.putText(processed_frame, distance_text, (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                        cv2.imshow('Tennis Ball Detection', processed_frame)

            # Exit Shortcut
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break

            if frame_count % 50 == 0:
                gc.collect()

    def stop(self):
        self.stopped = True
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = CameraDetector('/dev/ttyACM0', 9600)
    detector.run()
