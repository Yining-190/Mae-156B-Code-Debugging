import cv2
import numpy as np
import math
import serial
import time

# === SERIAL SETUP ===
# Replace this with your actual Mac serial port, such as:
# '/dev/cu.usbmodem14101'
# '/dev/cu.usbserial-1430'
SERIAL_PORT = '/dev/cu.usbmodem141201'   # <-- change this if needed
BAUD_RATE = 9600 # Match this with the Arduino's baud rate

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # wait for Arduino to reset
    print(f"Connected to serial port: {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    ser = None

# === HSV RANGES ===
green_lower = np.array([40, 50, 50])
green_upper = np.array([90, 255, 255])

red_lower1 = np.array([0, 60, 80])
red_upper1 = np.array([10, 255, 170])
red_lower2 = np.array([170, 60, 80])
red_upper2 = np.array([180, 255, 170])

# === DASHED LINE FUNCTION ===
def draw_dashed_line(img, pt1, pt2, color, thickness=1, dash_len=10):
    dist = int(np.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1]))
    if dist == 0:
        return
    for i in range(0, dist, 2 * dash_len):
        start = (
            int(pt1[0] + (pt2[0] - pt1[0]) * i / dist),
            int(pt1[1] + (pt2[1] - pt1[1]) * i / dist),
        )
        end = (
            int(pt1[0] + (pt2[0] - pt1[0]) * min(i + dash_len, dist) / dist),
            int(pt1[1] + (pt2[1] - pt1[1]) * min(i + dash_len, dist) / dist),
        )
        cv2.line(img, start, end, color, thickness)

# === VIDEO SOURCE ===
# Use Camo virtual camera
# On Mac, CAP_AVFOUNDATION is usually more reliable
for index in range(5):  # Test indices 0 to 4
    cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
    if cap.isOpened():
        print(f"Camera initialized successfully with index {index}.")
        break
    else:
        print(f"Failed to initialize camera with index {index}.")
    cap.release()
else:
    print("Error: Could not open video. Ensure Camo is running and connected.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break
time.sleep(2)

# If the Camo virtual camera index is different, try other indices (e.g., 0, 2, etc.)
if not cap.isOpened():
    print("Error: Could not open video. Ensure Camo is running and connected.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    frame = cv2.resize(frame, (360, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Green detection
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    green_cnts, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_center = None
    largest_green = None

    if green_cnts:
        largest_green = max(green_cnts, key=cv2.contourArea)
        M = cv2.moments(largest_green)
        if M["m00"] != 0:
            green_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # Red detection
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_center = None
    largest_red = None

    if red_cnts:
        largest_red = max(red_cnts, key=cv2.contourArea)
        M = cv2.moments(largest_red)
        if M["m00"] != 0:
            red_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # Draw contours
    if green_center is not None and largest_green is not None:
        cv2.drawContours(frame, [largest_green], -1, (0, 255, 0), 2)
        cv2.circle(frame, green_center, 8, (0, 255, 0), -1)

    if red_center is not None and largest_red is not None:
        cv2.drawContours(frame, [largest_red], -1, (0, 0, 255), 2)
        cv2.circle(frame, red_center, 8, (0, 0, 255), -1)

    # Compute angle
    if green_center is not None and red_center is not None:
        cv2.line(frame, green_center, red_center, (0, 0, 255), 2)
        draw_dashed_line(
            frame,
            (green_center[0], 0),
            (green_center[0], frame.shape[0]),
            (255, 255, 255)
        )

        dx = red_center[0] - green_center[0]
        dy = green_center[1] - red_center[1]
        angle = -math.degrees(math.atan2(dx, dy)) + 180

        if angle > 180:
            angle -= 360

        cv2.putText(
            frame,
            f"Angle: {angle:+.1f} deg",
            (50, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 0),
            2
        )

        # Send angle to Arduino
        if ser is not None:
            print("Serial connection established.")
            angle_str = f"{angle:.1f}\n"
            print(f"Sending angle to Arduino: {angle_str}")
            ser.write(angle_str.encode("utf-8"))

            # Read Arduino response
            if ser.in_waiting > 0:
                response = ser.readline().decode("utf-8", errors="ignore").strip()
                print(f"Arduino received: {response}")
            else:
                print("No response from Arduino.")
else:
    print("Serial connection not established.")

    cv2.imshow("Live Red Pendulum Tracker (Camo)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        if ser is not None:
            ser.close()
        exit()

cap.release()
cv2.destroyAllWindows()

if ser is not None:
    ser.close()