import cv2

for i in range(5):
    cap = cv2.VideoCapture(i)
    opened = cap.isOpened()
    ret, frame = cap.read()
    print(f"Index {i}: opened={opened}, ret={ret}")
    if ret and frame is not None:
        print("  shape:", frame.shape)
    cap.release()