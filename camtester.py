import cv2
import time

backends = [
    ("DEFAULT", None),
    ("CAP_ANY", cv2.CAP_ANY),
    ("CAP_AVFOUNDATION", cv2.CAP_AVFOUNDATION),
]

for name, backend in backends:
    print(f"\nTesting backend: {name}")
    
    if backend is None:
        cap = cv2.VideoCapture(1)
    else:
        cap = cv2.VideoCapture(1, backend)

    print("isOpened:", cap.isOpened())
    time.sleep(2)

    for i in range(10):
        ret, frame = cap.read()
        print(f"Attempt {i}: ret={ret}, frame is None: {frame is None}")
        if ret and frame is not None:
            print("Frame shape:", frame.shape)
            cv2.imshow(name, frame)
            cv2.waitKey(1000)
            break
        time.sleep(0.2)

    cap.release()
    cv2.destroyAllWindows()