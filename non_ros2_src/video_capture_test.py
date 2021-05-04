import cv2

def main():
    cap = cv2.VideoCapture()
    cap.open(1)
    if cap.isOpened():
        ret, frame = cap.read()
        h,  w = frame.shape[:2]
        print("cols = ", w)
        print("rows = ", h)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        ret, frame = cap.read()
        h,  w = frame.shape[:2]
        print("cols = ", w)
        print("rows = ", h)
        cv2.imwrite("coral_image.png", frame)


if __name__ == "__main__":
    # execute only if run as a script
    main()