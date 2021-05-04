import cv2

def main():
    cap = cv2.VideoCapture()
    cap.open(2)
    if cap.isOpened():
        ret, frame = cap.read()
        h,  w = frame.shape[:2]
        print("cols = ", w)
        print("rows = ", h)


if __name__ == "__main__":
    # execute only if run as a script
    main()