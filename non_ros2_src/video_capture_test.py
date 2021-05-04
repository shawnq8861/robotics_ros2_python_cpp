import cv2
import argparse

def main():
    parser = argparse.ArgumentParser(description='Process an integer arg')
    parser.add_argument('camera_idx', type=int)
    args = parser.parse_args()
    cap = cv2.VideoCapture()
    cap.open(args.camera_idx)
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
        api = cap.get(cv2.CAP_PROP_BACKEND)
        print("backend api = ", api)
        cv2.imwrite("coral_image.png", frame)


if __name__ == "__main__":
    # execute only if run as a script
    main()