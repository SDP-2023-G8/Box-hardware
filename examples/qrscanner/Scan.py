import cv2


def scan():
    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    while True:
        _, img = cap.read()
        data, bbox, _ = detector.detectAndDecode(img)
        if data:
            break
    print(data)
    cap.release()
# https://www.geeksforgeeks.org/webcam-qr-code-scanner-using-opencv/
# https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html#a7290bd6a5d59b14a37979c3a14fbf394
