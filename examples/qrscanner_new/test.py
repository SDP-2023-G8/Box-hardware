import Scan
import GenerateQRCode
import cv2
from pyzbar.pyzbar import decode

#GenerateQRCode.get_qrcode("group8-box")

# img = cv2.imread('data/test1.png')
img = cv2.imread('data/tmp.png')
decode_list = decode(img)
print(decode_list[0].data)
Scan.scan()