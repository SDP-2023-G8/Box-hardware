import qrcode

file = "./data/"


def get_qrcode(data):
	img = qrcode.make(data)
	img.save(file + 'tmp.png')
