# File containing unit tests for QR Verify
# @author Josflesan

from qr_verify import QRVerify
import unittest

class TestQrVerify(unittest.TestCase):

    def test_verify_emulated(self):
        verifier = QRVerify(hostname="localhost", host_port=9000)
        self.assertTrue(verifier.verify("12345678910+432f45b44c432414d2f97df0e5743818"))

    def test_verify_cloud(self):
        verifier = QRVerify()
        self.assertTrue(verifier.verify("12345678910+432f45b44c432414d2f97df0e5743818"))

    def test_verify_not_found(self):
        verifier = QRVerify()
        self.assertRaises(ConnectionError, verifier.verify, "imadethisup+fakehash")

    def test_verify_hash_invalid(self):
        verifier = QRVerify()
        self.assertFalse(verifier.verify("12345678910+fakehash"))

if __name__ == "__main__":
    unittest.main()
