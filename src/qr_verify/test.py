# File containing unit tests for QR Verify
# @author Josflesan

from qr_verify.qr_verify import QRVerify
import unittest

class TestQrVerify(unittest.TestCase):

    def test_verify(self):
        verifier = QRVerify(hostname="joflesan-ubuntu.local", host_port=5000)
        self.assertTrue(verifier.verify("63f52275b2422530719ec323+averysecurehash"))

    def test_verify_connection_err(self):
        verifier = QRVerify(hostname="fake", host_port=3000)
        self.assertRaises(ConnectionError, verifier.verify, "imadethisup+fakehash")

    def test_verify_delivery_not_found(self):
        verifier = QRVerify(hostname="joflesan-ubuntu.local", host_port=5000)
        self.assertRaises(LookupError, verifier.verify, "fakeithink+verysecureindeed")

    def test_verify_hash_invalid(self):
        verifier = QRVerify(hostname="joflesan-ubuntu.local", host_port=5000)
        self.assertFalse(verifier.verify("63f52275b2422530719ec323+fakehash"))

if __name__ == "__main__":
    unittest.main()
