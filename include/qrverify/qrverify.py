# Python Package that verifies a QR code's authenticity by
# querying a Firebase Realtime Database instance
# @author Josflesan

import http.client

class QRVerify:
    '''
    A class containing methods used to authenticate a delivery

    Attributes
    ----------
    hostname : str
        the name of the host where the Realtime Database instance lives.
        This will be 'localhost' or the IP of the machine in the LAN hosting the database
        if it is being locally emulated, otherwise it will be ____

    host_port : int
        the port in which the Realtime Database instance is being served if using
        local emulator.

    Methods
    -------

    verify(qrString):
        connects to the Realtime Database instance and verifies whether QR string is valid

    __updateScanned():
        private method to perform 'scanned' boolean flag change    
    '''

    def __init__(self, host_port : int, hostname="localhost"):
        self._hostname = hostname
        self._port = host_port


    def verify(self, qrString: str) -> bool:
        '''
        Method that performs the verification of the QR Code string by 
        querying the Firebase Realtime Database instance.

        @param qrString : the qrCode string resulting from correct scanning in the form 'deliveryId+hash'
        @returns boolean : True if delivery could be authenticated, else False
        '''
        pass

    def __updateScanned(self):
        '''
        Method that updates the scanned field inside the relevant delivery record.
        This is used by the Online Courier Service to navigate between screens.
        '''
        pass
