# Python Package that verifies a QR code's authenticity by
# querying a Firebase Realtime Database instance
# @author Josflesan

import requests
import json
from dotenv import dotenv_values

config = dotenv_values("firebase-settings.env")

class QRVerify:
    '''
    A class containing methods used to authenticate a delivery

    Attributes
    ----------
    kwargs['hostname'] : str
        the name of the host where the Realtime Database instance lives.
        This will be 'localhost' or the IP of the machine in the LAN hosting the database
        if it is being locally emulated. If not passed, the environment variable for the 
        production hostname will be used.

    kwargs['host_port'] : int
        the port in which the Realtime Database instance is being served if using
        local emulator.

    Methods
    -------
    verify(qrString):
        connects to the Realtime Database instance and verifies whether QR string is valid

    __updateScanned():
        private method to perform 'scanned' boolean flag change    
    '''

    def __init__(self, **kwargs):
        self._emulated = len(kwargs.keys()) > 0  # Only emulated if port is set
        self._hostname = kwargs.get('hostname', None)
        self._port = kwargs.get('host_port', None)


    def verify(self, qrString: str) -> bool:
        '''
        Method that performs the verification of the QR Code string by 
        querying the Firebase Realtime Database instance.

        @param qrString : the qrCode string resulting from correct scanning in the form 'deliveryId+hash'
        @returns boolean : True if delivery could be authenticated, else False
        '''

        deliveryId, hashCode = qrString.split('+')

        if self._emulated:
            resp = requests.get(f"http://{self._hostname}:{self._port}/deliveries/{deliveryId}.json/?ns=inbox-sdp")
        else:
            resp = requests.get(f"https://inbox-sdp-default-rtdb.europe-west1.firebasedatabase.app/deliveries/{deliveryId}.json")

        jsonObject = resp.json()

        if jsonObject is None:
            raise ConnectionError("Delivery does not exist")
        elif jsonObject['hash'] != hashCode:
            return False  # The hash code is not valid
        else:
            self.__updateScanned(deliveryId)  # Set scanned flag

        return True


    def __updateScanned(self, deliveryId: str):
        '''
        Method that updates the scanned field inside the relevant delivery record.
        This is used by the Online Courier Service to navigate between screens.

        @param deliveryId : the unique id string identifying the delivery
        '''

        payload = {'scanned': True}

        if self._emulated:
            requests.patch(f"http://{self._hostname}:{self._port}/deliveries/{deliveryId}.json/?ns=inbox-sdp", data=json.dumps(payload))
        else:
            requests.patch(f"https://inbox-sdp-default-rtdb.europe-west1.firebasedatabase.app/deliveries/{deliveryId}.json", data=json.dumps(payload))            

def main():
    '''
    Main method added so package is viewable by the colcon package manager
    '''
    verifier = QRVerify()

if __name__ == "__main__":
    main()
