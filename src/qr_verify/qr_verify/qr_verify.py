# Python Package that verifies a QR code's authenticity by
# querying our local REST API service 
# @author Josflesan

import requests
import json

class QRVerify:
    '''
    A class containing methods used to authenticate a delivery

    Attributes
    ----------
    kwargs['hostname'] : str
        the name of the host where the REST API instance lives.
        This will be 'localhost' or the IP of the machine in the LAN hosting the API.

    kwargs['host_port'] : int
        the port in which the REST API instance is being served.

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

        try:
            resp = requests.get(f"http://{self._hostname}:{self._port}/api/v1/deliveries/{deliveryId}/{hashCode}")
            jsonObject = resp.json()
            
            # Toggle the scanned flag
            requests.put(f"http://{self._hostname}:{self._port}/api/v1/deliveries/{deliveryId}")

            return jsonObject['result']
        except Exception:
            return False  # Not catching exceptions to not crash state machine

def main():
    '''
    Main method added so package is viewable by the colcon package manager
    '''
    verifier = QRVerify()

if __name__ == "__main__":
    main()