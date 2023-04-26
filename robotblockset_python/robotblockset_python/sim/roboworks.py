"""
This module contains object for communication with RoboWorks scene

"""

# MIT License
#
# Copyright (c) 2020-2020 Leon Zlajpah
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# pylint: disable=invalid-name

import socket
import struct
import sys
from collections import Iterable

class RoboWorksScene:
    """RoboWorks scene

    Parameters
    ----------
    sock : socket, optional
        Socket to be used for connection
    """

    connected = False

    def __init__(self, sock=None):
        if sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._is_connected = False
            except socket.error as err:
                print("Socket creation failed with error:\n",err)
        else:
            self.sock = sock
            self._is_connected = True

    def connect(self, scn: str, host='127.0.0.1') -> bool:
        """Connect to RoboWorks scene

        Parameters
        ----------
        scn : str
            RoboWorks model name: `scn`.scn
        host : str, optional
            Host IP, by default '127.0.0.1'

        Returns
        -------
        bool
            Socket connection status
        """
        port=8000
        for c in scn.lower():
            port += ord(c)

        if self._is_connected:
            hp = self.sock.getpeername()
            if hp[0]==host and hp[1]==port:
                return self._is_connected

        try:
            self._host = host
            self._port = port
            self.sock.connect((host, port))
            self._is_connected = True
        except ConnectionRefusedError:
            for c in '.scn':
                port += ord(c)
            try:
                self.sock.connect((host, port))
                self._is_connected = True
                self._host = host
                self._port = port
            except socket.error as err:
                print("Socket connection failed with error:\n",err)
        except socket.error as err:
            print("Socket connection failed with error:\n",err)
        return self._is_connected

    def set_tag_values(self, tagNames, tagValues):
        """
        Set values for RoboWorks scene tags

        Parameters:
        -----------
        tagNames : string
            RoboWorks scene tag names
        tagValues : list
            Tag values

        Returns:
        --------
            bool: command succes
        """
        if not self._is_connected:
            print("Not connected to RoboWorks")
            return None

        header = b's'
        tail = b'e'
        ack = b'\006'
        tagLen = list()
        allTags = ''
        numTags = 0
        for tag in tagNames.split(','):
            numTags += 1
            tagLen.append(len(tag))
            allTags += tag

        if not isinstance(tagValues, Iterable):
            tagValues = [tagValues]
        msg = header+bytes([numTags])+bytes(tagLen)+bytes(allTags,"ascii")+struct.pack('%sf' % numTags,*tagValues)+tail
        try:
            self.sock.sendall(msg)
            data = self.sock.recv(1024)
            return data[0]==ack[0]
        except socket.error as err:
            print("Socket connection failed with error:\n",err)
            return False

    def get_tag_values(self, tagNames):
        """
        Get values for RoboWorks scene tags

        Parameters:
        -----------
        tagNames : string
            RoboWorks scene tag names

        Returns:
        --------
        tagValues : list
            Tag values (None if error)
        """

        if not self._is_connected:
            print('Not connected to RoboWorks')
            return None

        header = b'g'
        tail = b'e'
        ack = b'\006'
        tagLen = list()
        allTags = ''
        numTags = 0
        for tag in tagNames.split(','):
            numTags += 1
            tagLen.append(len(tag))
            allTags += tag

        msg = header+bytes([numTags])+bytes(tagLen)+bytes(allTags,"ascii")+tail
        try:
            self.sock.sendall(msg)
            data = self.sock.recv(1024)
            if not data:
                print('No data')
                return None
            else:
                # print(data)
                if data[0] == ack[0] and data[-1] == tail[0]:
                    # print(struct.unpack_from('%sf' % numTags,data,1))
                    return struct.unpack_from('%sf' % numTags,data,1)
                else:
                    print('Wrong data! Reveived:\n',data)
                    return None

        except socket.error as err:
            print("Socket communication failed with error:\n",err)

    def disconnect(self):
        """
        Diconnect from RoboWorks scene
        """
        try:
            print("Disconnecting from RoboWorks")
            self.sock.close()
        except socket.error as err:
            print("Socket closing failed with error:\n",err)
        finally:
            self._is_connected = False


def isRoboWorks(scn):
    return isinstance(scn, RoboWorksScene)
