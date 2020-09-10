# -*- coding: utf-8 -*-
import socket
import struct

HOST = '10.1.1.30'
PORT = 8787

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(10)
while True:
    conn, addr = server.accept()
    clientMessage = conn.recv(1024)
    msg = struct.unpack('iddd', clientMessage)
    print(msg)

    serverMessage = 'I\'m here!'
    conn.sendall(serverMessage.encode())
    conn.close()
