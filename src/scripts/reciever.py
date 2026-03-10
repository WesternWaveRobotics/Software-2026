# Wireless communication reciever test code

import socket

# Testing
hostname = socket.gethostname()
print(socket.gethostbyname(hostname))

ser = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# .bind((host, port))
ser.bind(("0.0.0.0", 5000))
ser.listen(1)

# .accept() -> returns new socket and adress (connection, adress)
conn, adr = ser.accept()
print(adr)

# socket.recv() -> recieve data from socket

conn.close()
