from pygame import *
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 10001)
s.bind(server_address)
s.listen(2)
print('Server Started....')

(clientSocket, address) = s.accept()
print('Connected with ', address[0], ':', address[1])
msg = clientSocket.recv(1024)

if msg == 'bye':
	clientSocket.close()
else:
	mixer.init()
	mixer.music.load('piano.ogg')
	mixer.music.play(-1)

	msg = clientSocket.recv(1024)
	#print('msg: ', msg)
	if msg == b'bye':
		mixer.music.stop()
		print('server closed...')
		clientSocket.close()