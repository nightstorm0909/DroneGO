from fbchat import Client, log
from fbchat.models import *
import socket
import os

class Bot(Client):
	def onMessage(self, author_id, message_object, thread_id, thread_type, **kwargs):
		# onMessage(mid=None, author_id=None, message=None, message_object=None, thread_id=None, 
		#		    thread_type=ThreadType.USER, ts=None, metadata=None, msg=None)
		self.markAsDelivered(author_id, thread_id)
		self.markAsRead(author_id)
		

		#log.info('{} from {} in {}'.format(message_object, thread_id, thread_type.name))

		if author_id != self.uid:
			#self.send(message_object, thread_id = thread_id, thread_type = thread_type)
			msg = bytes(message_object.text, 'utf-8')
			#self.s.send(msg)
			self.setTypingStatus(TypingStatus.TYPING, thread_id = 100000020271834, thread_type = ThreadType.USER)

			if (message_object.text.lower() == 'bye'):
				sent = self.s.sendto(b'bye', self.server_address)
				#self.s.send(b'bye')
				client.send(Message(text='Shutting down the drone!!!'), thread_id = 100000020271834, thread_type = ThreadType.USER)
				
				client.stopListening()
				self.s.close()
				self.sock.send(b'bye')
				self.sock.close()

			if (message_object.text.lower() == 'start'):
				#self.s.send(b'start')
				sent = self.s.sendto(b'start', self.server_address)

			if (message_object.text.lower() == 'give me your status'):
				#self.s.send(b'start')
				sent = self.s.sendto(b'track', self.server_address)
				data, server = self.s.recvfrom(1024)
				
				print('python: ',data[0])
				msg = 'Tracking the bag!!!\nNow moving'
				statusReply = self.createMessage(data, msg)
				self.sendLocalImage('frame.jpg', message = statusReply, thread_id = 100000020271834, thread_type = ThreadType.USER)
				os.remove('frame.jpg')

				'''if ((data[0] == ord('x')) or (data[0] == ord('b')) or (data[0] == ord('f'))):
					self.sendLocalImage('frame.jpg', message = 'Yes still tracking!!!!', thread_id = 100000020271834, thread_type = ThreadType.USER)
					os.remove('frame.jpg')'''

			# send(message, thread_id=None, thread_type=ThreadType.USER)

	def createMessage(self, data, msg):
		# create a message based on the reply from the drone
		if (data[0] == ord('y')):
			return " Rotating"

		if(data[0] == ord('f')):
			msg = msg + ' forward'
		else:
			msg = msg + ' backward'

		if(data[1] == ord('l')):
			msg = msg + ', left'
		else:
			msg = msg + ', right'

		if(data[2] == ord('u')):
			msg = msg + ' and up'
		else:
			msg = msg + ' and down'

		return msg

	def sockInit(self):
		# initialize the socket for communicating with the c++ server process
		# client.send(Message(text='Shutting down the drone!!!'), thread_id = 100000020271834, thread_type = ThreadType.USER)
		self.sound_address = ('192.168.43.186', 10001)		
		#self.sound_address = ('localhost', 10001)
		self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	# UDP client
		self.server_address = ('localhost', 10000)					# for communicating with C++ UDP server
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.connect(self.sound_address)
		self.sock.send(b'stealing')
		#self.send(Message(text = 'Someone is stealing your bag!!!'), thread_id = 100000020271834, thread_type = ThreadType.USER)
		#self.s.connect(self.server_address)



print('fbChat process id:', os.getpid())
#client = Bot('ludoyiyi@stelliteop.info', 'ardrone2018')
client = Bot('midozi@dumoac.net', 'ardrone2018')
while os.path.isfile('frame.jpg') is not True:
	pass
client.sendLocalImage('frame.jpg', message = 'Someone is stealing your bag', thread_id = 100000020271834, thread_type = ThreadType.USER)
os.remove('frame.jpg')

client.sockInit()
#client.listen()
client.startListening()
client.onListening()
while client.listening and client.doOneListen(markAlive = True):
	pass
client.stopListening()
client.logout()