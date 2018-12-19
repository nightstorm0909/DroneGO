from fbchat import Client, log
from fbchat.models import *
import os

if __name__ == '__main__':
    print('fbChat process id:', os.getpid())
    client = Client('ludoyiyi@stelliteop.info', 'ardrone2018')
    client.setTypingStatus(TypingStatus.TYPING, thread_id = 100000020271834, thread_type = ThreadType.USER)
    #client.send(Message(text = 'Not tracking!!!!'), thread_id = 100000020271834, thread_type = ThreadType.USER)
    #client.send(Message(sticker=Sticker('767334476626295')), thread_id = 100000020271834, thread_type = ThreadType.USER)
    #client.logout()