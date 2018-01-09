# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 15:40:57 2015

@author: skylion
"""

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.contacts_pb2
@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect()) 

    def callback(data):
        message = pygazebo.msg.contacts_pb2.Contacts.FromString(data)
        print(message)

    subscriber = manager.subscribe('/gazebo/default/physics/contacts',
                     'gazebo.msgs.Contacts',
                     callback)

    yield From(subscriber.wait_for_connection())
    while(True):
        yield From(trollius.sleep(1.00))
        print('wait...')
    print(dir(manager))

import logging

logging.basicConfig()

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())