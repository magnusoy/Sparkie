# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

from communication.subscriber import Subscriber
from communication.publisher import Publisher

class Worker(Subscriber, Publisher):
    def __init__(self, sub_ip, sub_port, pub_ip, pub_port, topic):
        Subscriber.__init__(self, sub_ip, sub_port, topic)
        Publisher.__init__(self, pub_ip, sub_port, topic)