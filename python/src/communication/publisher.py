import zmq
from multiprocessing import Process

"""
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

while True:
    zipcode = 10001
    temperature = randrange(-80, 135)
    relhumidity = randrange(10, 60)

    socket.send_string("%i %i %i" % (zipcode, temperature, relhumidity))
"""
class Publisher(Process):
    """docstring"""

    def __init__(self, ip, port, _filter):
        Process.__init__(self)
        self.ip = ip
        self.port = port
        self.filter = _filter
        self.running = True
        self.msg = ''

    def initialize(self):
        """docstring"""
        
        self.context = zmq.Context()
        self.address = 'tcp://%s:%s' % (self.ip, self.port)
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.address)

    def run(self):
        """docstring"""
        
        self.initialize()

        while self.running:
            self.send(self.msg)

    def send(self, msg):
        """docstring"""
        
        self.socket.send_string('%i %s' % (self.filter, msg))

    def stop(self):
        """docstring"""
        
        self.running = False


class Worker(Publisher):
    """docstring"""

    def __init__(self, ip, port, _filter):
        Publisher.__init__(self, ip, port, _filter)
    
    def run(self):
        """docstring"""

        Publisher.initialize(self)
        
        while self.running:
            Publisher.send(self)


# Example of usage
if __name__ == "__main__":
    pub = Publisher('*', 5556, 0)
    pub.start()