from widgets.util.subscriber import Subscriber


SUB_IP = '127.0.0.1'
SUB_PORT = '5560'
SUB_TOPIC = 'serial'

sub = Subscriber(ip=SUB_IP, port=SUB_PORT, topic=SUB_TOPIC)
sub.initialize()


if __name__ == "__main__":
    
    while True:
        data = sub.read()
        print(data)
