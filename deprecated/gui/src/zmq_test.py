import zmq
"""
con = zmq.Context()
soc = con.socket(zmq.REQ)
soc.connect("tcp://10.10.10.219:5556")
soc.send("FOO".encode())
print(soc.recv())

"""
con = zmq.Context()
soc = con.socket(zmq.PUB)
soc.bind("tcp://*:5556")
req = soc.recv()
soc.send(req)
