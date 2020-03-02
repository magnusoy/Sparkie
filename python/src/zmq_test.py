"""
import zmq
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5556")
req = socket.recv()
socket.send(req)

"""
import zmq
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://10.10.10.111:5556")
socket.send("FOO".encode())
print(socket.recv())
