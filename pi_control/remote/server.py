#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import asyncio
import websockets
import zmq

context = zmq.Context()
# Socket to talk to server
print("Connecting to radio server")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

async def handle_socket(websocket, path):
    while True:
        data = await websocket.recv()
        print("[UI] Recevied: {}".format(data))
        socket.send_string(data)

        #  Get the reply.
        message = socket.recv_string()
        print("[RF] Received: {}".format(message))

start_server = websockets.serve(handle_socket, 'localhost', 8080)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()

""" 
async def recv_data(websocket, path):
    context = zmq.Context()

    # Socket to talk to server
    print("Connecting to radio server")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    while True:
        data = await websocket.recv()
        print("Received: %s" % data)
        socket.send_string(data)

start_server = websockets.serve(recv_data, 'localhost', 8080)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever() """