from opcua import Server
import time

server = Server()
server.set_endpoint("opc.tcp://0.0.0.0:49320")
server.start()
print("OPC UA 模拟服务器已启动，监听 49320 端口")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    server.stop()