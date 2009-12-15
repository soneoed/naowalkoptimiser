import socket

class Network:
    
    def __init__(self, ip="flexo.local", port=6767):
            
        self.IP = ip
        self.Port = port
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("10.0.1.248", self.Port))
        self.sock.setblocking(0)
        
    def send(self, vx, vy, v, targetx, targety, targetorientation):
        
        message = "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f" % (vx, vy, v, targetx, targety, targetorientation)
        self.sock.sendto(message, (self.IP, self.Port))
        
    def receive(self):
        
        try:
            packet = self.sock.recv(1024).strip()
            packet = packet.split(',')
            data = list()
            for word in packet:
                data.append(float(word))
            if len(data) == 3:
                self.data = data
            else:
                self.data = [0.0, 0.0, 0.0]
            return self.data
        except socket.error, e:
            print e
            self.data = [0.0, 0.0, 0.0]
        
        return self.data
        
        
if __name__ =='__main__':
    print "Testing Network.py"
    net = Network()
    net.send(2.51, 1.67777, 0.32222, 1, 0.001, 4.5)
