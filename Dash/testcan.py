import can
import time
import multiprocessing


# "Abstract" class CanNode. All CAN components on the vehicle can be simulated by implementing this class
# FUTURE: may implement virtual CAN communication between virtual CanNodes, but it seems unnecessary at this point,
class CanNode(multiprocessing.Process):
    def __init__(self, node_name, bus, notifer, filters = []):
        # must call this before anything else
        multiprocessing.Process.__init__(self)
        # then any other initialization
        self._name = node_name
        self.__bus = bus
        notifer.add_listener(self._rx_callback)
        self.__filters = filters
        
    def transmit(self, msg):
        try:
            self.__bus.send(msg)
            self.tx_callback(msg)
        except can.CanError:
            print("ERROR: Failed to send. Node: " + self._name)
    
    # real rx_calback, handle the filtering of messages
    def _rx_callback(self, msg):
        # if no filter pass through everything
        if not self.__filters:
            self.rx_callback(msg)
            return
        for (fil, msk) in self.__filters:
            if (msg.id & msk) == fil:
                self.rx_callback(msg)
                return


    # high level rx_callback. Called everytime a CAN msg is received    
    def rx_callback(self, msg):
        pass
        
    # high level tx_callback. Called everytime a CAN msg is sent
    def tx_callback(self, msg):
        pass
    
    def run(self):
        while(True):
            pass
    
    
# example 1
class SimpleSenderNode(CanNode):
    def rx_callback(self, msg):
        print("RECEIVED: " + self._name + ": " + str(msg))
        
    def tx_callback(self, msg):
        print("SENT: " + self._name)

    def run(self):
        for i in range(1000):
            msg = can.Message(arbitration_id=0x123, data=[i//256, i%256], is_extended_id=False) # id length is 11
            self.transmit(msg)
            time.sleep(0.1)

# example 2
class SimpleListenerNode(CanNode):
    def rx_callback(self, msg):
        print("RECEIVED: " + self._name + ": " + str(msg))


# terminate processes after t seconds
def terminate_after(t, procs):
    time.sleep(t)
    for p in procs:
        p.terminate()


# main function
if __name__ == '__main__':
    bus = can.ThreadSafeBus(bustype='socketcan', channel='can0', bitrate=1000000)
    notifer = can.Notifier(bus, [])
    can_nodes = [ SimpleSenderNode("Sender1", bus, notifer), SimpleListenerNode("Listener1", bus, notifer),]
    
    # run all nodes concurrently
    with multiprocessing.Pool() as pool:
        for n in can_nodes:
            pool.apply_async(n.start())
        #pool.apply_async(terminate_after(10, can_nodes)) # terminate after 10s
