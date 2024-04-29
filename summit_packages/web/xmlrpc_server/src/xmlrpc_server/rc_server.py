#!/usr/bin/env python
import sys
import threading

if sys.version_info.major < 3:
    from SimpleXMLRPCServer import SimpleXMLRPCServer
    from SocketServer import ThreadingMixIn
else:
    from xmlrpc.server import SimpleXMLRPCServer
    from socketserver import ThreadingMixIn


from rcomponent.rcomponent import *

class MultithreadedSimpleXMLRPCServer(ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True

class XMLRPCServer(MultithreadedSimpleXMLRPCServer, RComponent):
    
    def __init__(self):
        RComponent.__init__(self)

    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self.ip_address="0.0.0.0"
        self.ip_address = rospy.get_param('~ip_address', self.ip_address)

        self.port=50000
        self.port = rospy.get_param('~port', self.port)

        self.xmlrpc_log = True
        self.xmlrpc_log = rospy.get_param('~xmlrpc_log', self.xmlrpc_log)
        
        self.server = MultithreadedSimpleXMLRPCServer((self.ip_address, self.port), logRequests=self.xmlrpc_log, allow_none=True)
        self.server.RequestHandlerClass.protocol_version = "HTTP/1.1"

        return 0

    
    def setup(self):
        self.server.register_function(self.get_state, 'get_state')
        self.server.register_function(self.get_desired_freq, 'get_desired_freq')
        self.server.register_function(self.get_real_freq, 'get_real_freq')
        self.server.register_function(self.get_state_description, 'get_state_description')

        self.server_thread = threading.Thread(target=self.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        RComponent.setup(self)

        return 0
    
    def get_state(self):
        return self._state

    def get_desired_freq(self):
        return self._desired_freq

    def get_real_freq(self):
        return self._real_freq
    
    def get_state_description(self):
        return RComponent.state_to_string(self, self._state)

    def serve_forever(self):
        while True:
            self.server.handle_request()
        
        sys.exit()
    
    def shutdown(self):
        self.server.shutdown()

