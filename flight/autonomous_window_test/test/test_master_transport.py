"""Real local XML-RPC traffic tests, without ROS or vehicle IO."""
import unittest,threading,xmlrpc.client,concurrent.futures
from xmlrpc.server import SimpleXMLRPCServer
from test_auto_transport import app

class MasterTransport(unittest.TestCase):
    def test_concurrent_graph_and_identity_calls(self):
        server=SimpleXMLRPCServer(('127.0.0.1',0),logRequests=False,allow_none=True)
        server.register_function(lambda:[[['/pose',['/source']]],[],[]],'getSystemState')
        server.register_function(lambda name:'http://source/'+name,'lookupNode')
        serving=threading.Thread(target=server.serve_forever,daemon=True);serving.start()
        host,port=server.server_address;barrier=threading.Barrier(6)
        master=app.ThreadLocalMaster(lambda:xmlrpc.client.ServerProxy('http://%s:%s'%(host,port),allow_none=True))
        def poll(index):
            barrier.wait(timeout=3)
            for _ in range(15):
                graph=master.getSystemState()
                self.assertEqual(graph[0][0],['/pose',['/source']])
                self.assertEqual(master.lookupNode('/source'),'http://source//source')
            return True
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
                jobs=[pool.submit(poll,i) for i in range(6)]
                self.assertTrue(all(j.result(timeout=15) for j in jobs))
        finally:
            server.shutdown();server.server_close();serving.join(timeout=2)
    def test_lookup_failure_is_not_hidden_as_success(self):
        class Broken:
            def lookupNode(self,name):raise RuntimeError('node disappeared')
        master=app.ThreadLocalMaster(Broken)
        with self.assertRaisesRegex(RuntimeError,'node disappeared'):master.lookupNode('/source')
