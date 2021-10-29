import threading, time, json
import tornado.ioloop, tornado.web

class WorkersEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self):
        self.write(self.factory.threads)


class WorkerEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self):
         
        pass

    def post(self):
        # slider = self.get_body_argument("slider")
        pass



class API(threading.Thread):
    def __init__(self, factory):
        threading.Thread.__init__(self)
        workersEndpoint = (r"/workers", WorkersEndpoint, {'factory': factory,})
        workerEndpoint = (r"/worker", WorkerEndpoint, {'factory': factory})
        # = (r"/thread", StatusHandler, {'io': hardware, 'wristband': wrist})

        app = tornado.web.Application([workersEndpoint, workerEndpoint])
        app.listen(8000)
        tornado.ioloop.IOLoop.current().start()