import threading, time, json
import tornado.ioloop, tornado.web

class ConfigEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self):
        print("something something config")

class ReloadEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self):
        print("reloading everything")

class WorkersEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self):
        obj = []
        for thread in self.factory.threads:
            this = {
                'name': thread, 
                'state':'loaded'
            }
            
            obj.append(this)

        self.write(json.dumps(obj))

class WorkerEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory

    def get(self, _id, action):

        if action=="stop":
            res = self.factory.stop_thread(_id)
        elif action=="start":
            res = self.factory.start_thread(_id)
        elif action=="info":
            res = self.factory.info_thread(_id)
        else:
            self.write({'nothing':''})

        self.write({'nothing':'success'})

class API(threading.Thread):
    def __init__(self, factory):
        threading.Thread.__init__(self)

        configEndpoint = (r"/api/config", ConfigEndpoint, {'factory': factory,})
        reloadEndpoint = (r"/api/reload", ReloadEndpoint, {'factory': factory,})
        workersEndpoint = (r"/api/workers", WorkersEndpoint, {'factory': factory,})
        workerEndpoint = (r"/api/worker/([^/]+)?/([^/]+)?", WorkerEndpoint, {'factory': factory})

        self.app = tornado.web.Application([configEndpoint, reloadEndpoint, workersEndpoint, workerEndpoint])
    
    def start(self):
        self.app.listen(8000)
        tornado.ioloop.IOLoop.current().start()