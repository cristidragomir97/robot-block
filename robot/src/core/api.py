import threading, time, json
import tornado.ioloop, tornado.web
from core.config import Config, Device, External, Interface

class ConfigEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory, config):
        self.factory = factory
        self.config = config
    
    def write_error(self, *args, **kwargs):
        err_cls, err, traceback = kwargs['exc_info']
        print(err_cls, err, traceback)
        

    def get(self, id):
        self.write(self.config.contents)

    def post(self):
        self.set_header("Content-Type", "text/json")
        data = json.loads(self.request.body)
        
        try:
            err, _, _, _ = self.config.parse(data[0])
            self.write(json.dumps({"errors":err}))
        except Exception as e:
            self.write(json.dumps({"error":[e]}))
        
class ReloadEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory
        print(self.factory)

    def get(self):
        print("reloading everything")
        self.factory.reload()
        self.write(json.dumps({'info':'reloaded workers'}))

class WorkersEndpoint(tornado.web.RequestHandler):
    def initialize(self, factory):
        self.factory = factory
        print(self.factory)

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
        print(self.factory)

    def get(self, _id, action):
        
        try:
            if action=="stop":
                res = self.factory.stop_thread(_id)
                self.write(res)
            elif action=="start":
                res = self.factory.start_thread(_id)
                self.write(res)
            elif action=="info":
                res = self.factory.info_thread(_id)
                self.write(res)
            else:
                self.write({'nothing':''})
        except Exception as e:
            self.write({'error':e})

class API(threading.Thread):
    def __init__(self, factory, config):
        threading.Thread.__init__(self)

        configEndpoint = (r"/api/config", ConfigEndpoint, {'factory': factory, 'config': config})
        reloadEndpoint = (r"/api/reload", ReloadEndpoint, {'factory': factory})
        workersEndpoint = (r"/api/workers", WorkersEndpoint, {'factory': factory})
        workerEndpoint = (r"/api/worker/([^/]+)?/([^/]+)?", WorkerEndpoint, {'factory': factory})

        

        self.app = tornado.web.Application([configEndpoint, reloadEndpoint, workersEndpoint, workerEndpoint])
    
    def start(self):
        self.app.listen(8000)
        tornado.ioloop.IOLoop.current().start()