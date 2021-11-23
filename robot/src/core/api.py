import threading, time, json, os, sys
import tornado.ioloop, tornado.web
from core.config import Config, Device, Script, Interface


class LogsEndpoint(tornado.web.RequestHandler):
    def get(self,id):
        try:
            file_to_get = "logs/{}.log".format(id)
            with open(file_to_get) as f:
                obj = []
                for line in f.readlines():
                    obj.append(line)

                self.write(json.dumps({'data': obj}))
        except Exception as e:
            self.write(json.dumps({'data':'worker not started yet'}))
    
    

class ConfigEndpoint(tornado.web.RequestHandler):
    
    def initialize(self, factory, config):
        self.factory = factory
        self.config = config
    
    def write_error(self, *args, **kwargs):
        err_cls, err, traceback = kwargs['exc_info']
        print(err_cls, err, traceback)
        
    def get(self):
        self.write(self.config.contents)

    def post(self):
        self.set_header("Content-Type", "text/json")
        data = self.request.body.decode("utf8")
        data = json.loads(self.request.body)

        
class ReloadEndpoint(tornado.web.RequestHandler):
    
    def initialize(self, factory):
        self.factory = factory

    def get(self):
        print("reloading everything")
        self.factory.reload()
        self.write(json.dumps({'info':'reloaded workers'}))

class WorkersEndpoint(tornado.web.RequestHandler):
    
    def initialize(self, factory):
        self.factory = factory


    def get(self):
        obj = []
            
        for thread in self.factory.threads:
            this = self.factory.threads[thread]["info"]
            obj.append(this)


        print(obj)
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
        sys.stdout.register('logs/main.log')
        threading.Thread.__init__(self)


        logsEndpoint = (r"/api/logs/([^/]+)?", LogsEndpoint)
        configEndpoint = (r"/api/config", ConfigEndpoint, {'factory': factory, 'config': config})
        reloadEndpoint = (r"/api/reload", ReloadEndpoint, {'factory': factory})
        workersEndpoint = (r"/api/workers", WorkersEndpoint, {'factory': factory})
        workerEndpoint = (r"/api/worker/([^/]+)?/([^/]+)?", WorkerEndpoint, {'factory': factory})

        path = os.path.dirname(os.path.realpath(__file__)).replace('/core', '/web')
        
        static =  (r"/(.*)", tornado.web.StaticFileHandler, 
                            {'path': path, 
                            'default_filename': 'index.html'})

        self.app = tornado.web.Application([logsEndpoint, configEndpoint, reloadEndpoint, workersEndpoint, workerEndpoint, static])
    
    def start(self):
        self.app.listen(8000)
        tornado.ioloop.IOLoop.current().start()