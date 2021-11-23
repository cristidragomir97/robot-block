import threading, sys

class Log(object):
    def __init__(self):
        
        self.terminal = sys.stdout                  # To continue writing to terminal
        self.log={}                                 # A dictionary of file pointers for file logging

    def register(self,filename):                    # To start redirecting to filename
        ident = threading.currentThread().ident     # Get thread ident (thanks @michscoots)
        if ident in self.log:                       # If already in dictionary :
            self.log[ident].close()                 # Closing current file pointer
        self.log[ident] = open(filename, "a")       # Creating a new file pointed associated with thread id

    def write(self, message):
        self.terminal.write(message)                # Write in terminal (comment this line to remove terminal logging)
        ident = threading.currentThread().ident     # Get Thread id
        if ident in self.log:                       # Check if file pointer exists
            self.log[ident].write(message)          # write in file
        else:                                       # if no file pointer 
            for ident in self.log:                  # write in all thread (this can be replaced by a Write in terminal)
                 self.log[ident].write(message)  
    def flush(self):
            #this flush method is needed for python 3 compatibility.
            #this handles the flush command by doing nothing.
            #you might want to specify some extra behavior here.
            pass  