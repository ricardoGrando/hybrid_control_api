
import time

class runOnThormang(object):
    def __init__(self):
        pass

    def runOnThormang(self, angles, mutex, linkThreads, grip, ticks):
        for tick in range(0, ticks):     
            mutex.acquire()

            publishersFlag = True
            for i in range(0, len(linkThreads)):
                if linkThreads[i].flag == True:
                    publishersFlag = False
                    break    
            
            if publishersFlag == True:
                
                linkThreads[0].setValue(angles[0])
                linkThreads[1].setValue(angles[1])
                linkThreads[2].setValue(angles[2])
                linkThreads[3].setValue(angles[3])
                linkThreads[4].setValue(angles[4])
                linkThreads[5].setValue(angles[5])
                linkThreads[6].setValue(angles[6])

                linkThreads[7].setValue(grip)
                linkThreads[8].setValue(grip)

                for i in range(0, len(linkThreads)):                                        
                    linkThreads[i].setFlag()

                mutex.notify_all()
            else:
                mutex.wait() 
            
            mutex.release() 

        # for i in range(0, len(pubList)):
        #     self.linkThreads[i].join()
