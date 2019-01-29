import numpy as np
import matplotlib.pyplot as plt



#twoList = ["2j_5_0_20.txt", "2j_15_20_50.txt", "2j_20_50_100.txt", "2j_20_0_100.txt"]
#threeList = ["3j_5_0_20.txt", "3j_15_20_50.txt", "3j_20_50_100.txt", "3j_20_0_100.txt"]

file = "20_0.7_0.35_loss"

twoList = ["/home/ricardo/catkin_ws/src/hybrid_control_api/scripts/plots/"+file+".txt"]

fig = plt.figure()

for i in range(0, len(twoList)):
    with open(twoList[i]) as f:
        lines = f.readlines()

        train_loss = []
        test_loss = []

        for j in range(0, len(lines)):
            a = lines[j].split(',')
            #print(a)
            train_loss.append(float(a[1]))
            test_loss.append(float(a[2]))          
        
        x = range(0, len(test_loss))
        y = test_loss
        plt.plot(x, y, label="Test Loss")
        plt.legend()

        x = range(0, len(train_loss))
        y = train_loss
        plt.plot(x, y, label="Train Loss")
        plt.legend()

plt.savefig(file+'.png')

plt.show()