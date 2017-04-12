import time
import yarp

if __name__ == "__main__":
    yp = yarp.Network()
    yp.init()

    port = yarp.Port()
    btl = yarp.Bottle()
    for i in range(5):
        print("Starting up writer port")
        port.open("/writer")

        for j in range(200):
            btl.clear()
            btl.addInt(j)
            port.write(btl)
            time.sleep(0.01)

        print("Closing writer port")
        port.close()
        time.sleep(1)
