import yarp
import time

if __name__ == "__main__":
    yp = yarp.Network()
    yp.init()

    port_in = yarp.BufferedPortBottle()
    port_in.open("/receiver")
    style = yarp.ContactStyle()
    style.persistent = True
    yp.connect("/writer", "/receiver", style)
    n_connects = 0
    port_was_connected = False
    while n_connects < 5:
        while port_in.getInputCount() > 0:
            if not port_was_connected:
                print("Connected")
                port_was_connected = True
            btl = port_in.read(False)
            if btl is not None:
                print(btl.get(0).asInt())

        if port_was_connected:
            print("Disconnected.")
            n_connects += 1
            port_was_connected = False

    yp.disconnect("/writer", "/receiver", style)
    port_in.close()
