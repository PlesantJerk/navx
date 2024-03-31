# navx - an API for navx2 STMicroelectronics navigation robotics board

This is a pure python library that can read all the sensors on the navx board.  Its conposed of 3 classes.

**NavigationEntry**: This class is a data class that represents a single reading from the board.  It will capture the following attributes:
- yaw
- pitch
- roll
- fused_heading (compass directon 0-360)
- acceleration in x, y, and z direction
- velocity in x, y, and z direction
- displacement in x, y and z direction
- quaternion (w,x,y and z)
- temp (temperature in celcius)

**Navigator**: This class connects to the virtual COM port setup when you plug the navx board into the USB port. This class manages buffering the information from the COM port and handles moving to each record in the feed.

**NavigationMonitor**: This class launches background thread and continues to monitor the COM port using the **Navigator** class.  its key property is current which returns a **NavigationEntry** that represents the current state of the card.


**Example**

```python
import time
import nav as nm

monitor = nm.NavigationMonitor()
# you can move the board around and see how it changes the values
for i in range(0,60):
   q: nm.NavigatorEntry = monitor.current
   print(q)
   time.sleep(0.1)

monitor.stop()
```
