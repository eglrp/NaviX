import os, sys, stat

# Assuming /tmp/foo.txt exists, Set a file execute by the group.
try:
	os.chmod("/dev/ttyUSB0", stat.S_IRWXU|stat.S_IRGRP|stat.S_IROTH)
	os.chmod("/dev/ttyACM0", stat.S_IRWXU|stat.S_IRGRP|stat.S_IROTH)
	os.chmod("/dev/input/js0", stat.S_IRWXU|stat.S_IRGRP|stat.S_IROTH)
except Exception as err:  
    print(err)  
finally:  
    print("Goodbye!")  
