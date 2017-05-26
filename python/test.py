from fuzzer import driver
#import matplotlib.pyplot as plt
import time

f = driver.Fuzzer()
f.stream_enable(True)
time.sleep(1)
f.stream_enable(False)
