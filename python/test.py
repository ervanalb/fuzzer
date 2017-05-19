from fuzzer import driver
import matplotlib.pyplot as plt

f = driver.Fuzzer()
a = f.raw_input(4096)
plt.plot(a)
plt.show()
