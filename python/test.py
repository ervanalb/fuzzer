from fuzzer import driver
import matplotlib.pyplot as plt

f = driver.Fuzzer()
f.configure_pin(0, f.CONF_OUTPUT)
f.raw_output([0, 1] * 2048)
a = f.raw_input(4096)
print(a)
plt.plot(a)
plt.show()
