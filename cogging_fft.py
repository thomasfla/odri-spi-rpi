#!/usr/bin/python3
#Thomas Flayols - feb 2022
import time
import timeit
from IPython import embed
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import rfft, irfft, fft, ifft
from numpy import pi, exp
PLOT = True
data = np.load("cogging_map.npz")

pos = data['arr_0']#[:1000]
cur = data['arr_1']#[:1000]
A = fft(cur-cur.mean())
n = len(A)
A=A/n #normlize 
freq = 1j*2*pi*np.linspace(0,n-1,n)/n     #Spacial frequencies 
best_coefs_index = abs(A).argsort()[-30:] #The 10 most significant coefs
A_red = A[best_coefs_index]
freq_red = freq[best_coefs_index] 

plt.plot(cur)

def idft(A,freq,m):
	x=0
	n = len(A)
	for k in range(n):
		x  +=  (A[k] * exp(freq[k]*m)).real
	return x

result = timeit.timeit('idft(A,freq,0.1234)', globals=globals(), number=1)
print(f"Calling idft on complete dataset {result*1e6} us")
result = timeit.timeit('idft(A_red,freq_red,0.1234)', globals=globals(), number=1)
print(f"Calling idft on compressed dataset {result*1e6} us")

#sanity check, plot the compress and original fft result
if PLOT:
	c1 = []
	c2 = []
	for p in range(1000):
		c1.append(idft(A,freq,p))
		c2.append(idft(A_red,freq_red,p))
	plt.plot(c1)
	plt.plot(c2)
	plt.show()

print (f"A_red={A_red}")
print (f"freq_red={freq_red}")
print (f"nCogSamp={n}")
embed()
