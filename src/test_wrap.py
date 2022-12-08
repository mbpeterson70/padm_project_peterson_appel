import numpy as np

unwrapped_conf = []
ref_conf = []
wrapped_conf = []
should_be = []

ref_conf.append(np.pi)
wrapped_conf.append(np.pi + .1)
should_be.append(np.pi + .1)

ref_conf.append(0)
wrapped_conf.append(2*np.pi + .1)
should_be.append(.1)

ref_conf.append(0)
wrapped_conf.append(2*np.pi - .1)
should_be.append(-.1)

ref_conf.append(np.pi)
wrapped_conf.append(3*np.pi + .1)
should_be.append(np.pi + .1)

ref_conf.append(-np.pi)
wrapped_conf.append(3*np.pi + .1)
should_be.append(-np.pi + .1)

for i in range(len(ref_conf)):
    if np.abs(wrapped_conf[i] - ref_conf[i]) < np.pi:
        unwrapped_conf.append(wrapped_conf[i])
    else:
        diff = (wrapped_conf[i] % (2*np.pi)) - (ref_conf[i] % (2*np.pi))
        if abs(diff) < np.pi:
            unwrapped_conf.append(ref_conf[i] + diff)  
        else:
            unwrapped_conf.append(ref_conf[i] + diff - np.sign(diff)*2*np.pi)

for i in range(len(should_be)):
    print(f'{i}: unwrapped_conf: {unwrapped_conf[i]}, should be: {should_be[i]}')