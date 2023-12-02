import pandas as pd
from matplotlib import pyplot as plt

df = pd.read_csv("adclog.csv")
plt.plot(df.values[:, 0])
plt.show()


