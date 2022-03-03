# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm

def main():
    #csvの読み取り
    #df = pd.read_csv("n1.csv")#csvの読み取り
    PATH="data/"
    df = pd.read_csv(PATH+"n1.csv")
    #df = pd.read_csv(PATH+"s1.csv")
    #df = pd.read_csv(PATH+"i1.csv")
    #df = pd.read_csv(PATH+"h1.csv")
    print(len(df))
    print(df.head(10))

    #二次元ヒストグラムの表示
    fig=plt.figure(figsize=(10,4))

    ax = fig.add_subplot(111)
    H = ax.hist2d(df["y"].values.tolist(),df["x"].values.tolist(), bins=40, cmap=cm.jet)
    #ax.set_xlim(140,225)
    #ax.set_ylim(130,225)
    #ax.set_title('origin graph')
    ax.set_xlabel('y')
    ax.set_ylabel('x')
    fig.colorbar(H[3],ax=ax)
    plt.show()

if __name__ == "__main__":
    main()