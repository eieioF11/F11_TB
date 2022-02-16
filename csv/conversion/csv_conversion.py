# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

def main():
    fig=plt.figure(figsize=(8,8))
    df = pd.read_csv("3.csv")
    print(df.head(10))
    #fig.add_subplot(121)
    df = df[(df["x"]<160) | (df["x"]>184) | (df["y"]<180) | (df["y"]>200)]
    df_=df[df.duplicated()]#重複したデータの抽出
    #df=df[df.duplicated()]
    #df.hist()
    plt.plot(df["y"],df["x"],"*",c="red")
    plt.plot(df_["y"],df_["x"],"*",c="b")
    plt.show()
    df.to_csv("h1.csv",index=False)

if __name__ == "__main__":
    main()