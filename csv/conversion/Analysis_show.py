# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm

def main():
    #csvの読み取り
    df = pd.read_csv("15.csv")#csvの読み取り
    PATH="data/"
    #df1 = pd.read_csv(PATH+"n1.csv")
    #df2 = pd.read_csv(PATH+"s1.csv")
    #df3 = pd.read_csv(PATH+"i1.csv")
    #df4 = pd.read_csv(PATH+"h1.csv")
    #print(len(df1),len(df2),len(df3),len(df4))
    #合成
    #df = pd.concat([df1,df2,df3,df4])
    print(len(df))
    print(df.head(10))

    df = df[(df["x"]<160) | (df["x"]>184) | (df["y"]<188) | (df["y"]>200)]#手動ノイズ除去 気合

    data=[]
    for index, value in df.value_counts().iteritems():#重複した要素のカウント
        data.append([index[0],index[1],value])

    df = pd.DataFrame(data,columns =["x","y","c"])#要素のカウントを追加したDataFrame作成
    print(df.head(10))
    max=df["c"].max()#要素のカウント最大値
    ratio=5#[%]
    df_=df[df["c"] > max*(ratio/100)]#要素のカウントがratio[%]以上のデータを抜き出し

    #二次元ヒストグラムの表示
    fig=plt.figure(figsize=(10,4))

    fsize=20

    ax = fig.add_subplot(121)
    H = ax.hist2d(df["y"].values.tolist(),df["x"].values.tolist(), bins=40, cmap=cm.jet)
    #ax.set_xlim(140,225)
    #ax.set_ylim(130,225)
    ax.set_title('origin graph')
    ax.set_xlabel('y',fontsize=fsize)
    ax.set_ylabel('x',fontsize=fsize)
    fig.colorbar(H[3],ax=ax)

    ax = fig.add_subplot(122)
    H = ax.hist2d(df_["y"].values.tolist(),df_["x"].values.tolist(), bins=40, cmap=cm.jet)
    #ax.set_xlim(140,225)
    #ax.set_ylim(130,225)
    ax.set_title('More than '+str(ratio)+"%")
    ax.set_xlabel('y',fontsize=fsize)
    ax.set_ylabel('x',fontsize=fsize)
    fig.colorbar(H[3],ax=ax)

    #plt.xticks(rotation=90)
    plt.show()

if __name__ == "__main__":
    main()