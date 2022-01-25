import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

def main():
    fig=plt.figure(figsize=(8,8))
    df = pd.read_csv("15.csv")
    print(df.head(10))
    #fig.add_subplot(121)
    df=df[df.duplicated()]
    df = df[(df["x"]<172) | (df["x"]>184) | (df["y"]<188) | (df["y"]>197)]
    #df=df[df.duplicated()]
    #df.hist()
    plt.plot(df["x"],df["y"],"*")
    plt.show()
    df.to_csv("16.csv",index=False)

if __name__ == "__main__":
    main()