import argparse
import pandas as pd
from matplotlib import pyplot as plt

class Graph():
    
    def graph(self, filename):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        
        df = pd.read_csv(filename)
        # read the csv files using pandas excluding the timedate column
        # df = df.drop(columns=['time'], axis=1)
        
        # Plot the binary detection
        plt.plot(df['time'] / 1e9, df['detection'], label='Binary Detection', color='blue', linestyle='-', marker='o')

        # Plot the smoothed detection
        plt.plot(df['time'] / 1e9, df['smoothed_detection'], label='Smoothed Detection', color='orange', linestyle='-', marker='x')
        
        # Labels and Title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Detection Value')
        plt.title('Binary and Smoothed Detection over Time')
        plt.legend()
        plt.grid(True)
        plt.show()




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="csv file to be graphed")
    filename = parser.parse_args().filename
    # breakpoint()
    graph = Graph()
    graph.graph(filename)