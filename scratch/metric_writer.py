#!/usr/bin/python3

import csv
import os
import pandas as pd


def main():
    header = ['Source_IP','Destination_IP','Metrics']
    all_args = sys.argv
    coords = all_args[-1].split(":")

    source = coords[0]
    destination = coords[1]
    metrics = coords[2]

    data = [source, destination, metrics]

    with open('vehicle_metrics.csv', 'a', encoding='UTF8') as f:
        writer = csv.writer(f)

        # write the header
        # writer.writerow(header)
  
        # write the data
        writer.writerow(data)

if __name__ == "__main__":
    main()
