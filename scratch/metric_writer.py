#!/usr/bin/python3

import csv
import os
import sys
import pandas as pd


if __name__== "__main__":
    header = ['PoseX','PoseY','Velocity_X','Velocity_Y','SourceIP', 'DestIP', 'Transmitted', 'Received', 'Throughput','Avg. Delay','Avg. Jitter', 'LostPackets','DistToEnB', 'CellID', 'AttachedToEnB', 'NumOfUesConnected', 'RSRQ', 'RSRP']
 
    all_args = sys.argv

    filename = 'vehicle_metrics.csv'

    coords = all_args[-1].split(":")
    pose_x = coords[0]
    pose_y = coords[1]
    velocity_x = coords[2]
    velocity_y = coords[3]
    source_ip = coords[4]
    dest_ip = coords[5]
    tx = coords[6]
    rx = coords[7]
    thruput = coords[8]
    avg_delay = coords[9]
    avg_jitter = coords[10]
    num_lost_pkt = coords[11]
    dist_to_enb = coords[12]
    cellid_attached = coords[13]
    enb_attached_to = coords[14]
    num_ues_attached = coords[15]
    rsrq = coords[16]
    rsrp = coords[17]
    #rssi = coords[18]

    data = [pose_x, pose_y, velocity_x, velocity_y, source_ip, dest_ip, tx, rx, thruput, avg_delay, avg_jitter, num_lost_pkt, dist_to_enb, cellid_attached, enb_attached_to, num_ues_attached, rsrq, rsrp]

    try:        
        filename = 'vehicle_metrics.csv'      

        file_exists = os.path.isfile(filename)
        
        with open(filename, 'a', encoding='UTF8') as f:
            writer = csv.writer(f, delimiter=',', lineterminator='\n')

            if not file_exists:
                writer.writerow(header)

            writer.writerow(data)        
    
    except csv.Error as e:
        sys.exit('file {}, line {}: {}'.format(filename, writer.line_num, e))

