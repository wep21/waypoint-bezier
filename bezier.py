#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import argparse
import os

def bezier(start, end,
           start_control_point, end_control_point, resampling_num):
        calc_x = lambda t: (1-t)**3*start['x'] \
                            + 3*(1-t)**2*t*start_control_point['x'] \
                            + 3*(1-t)*t**2*end_control_point['x'] \
                            + t**3*end['x']
        calc_y = lambda t: (1-t)**3*start['y'] \
                            + 3*(1-t)**2*t*start_control_point['y'] \
                            + 3*(1-t)*t**2*end_control_point['y'] \
                            + t**3*end['y']
        calc_z = lambda t: (1-t)**3*start['z'] \
                            + 3*(1-t)**2*t*start_control_point['z'] \
                            + 3*(1-t)*t**2*end_control_point['z'] \
                            + t**3*end['z']
        calc_yaw = lambda t: np.arctan2((-3*(1-t)**2*start['y']+3*(1-t)*(1-3*t)
                                         * start_control_point['y']
                                         + 3*t*(2-3*t)*end_control_point['y']
                                         + 3*t**2*end['y']),
                                        (-3*(1-t)**2*start['x']
                                         + 3*(1-t)*(1-3*t)
                                         * start_control_point['x']
                                         + 3*t*(2-3*t)*end_control_point['x'] 
                                         + 3*t**2*end['x']))
        list_t = np.linspace(0, 1, resampling_num)
        x = list(map(calc_x, list_t))
        y = list(map(calc_y, list_t))
        z = list(map(calc_z, list_t))
        yaw = list(map(calc_yaw, list_t))
        return x, y, z, yaw


def control_points(x, y, z):
    if (len(x) != len(y) or len(y) != len(z) or len(z) != len(x)):
        raise Exception("Size does not match.")
    l = len(x)
    A = np.zeros((2*l-2, 2*l-2))
    b_x = np.zeros(2*l-2)
    b_y = np.zeros(2*l-2)
    b_z = np.zeros(2*l-2)
    A[0, 0] = -2
    A[0, 1] = 1
    A[2*l-3, 2*l-4] = 1
    A[2*l-3, 2*l-3] = -2
    b_x[0] = -x[0]
    b_y[0] = -y[0]
    b_z[0] = -z[0]
    b_x[2*l-3] = -x[l-1]
    b_y[2*l-3] = -y[l-1]
    b_z[2*l-3] = -z[l-1]
    for i in range(1, l-1):
        A[i*2-1, i*2-1] = 1
        A[i*2-1, i*2] = 1
        A[i*2, i*2-2] = 1
        A[i*2, i*2-1] = -2
        A[i*2, i*2] = 2
        A[i*2, i*2+1] = -1
        b_x[i*2-1] = 2*x[i]
        b_x[i*2] = 0
        b_y[i*2-1] = 2*y[i]
        b_y[i*2] = 0
        b_z[i*2-1] = 2*z[i]
        b_z[i*2] = 0
    inv_A = np.linalg.inv(A)
    control_x = np.dot(inv_A, b_x)
    control_y = np.dot(inv_A, b_y)
    control_z = np.dot(inv_A, b_z)
    return control_x, control_y, control_z


if __name__ == '__main__':
    # arg setting
    paser = argparse.ArgumentParser(description='waypoint bezier replanner')
    paser.add_argument('--filename', '-f', type=str,
                       help='waypoint csv filename',
                       required=True)
    paser.add_argument('--step', '-s', type=int,
                       help='step for bezier fitting',
                       required=True)                   
    paser.add_argument('--resampling_num', '-r', type=int,
                       help='resampling number',
                       required=True)
    args = paser.parse_args()
    x = []
    y = []
    z = []
    yaw = []
    velocity = []
    change_flag = [] 
    # import csv
    df_path = pd.read_csv(args.filename)
    # calcurate step points
    wp_size = len(df_path)
    step = args.step
    df_path_step = df_path[::step]
    last_step = (wp_size - 1) % step
    if(last_step != 0):
        df_path_step = df_path_step.append(df_path.iloc[-1])
    # calcurate control points
    x_step = np.array(df_path_step['x'])
    y_step = np.array(df_path_step['y'])
    z_step = np.array(df_path_step['z'])
    cx, cy, cz = control_points(x_step, y_step, z_step)
    # calcurate bezier fitting
    for i in range(len(x_step)-1):
        scp = {}
        ecp = {}
        scp['x'] = cx[i*2]
        scp['y'] = cy[i*2]
        scp['z'] = cz[i*2]
        ecp['x'] = cx[i*2+1]
        ecp['y'] = cy[i*2+1]
        ecp['z'] = cz[i*2+1]
        start = df_path_step.iloc[i]
        end = df_path_step.iloc[i+1]
        resampling_num = args.resampling_num
        if(i == len(x_step)-2 and last_step > 0):
            resampling_num = int(resampling_num / step * last_step + 1)
        bx, by, bz, byaw = bezier(start=start, end=end, 
                                  start_control_point=scp,
                                  end_control_point=ecp,
                                  resampling_num=resampling_num)
        bvelocity = np.linspace(start['velocity'], end['velocity'],
                                resampling_num).tolist()
        bchange_flag = [0] * resampling_num

        if (i != len(x_step)-2):
            bx.pop(-1)
            by.pop(-1)
            bz.pop(-1)
            byaw.pop(-1)
            bvelocity.pop(-1)
            bchange_flag.pop(-1)
        x.extend(bx)
        y.extend(by)
        z.extend(bz)
        yaw.extend(byaw)
        velocity.extend(bvelocity)
        change_flag.extend(bchange_flag)
        bx.clear()
        by.clear()
        bz.clear()
        byaw.clear()
        bvelocity.clear()
        bchange_flag.clear()
    # plot result
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111)
    ax.scatter(df_path_step['x'], df_path_step['y'], s=100)
    ax.scatter(df_path['x'], df_path['y'])
    ax.scatter(x, y)
    plt.show()
    # save waypoints to csv
    df_path2 = pd.DataFrame(columns=['x', 'y', 'z', 'yaw', 
                            'velocity', 'change_flag'])
    df_path2['x'] = x
    df_path2['y'] = y
    df_path2['z'] = z
    df_path2['yaw'] = yaw
    df_path2['velocity'] = velocity
    df_path2['change_flag'] = change_flag
    dir = os.path.dirname(os.path.abspath(__file__))
    basename = os.path.basename(args.filename)
    df_path2.to_csv(dir + "/" + basename[:-4] + "-" + str(args.step) 
                    + "-" + str(args.resampling_num) + ".csv",
                    header=True, index=False)

