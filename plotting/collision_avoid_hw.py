#!/usr/bin/env python
import rosbag
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
import numpy as np
import tf.transformations as tr

if __name__=='__main__':
    # Read bag file

    bag = rosbag.Bag('/home/davidfan/bag_files/xmaxx_data/xmaxx1_2019-09-14-16-54-00_mars_yard_test7/rosbag/xmaxx1_common_2019-09-14-16-54-36_0.bag.active')
    no_adapt_t_start = 193
    no_adapt_t_end = 235.5
    adapt_t_start = 235.5
    adapt_t_end = 321
    ref_t_start = 193
    ref_t_end = 321

    min_scan_time = 140
    max_scan_time = 152

    debug_topic = 'adaptive_clbf/debug';
    odom_topic = 'camera/odom/sample';
    ref_topic = 'reference_vis';
    scan_topic = 'scan'

    # Get data from bag files
    odom_px = [(t.to_sec(), msg.pose.pose.position.x) for (topic, msg, t) in bag.read_messages(topics=[odom_topic])]
    t0 = odom_px[0][0]  # TODO: read this directly to speed up
    odom_px = [(t - t0, x) for (t, x) in odom_px]
    odom_px_t = [t[0] for t in odom_px]
    odom_px_val = [val[1] for val in odom_px]

    odom_py = [(t.to_sec(), msg.pose.pose.position.y) for (topic, msg, t) in bag.read_messages(topics=[odom_topic])]
    t0 = odom_py[0][0]  # TODO: read this directly to speed up
    odom_py = [(t - t0, x) for (t, x) in odom_py]
    odom_py_t = [t[0] for t in odom_py]
    odom_py_val = [val[1] for val in odom_py]

    odom_q = [(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w) for (topic, msg, t) in bag.read_messages(topics=[odom_topic])]

    # no_adapt_x = [odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > no_adapt_t_start and t < no_adapt_t_end]
    # no_adapt_y = [odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > no_adapt_t_start and t < no_adapt_t_end]


    # adapt_x = [odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > adapt_t_start and t < adapt_t_end]
    # adapt_y = [odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > adapt_t_start and t < adapt_t_end]


    ref_px = [(t.to_sec(), msg.pose.pose.position.x) for (topic, msg, t) in bag.read_messages(topics=[ref_topic])]
    t0 = ref_px[0][0]  # TODO: read this directly to speed up
    ref_px = [(t - t0, x) for (t, x) in ref_px]
    ref_px_t = [t[0] for t in ref_px]
    ref_px_val = [val[1] for val in ref_px]

    ref_py = [(t.to_sec(), msg.pose.pose.position.y) for (topic, msg, t) in bag.read_messages(topics=[ref_topic])]
    t0 = ref_py[0][0]  # TODO: read this directly to speed up
    ref_py = [(t - t0, x) for (t, x) in ref_py]
    ref_py_t = [t[0] for t in ref_py]
    ref_py_val = [val[1] for val in ref_py]

    # ref_x = [ref_px_val[i] for (i, t) in enumerate(ref_px_t) if t > adapt_t_start and t < adapt_t_end]
    # ref_y = [ref_py_val[i] for (i, t) in enumerate(ref_py_t) if t > adapt_t_start and t < adapt_t_end]


    scan_info = [(t.to_sec(), msg.angle_min, msg.angle_max, msg.angle_increment) for (topic, msg, t) in bag.read_messages(topics=[scan_topic])]
    angle_min = [msg[1] for msg in scan_info]
    angle_max = [msg[2] for msg in scan_info]
    angle_increment = [msg[3] for msg in scan_info]
    t0 = scan_info[0][0]
    scan_t = [msg[0]-t0 for msg in scan_info]

    scan_ranges = [msg.ranges for (topic, msg, t) in bag.read_messages(topics=[scan_topic])]
    scan_px = []
    scan_py = []
    odom_idx = 0
    for i,ranges in enumerate(scan_ranges):
        if scan_t[i] > min_scan_time and scan_t[i] < max_scan_time:
            while True:
                if (odom_px_t[odom_idx] >= scan_t[i]):

                    q = odom_q[odom_idx]
                    euler = tr.euler_from_quaternion(q)
                    current_heading = euler[2]
                    angles = np.arange(angle_min[i],angle_max[i]-angle_increment[i],angle_increment[i]) + current_heading
                    ranges = np.array(ranges)
                    angles = angles[np.isfinite(ranges) & (ranges < 3.0)]
                    ranges = ranges[np.isfinite(ranges) & (ranges < 3.0)]

                    downsample_idx = np.random.choice(range(ranges.shape[0]),size=int(ranges.shape[0]*0.7))
                    angles = np.array([angles[i] for i in downsample_idx])
                    ranges = np.array([ranges[i] for i in downsample_idx])

                    new_px = ranges * np.cos(angles) + odom_px_val[odom_idx]
                    new_py = ranges * np.sin(angles) + odom_py_val[odom_idx]
                    new_scan_t = ranges * 0 + odom_px_t[odom_idx]
                    
                    scan_px.append(new_px)
                    scan_py.append(new_py)
                    break

                odom_idx = odom_idx + 1

    scan_odom_x = [odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > min_scan_time and t < max_scan_time]
    scan_odom_y = [odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > min_scan_time and t < max_scan_time]

    from itertools import islice
    def downsample_to_proportion(rows, proportion=1):
        return list(islice(rows, 0, len(rows), int(1/proportion)))
    scan_odom_x = downsample_to_proportion(scan_odom_x,0.025)
    scan_odom_y = downsample_to_proportion(scan_odom_y,0.025)

    ref_x = [ref_px_val[i] for (i, t) in enumerate(ref_px_t) if t > min_scan_time and t < max_scan_time]
    ref_y = [ref_py_val[i] for (i, t) in enumerate(ref_py_t) if t > min_scan_time and t < max_scan_time]
    ref_x = downsample_to_proportion(ref_x,0.25)
    ref_y = downsample_to_proportion(ref_y,0.25)


    f=plt.figure(num=1, figsize=(5,5), dpi=300)
    ax = plt.gca()


    # n = ref_x.__len__()
    # colors = plt.cm.jet(np.linspace(0,1,n))
    # for i in range(n):
        # plt.scatter(ref_x[i], ref_y[i], s=10, marker='^',linestyle='None',color=colors[i])
    plt.plot(ref_x, ref_y, 'b', zorder=1)

    # scan_px = downsample_to_proportion(scan_px, 0.7)
    # scan_py = downsample_to_proportion(scan_py, 0.7)

    n = scan_px.__len__()
    colors = plt.cm.jet(np.linspace(0,1,n))
    print(n)
    for i in range(n):
        if i > 150:
            plt.scatter(scan_px[i],scan_py[i],s=0.2,marker='o',linestyle='None',color=colors[i], zorder=2)

    n = scan_odom_x.__len__()
    colors = plt.cm.jet(np.linspace(0,1,n))
    for i in range(n):
        plt.scatter(scan_odom_x[i], scan_odom_y[i], s=20,marker='+',linestyle='None',color=colors[i], zorder=3)

    circle2 = plt.Circle((1.6, 0.1), 0.2, color='g', fill=False)
    ax.add_artist(circle2)

    plt.xlim((-5,10))
    plt.show()
    f.savefig('collision_avoid_hw.pdf', bbox_inches='tight')

    # plt.figure(num=1, figsize=(10,5), dpi=300)
    # # plt.subplot(3,1,1)
    # plt.plot(no_adapt_x, no_adapt_y, 'r',  markersize='0.5', alpha=0.8, label = 'No adaptation')
    # plt.plot(adapt_x, adapt_y, 'g',  markersize='0.5', alpha=0.8, label = 'Adaptation')
    # plt.plot(ref_x, ref_y, 'b',  markersize='0.5', alpha=0.8, label = 'Reference')

    # # plt.fill_between(ciots, ciovx_std_dn, ciovx_std_up, 
    #                     # color = 'r', alpha = 0.2)#, 


    # # plt.ylabel('Velocity - X [m/s]', fontsize=14,
    #           # horizontalalignment='center')   
    # ax = plt.gca()
    # ax.legend(loc='upper left')
    # # plt.grid(True)
    # # plt.axis([0, None, None, None])
    # # plt.xlim((t_start,t_end))
    # # plt.ylim((-3.,1.5))

    # # ax.axes.get_xaxis().set_ticklabels([])
    # # ax.xaxis.set_minor_locator(AutoMinorLocator(8))
    # # ax.yaxis.set_minor_locator(AutoMinorLocator(4))
    # # ax.grid(which='minor', color='#CCCCCC', linestyle=':')

    # plt.show()

    # plt.savefig('collision_avoid_hw.pdf', bbox_inches='tight')