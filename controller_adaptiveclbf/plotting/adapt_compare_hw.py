#!/usr/bin/env python
import rosbag
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
import numpy as np

if __name__=='__main__':
    # Read bag file
    # bag = rosbag.Bag('/home/davidfan/bag_files/xmaxx_data/xmaxx1_2019-09-14-16-54-00_mars_yard_test7/rosbag/xmaxx1_common_2019-09-14-16-54-36_0.bag.active')
    # no_adapt_t_start = 193
    # no_adapt_t_end = 235.5
    # adapt_t_start = 235.5
    # adapt_t_end = 321
    # ref_t_start = 193
    # ref_t_end = 321

    # bag = rosbag.Bag('/home/davidfan//bag_files/xmaxx_data/xmaxx1_2019-09-14-11-08-00_mars_yard_test3/rosbag/xmaxx1_common_2019-09-14-11-08-57_0.bag.active')
    # no_adapt_t_start = 156
    # no_adapt_t_end = 298.8
    # adapt_t_start = 298.8
    # adapt_t_end = 399
    # ref_t_start = 156
    # ref_t_end = 399

    # debug_topic = 'adaptive_clbf/debug';
    # odom_topic = 'camera/odom/sample';
    # ref_topic = 'reference_vis';

    # # Get data from bag files
    # odom_px = [(t.to_sec(), msg.pose.pose.position.x) for (topic, msg, t) in bag.read_messages(topics=[odom_topic])]
    # t0 = odom_px[0][0]  # TODO: read this directly to speed up
    # odom_px = [(t - t0, x) for (t, x) in odom_px]
    # odom_px_t = [t[0] for t in odom_px]
    # odom_px_val = [val[1] for val in odom_px]

    # odom_py = [(t.to_sec(), msg.pose.pose.position.y) for (topic, msg, t) in bag.read_messages(topics=[odom_topic])]
    # t0 = odom_py[0][0]  # TODO: read this directly to speed up
    # odom_py = [(t - t0, x) for (t, x) in odom_py]
    # odom_py_t = [t[0] for t in odom_py]
    # odom_py_val = [val[1] for val in odom_py]

    # no_adapt_x = [odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > no_adapt_t_start and t < no_adapt_t_end]
    # no_adapt_y = [odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > no_adapt_t_start and t < no_adapt_t_end]


    # adapt_x = [odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > adapt_t_start and t < adapt_t_end]
    # adapt_y = [odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > adapt_t_start and t < adapt_t_end]


    # ref_px = [(t.to_sec(), msg.pose.pose.position.x) for (topic, msg, t) in bag.read_messages(topics=[ref_topic])]
    # t0 = ref_px[0][0]  # TODO: read this directly to speed up
    # ref_px = [(t - t0, x) for (t, x) in ref_px]
    # ref_px_t = [t[0] for t in ref_px]
    # ref_px_val = [val[1] for val in ref_px]

    # ref_py = [(t.to_sec(), msg.pose.pose.position.y) for (topic, msg, t) in bag.read_messages(topics=[ref_topic])]
    # t0 = ref_py[0][0]  # TODO: read this directly to speed up
    # ref_py = [(t - t0, x) for (t, x) in ref_py]
    # ref_py_t = [t[0] for t in ref_py]
    # ref_py_val = [val[1] for val in ref_py]

    # ref_x = [ref_px_val[i] for (i, t) in enumerate(ref_px_t) if t > adapt_t_start and t < adapt_t_end]
    # ref_y = [ref_py_val[i] for (i, t) in enumerate(ref_py_t) if t > adapt_t_start and t < adapt_t_end]

    # f=plt.figure(num=1, figsize=(10,5), dpi=300)
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

    # f.savefig('adapt_compare_hw.pdf', bbox_inches='tight')









    bag = rosbag.Bag('/home/davidfan//bag_files/xmaxx_data/xmaxx1_2019-09-14-11-08-00_mars_yard_test3/rosbag/xmaxx1_common_2019-09-14-11-08-57_0.bag.active')
    no_adapt_t_start = 156
    no_adapt_t_end = 298.8
    adapt_t_start = 298.8
    adapt_t_end = 399
    ref_t_start = 156
    ref_t_end = 399

    debug_topic = 'adaptive_clbf/debug';
    odom_topic = 'camera/odom/sample';
    ref_topic = 'reference_vis';

    # Get data from bag files
    odom_px = [(t.to_sec(), msg.z[0]) for (topic, msg, t) in bag.read_messages(topics=[debug_topic])]
    t0 = odom_px[0][0]  # TODO: read this directly to speed up
    odom_px = [(t - t0, x) for (t, x) in odom_px]
    odom_px_t = [t[0] for t in odom_px]
    odom_px_val = [val[1] for val in odom_px]

    odom_py = [(t.to_sec(), msg.z[1]) for (topic, msg, t) in bag.read_messages(topics=[debug_topic])]
    t0 = odom_py[0][0]  # TODO: read this directly to speed up
    odom_py = [(t - t0, x) for (t, x) in odom_py]
    odom_py_t = [t[0] for t in odom_py]
    odom_py_val = [val[1] for val in odom_py]

    no_adapt_x = np.array([odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > no_adapt_t_start and t < no_adapt_t_end])
    no_adapt_y = np.array([odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > no_adapt_t_start and t < no_adapt_t_end])


    adapt_x = np.array([odom_px_val[i] for (i, t) in enumerate(odom_px_t) if t > adapt_t_start and t < adapt_t_end])
    adapt_y = np.array([odom_py_val[i] for (i, t) in enumerate(odom_py_t) if t > adapt_t_start and t < adapt_t_end])


    ref_px = [(t.to_sec(), msg.z_ref[0]) for (topic, msg, t) in bag.read_messages(topics=[debug_topic])]
    t0 = ref_px[0][0]  # TODO: read this directly to speed up
    ref_px = [(t - t0, x) for (t, x) in ref_px]
    ref_px_t = [t[0] for t in ref_px]
    ref_px_val = [val[1] for val in ref_px]

    ref_py = [(t.to_sec(), msg.z_ref[1]) for (topic, msg, t) in bag.read_messages(topics=[debug_topic])]
    t0 = ref_py[0][0]  # TODO: read this directly to speed up
    ref_py = [(t - t0, x) for (t, x) in ref_py]
    ref_py_t = [t[0] for t in ref_py]
    ref_py_val = [val[1] for val in ref_py]

    no_adapt_ref_x = np.array([ref_px_val[i] for (i, t) in enumerate(ref_px_t) if t > no_adapt_t_start and t < no_adapt_t_end])
    no_adapt_ref_y = np.array([ref_py_val[i] for (i, t) in enumerate(ref_py_t) if t > no_adapt_t_start and t < no_adapt_t_end])


    adapt_ref_x = np.array([ref_px_val[i] for (i, t) in enumerate(ref_px_t) if t > adapt_t_start and t < adapt_t_end])
    adapt_ref_y = np.array([ref_py_val[i] for (i, t) in enumerate(ref_py_t) if t > adapt_t_start and t < adapt_t_end])

    # f=plt.figure(num=1, figsize=(10,5), dpi=300)
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

    no_adapt_err = np.sqrt((no_adapt_x - no_adapt_ref_x)**2 + (no_adapt_y - no_adapt_ref_y)**2)
    adapt_err = np.sqrt((adapt_x - adapt_ref_x)**2 + (adapt_y - adapt_ref_y)**2)
    
    print("mean no adapt err: ", np.mean(no_adapt_err))
    print("stddev no adapt err: ", np.std(no_adapt_err))
    print("max no adapt err: ", np.max(no_adapt_err))

    print("mean adapt err: ", np.mean(adapt_err))
    print("stddev adapt err: ", np.std(adapt_err))
    print("max adapt err: ", np.max(adapt_err))