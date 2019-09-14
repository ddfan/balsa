import rosbag
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

BASE_PATH = '/home/nereid/learn2adapt_ws/'

# retrieves data from rosbag & stores them in arrays
def get_data(bag):
	odom = np.array([[0,0]])
	ref = np.array([[0,0]])
	time_elapsed = np.array([0])

	for topic, msg, t in bag.read_messages(topics=['/downward/vio/odometry', '/reference_vis','/time_elapsed']):

		if topic == '/downward/vio/odometry':
			odom = np.append(odom,[[msg.pose.pose.position.x, msg.pose.pose.position.y]],axis=0)

		if topic == '/reference_vis':
			ref = np.append(ref,[[msg.pose.pose.position.x, msg.pose.pose.position.y]],axis=0)

		if topic == '/time_elapsed':
			time_elapsed = np.append(time_elapsed, [msg.data.to_sec()], axis=0)

	return odom, ref, time_elapsed

def error(x1,x2):
	return np.sqrt((x1[:,0] - x2[:,0])**2 + (x1[:,1] - x2[:,1])**2)

with rosbag.Bag(BASE_PATH + 'nolearning3.bag', 'r') as bag:
	nolearn_odom, nolearn_ref, nolearn_time = get_data(bag)
	nolearn_err = error(nolearn_odom, nolearn_ref)
	bag.close()

with rosbag.Bag(BASE_PATH + 'test.bag', 'r') as bag:
	alpaca_odom, alpaca_ref, alpaca_time = get_data(bag)
	alpaca_err = error(alpaca_odom, alpaca_ref)
	bag.close()

with rosbag.Bag(BASE_PATH + 'vanilla3.bag', 'r') as bag:
	vanilla_odom, vanilla_ref, vanilla_time = get_data(bag)
	vanilla_err = error(vanilla_odom, vanilla_ref)
	bag.close()

with rosbag.Bag(BASE_PATH + 'gp3.bag', 'r') as bag:
	gp_odom, gp_ref, gp_time = get_data(bag)
	gp_err = error(gp_odom, gp_ref)
	bag.close()


# plots combined pose errors
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.plot(nolearn_time, nolearn_err)
plt.plot(alpaca_time, alpaca_err)
plt.plot(vanilla_time, vanilla_err)
plt.plot(gp_time, gp_err)
# line for when model was used
x1, y1 = [[30, 30], [0, np.max(alpaca_err)]]
plt.plot(x1,y1)
plt.legend(['nolearn', 'alpaca', 'vanilla', 'gp'])
plt.ylabel("Pose error (m)")
plt.xlabel("Time (s)")

# -- subplots of last 30s of pose errors
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.subplot(411)
plt.plot(nolearn_time[-nolearn_time.shape[0]/4:], nolearn_err[-nolearn_time.shape[0]/4:])
plt.ylabel("Pose error (m)")
plt.legend(["No Learning"])

plt.subplot(412)
plt.plot(alpaca_time[-alpaca_time.shape[0]/4:], alpaca_err[-alpaca_time.shape[0]/4:])
plt.ylabel("Pose error (m)")
plt.legend(["ALPaCA"])

plt.subplot(413)
plt.plot(vanilla_time[-vanilla_time.shape[0]/4:], vanilla_err[-vanilla_time.shape[0]/4:])
plt.ylabel("Pose error (m)")
plt.legend(["VanillaNN"])

plt.subplot(414)
plt.plot(gp_time[-gp_time.shape[0]/4:], gp_err[-gp_time.shape[0]/4:])
plt.ylabel("Pose error (m)")
plt.xlabel("Time (s)")
plt.legend(["GP"])

# -- plots xy position
plt.figure()
plt.subplot(221)
plt.rcParams.update({'font.size': 12})
plt.plot(nolearn_odom[:,0],nolearn_odom[:,1])
plt.plot(nolearn_ref[:,0],nolearn_ref[:,1])
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['odom', 'ref'])
plt.title('No Learning')

plt.subplot(222)
plt.rcParams.update({'font.size': 12})
plt.plot(alpaca_odom[:,0],alpaca_odom[:,1])
plt.plot(alpaca_ref[:,0],alpaca_ref[:,1])
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['odom', 'ref'])
plt.title('ALPaCA')

plt.subplot(223)
plt.rcParams.update({'font.size': 12})
plt.plot(vanilla_odom[:,0],vanilla_odom[:,1])
plt.plot(vanilla_ref[:,0],vanilla_ref[:,1])
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['odom', 'ref'])
plt.title('Vanilla')

plt.subplot(224)
plt.rcParams.update({'font.size': 12})
plt.plot(gp_odom[:,0],gp_odom[:,1])
plt.plot(gp_ref[:,0],gp_ref[:,1])
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['odom', 'ref'])
plt.title('GP')

plt.show()
