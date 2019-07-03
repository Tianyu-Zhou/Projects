# setup
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time, sys, os

data = np.load(sys.argv[1])

flattened_data = data.reshape(data.shape[0], data.shape[1]*7)
flattened_data = np.delete(flattened_data, np.where(~flattened_data.any(axis=1))[0], axis=0)

fstem = "Results from "+os.path.basename(sys.argv[1])[:-4]
try: os.mkdir(fstem)
except OSError: pass
#os.rename(sys.argv[1], fstem+"/"+os.path.basename(sys.argv[1]))

if not os.path.exists(fstem+"/"+"Data "+fstem+".csv"):
    print "Saving flattened data in", fstem
    np.savetxt(fstem+"/"+"Data "+fstem+".csv", flattened_data, delimiter=",")

joint_labels = ["w0", "w1", "w2", "e0", "e1", "s0", "s1"]
t = flattened_data[:, 0]
abscissa = "t (sec)"



# basic plots
plot_labels = [
                ["Position vs. Time", "q (rad)"],
                ["Velocity vs. Time", "qdot (rad/sec)"],
                ["Admittance Pos. vs. Time", "q_a (rad)"],
                ["Admittance Vel. vs. Time", "qdot_a (rad/sec)"],
                ["Admittance Accel. vs. Time", "qddot_a (rad/sec^2)"],
                ["Desired Pos. vs. Time", "q_d (rad)"],
                ["Desired Vel. vs. Time", "qdot_d (rad/sec)"],
                ["Desired Torque vs. Time", "tau_d (Nm)"],
                ["Torque Applied vs. Time", "tau_app (Nm)"]
]

for i in xrange(1,1+len(plot_labels)):
    if os.path.exists(fstem+"/"+plot_labels[i-1][0]+".pdf"): continue
    print "Generating plot", i
    plt.figure(i)
    ##plt.yscale("log")
    for j in xrange(7*i, 7*(i+1)):
        plt.plot(t, flattened_data[:, j])
    plt.title(plot_labels[i-1][0])
    plt.ylabel(plot_labels[i-1][1])
    plt.xlabel(abscissa)
    plt.legend(joint_labels)
    plt.savefig(fstem+"/"+plot_labels[i-1][0]+".pdf", bbox_inches="tight")


# # detail plots
# d_plot_labels = [
#                 ["Position vs. Time", "q (rad)"],
#                 ["Velocity vs. Time", "qdot (rad/sec)"]
# ]
#
# d_joints = (6)
#
# for k in d_joints:
#     for i in xrange(1,1+len(d_plot_labels)):
#         ##if os.path.exists(fstem+"/"+joint_labels[k]+" "+d_plot_labels[i-1][0]+".pdf"): continue
#         print "Generating detail plot", i
#         plt.figure(len(plot_labels)+i)
#         ##plt.yscale("log")
#         plt.plot(t, flattened_data[:, k])
#         plt.title(plot_labels[i-1][0])
#         plt.ylabel(plot_labels[i-1][1])
#         plt.xlabel(abscissa)
#         plt.legend(joint_labels)
#         plt.savefig(fstem+"/"+plot_labels[i-1][0]+".pdf", bbox_inches="tight")


# multi plots
m_plot_labels = [
                ["Norms of Actual, Admitted and Desired Positions vs. Time", "rad"],
                ["Norms of Actual, Admitted and Desired Velocities vs. Time", "rad/sec"],
                ["Actual, Admitted and Desired Positions vs. Time for s0", "s0 Position (rad)"],
                ["Actual, Admitted and Desired Positions vs. Time for s1", "s1 Position (rad)"]
]
m_plot_legends = [
                ["2-norm of q", "2-norm of q_d + q_a", "2-norm of q_d"],
                ["2-norm of qdot", "2-norm of qdot_d + qdot_a", "2-norm of qdot_d"],
                ["q", "q_d + q_a", "q_d"],
                ["q", "q_d + q_a", "q_d"]

]

joint_offsets = [5,6]

for i in xrange(1,1+len(m_plot_labels)):
    if os.path.exists(fstem+"/"+m_plot_labels[i-1][0]+".pdf"): continue
    print "Generating multi plot", i
    plt.figure(len(plot_labels)+i)
    ##plt.yscale("log")
    if i == 1:
        offsets = (7, 21, 42)
        for j in xrange(3):
            d = np.array([np.linalg.norm(flattened_data[k][offsets[j]:offsets[j]+6]) for k in xrange(flattened_data.shape[0])])
            if j == 1: d += np.array([np.linalg.norm(flattened_data[k][offsets[j+1]:offsets[j+1]+6]) for k in xrange(flattened_data.shape[0])])
            plt.plot(t, d)
    if i == 2:
        offsets = (14, 28, 49)
        for j in xrange(3):
            d = np.array([np.linalg.norm(flattened_data[k][offsets[j]:offsets[j]+6]) for k in xrange(flattened_data.shape[0])])
            if j == 1: d += np.array([np.linalg.norm(flattened_data[k][offsets[j+1]:offsets[j+1]+6]) for k in xrange(flattened_data.shape[0])])
            plt.plot(t, d)
    if i >= 3:
        offsets = (7, 21, 42)
        for j in xrange(3):
            d = flattened_data[:,offsets[j]+joint_offsets[i-3]]
            if j == 1: d += flattened_data[:,offsets[j+1]+joint_offsets[i-3]]
            plt.plot(t, d)

    plt.title(m_plot_labels[i-1][0])
    plt.ylabel(m_plot_labels[i-1][1])
    plt.xlabel(abscissa)
    plt.legend(m_plot_legends[i-1])
    ##plt.legend(joint_labels)
    plt.savefig(fstem+"/"+m_plot_labels[i-1][0]+".pdf", bbox_inches="tight")



plt.show()
