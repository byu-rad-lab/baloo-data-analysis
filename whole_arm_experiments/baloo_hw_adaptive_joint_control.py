'''
Curtis Johnson June 2024
'''
from mrac.ManipulatorMRACRBF import ManipulatorMRACRBF
import numpy as np
import time
from baloo_hardware.baloo_hw_interface import BalooHWInterface
import rospy
from std_msgs.msg import Float32MultiArray
from mrac.msg import MRACRBFState

np.set_printoptions(precision=3, suppress=True)

rospy.init_node("baloo_adaptive_joint_controller")

ctrl_pub = rospy.Publisher("/baloo/left_arm/joint_controller",
                           MRACRBFState,
                           queue_size=1,
                           tcp_nodelay=True)

controller = ManipulatorMRACRBF(
    num_gen_coords=6,
    numberOfRBFCenters=
    10,  #number doesn't seem to matter too much for performance, but more smooths out the control signal to a point.
    RBFmins=np.array([-1] * 6 * 4) *
    1.0,  #TODO: stopping parameter adjustment somehow? saturate? sat() function doesn't seem to do much.
    RBFmaxes=np.array([1] * 6 * 4) * 1.0,
    zeta=np.array([1] * 6),
    time_constant=np.array(
        [.75] * 6
    ),  #this needs to be tuned aware of underlying dynamics, otherwise smoothing effect is lost. Optional if you have desired trajecotyr already.
    Lambda=[9., 9., 9., 9., 20.,
            20.],  #lambda*KD is weight on position error qdes - q
    Gamma=15.,  #don't love having this so high, its kind of like the itegrator
    KD=[2., 2., 2., 2., 2.,
        2.],  #weight on velocity error, too high causes chattering
    ctrl_dt=.002,
)

#fill one-time only information
msg = MRACRBFState()
msg.header.frame_id = "left_arm"
msg.generalized_coordinates = controller.n
msg.numberRBFs = controller.M
msg.RBFmin = (controller.RBFmins).tolist()
msg.RBFmax = (controller.RBFmaxes).tolist()
msg.zeta = (controller.zeta).tolist()
msg.time_constant = (controller.time_constant).tolist()
msg.Lambda = np.diag(controller.Lambda).tolist()
msg.Gamma = np.diag(controller.Gamma).tolist()
msg.KD = np.diag(controller.Kd).tolist()

#hardware tuning notes:
# KD seems much more touchy from noise. tune best you can.
#increase lambda to eliminate steady state error, but too high causes chattering
#then increase gamma

# lower gains 'behaves nicer', less twitching and better disturance rejections,  bu the actual performance is worse.
# does picking a slower reference system with higher gains help get both? Doesn't seem to help. This seems like a direct tradeoff.

first_time = True


def q_des_function_generator(t):
    if t < 10:
        test = np.asarray([0.7, -0.5, -0.25, 0.4, -0.7, 0.5])
    elif t < 20:
        test = np.asarray([0.2, -0.1, -0.6, 0.4, -0.4, 0.3])
    elif t < 30:
        test = np.asarray([0.5, 0.5, 0.3, 0.3, 0.6, 0.4])
    elif t < 40:
        test = np.asarray([-0.7, 0.5, 0.25, -0.4, 0.7, 0.5])
    elif t < 50:
        test = np.asarray([-0.2, 0.1, 0.6, -0.4, 0.4, -0.3])
    elif t < 60:
        test = np.asarray([0.0, -0.0, .6, 0.6, -0.25,
                           0.8])  #release weight here
    elif t < 70:
        test = np.asarray([-.6, -0.3, -.6, -0.6, -0.5, -0.2])
    elif t < 80:
        test = np.asarray([0.0, -0.0, .0, -0.4, -0.25, 0.])
    elif t < 90:
        test = np.asarray([0.7, -0.4, .6, -0.6, -0.25, 0.8])
    elif t < 100:
        test = np.asarray([-0.4, .5, -.4, 0.2, 0.5, -0.8])
    elif t < 110:
        test = np.asarray([0.6, .3, .0, 0.6, -0.25, 0.6])
    else:
        test = np.zeros(6)

    # test = np.zeros(6)

    return test, None, None


def q_des_sine_wave(t):
    t = t / 5
    #sine wave commands
    qdes = np.asarray([
        0.5 * np.sin(t), 0.5 * np.cos(t), 0.5 * np.sin(t), 0.5 * np.cos(t),
        0.5 * np.sin(t), 0.5 * np.cos(t)
    ])
    qd_des = np.asarray([
        0.5 * np.cos(t), -0.5 * np.sin(t), 0.5 * np.cos(t), -0.5 * np.sin(t),
        0.5 * np.cos(t), -0.5 * np.sin(t)
    ])

    qdd_des = np.asarray([
        -0.5 * np.sin(t), -0.5 * np.cos(t), -0.5 * np.sin(t), -0.5 * np.cos(t),
        -0.5 * np.sin(t), -0.5 * np.cos(t)
    ])

    return qdes, qd_des, qdd_des


def torque2pressures(torque):
    # convert torque to pressures
    p0 = 175 + torque
    p1 = 175 - torque

    return p0, p1


baloo = BalooHWInterface()

input("Press Enter to home elevator...")

homed = baloo.elevator.home_elevator()

if not homed:
    raise Exception("Failed to home elevator")
else:
    print("Elevator homed")

start = time.time()

tau_prev = np.zeros(6)
alpha = 1

rate = rospy.Rate(1 / .002)

while not rospy.is_shutdown():
    rospy.loginfo_once("Starting control loop")
    # if first_time:
    #     input("Press Enter to continue...")
    #     first_time = False
    step_start = time.time()

    q0, q1, q2 = baloo.left_arm.get_joint_angles()
    q0dot, q1dot, q2dot = baloo.left_arm.get_joint_vel()

    q = np.hstack([q0, q1, q2]).squeeze()
    qdot = np.hstack([q0dot, q1dot, q2dot]).squeeze()

    # q_des = np.asarray([0.7, -0.5, -0.25, 0.4, -0.7, 0.5])
    q_des, qd_des, qdd_des = q_des_function_generator(time.time() - start)
    # q_des, qd_des, qdd_des = q_des_sine_wave(time.time() - start)
    msg.q_cmd = q_des.tolist()

    # start = time.time()
    tau, s, theta_hat, tau_ff, tau_pd, q_des, qd_des, qdd_des = controller.solve_for_next_u(
        q, qdot, q_des, qd_des, qdd_des)

    # print(f"Tau_ff: {tau_ff}, Tau_pd: {tau_pd}")
    # print(f"Time to solve: {time.time() - start}")
    tau_applied = alpha * tau + (1 - alpha) * tau_prev
    tau_prev = tau_applied

    tau_spring_ff = 35 * q_des
    tau_applied += tau_spring_ff
    # print(f"tau: {tau}, tau_spring_ff: {tau_spring_ff}")

    #fill ROS message
    msg.header.stamp = rospy.Time.now()
    msg.q_des = q_des.tolist()
    msg.qdot_des = qd_des.tolist()
    msg.qddot_des = qdd_des.tolist()
    msg.q = q.tolist()
    msg.qdot = qdot.tolist()
    msg.s = s.tolist()
    msg.command = tau_applied.tolist()
    msg.theta_hat = list(theta_hat.flatten())
    ctrl_pub.publish(msg)

    p_cmds = []
    for i in range(3):
        p0, p1 = torque2pressures(tau_applied[i * 2])
        p2, p3 = torque2pressures(tau_applied[(2 * i) + 1])
        p_cmds.append(np.array([p0, p1, p2, p3]))

    # print(f"Pressures: {p_cmds}")

    baloo.left_arm.send_pressure_commands(p_cmds)

    if time.time() - start > 120:
        baloo.left_arm.send_pressure_commands([[0, 0, 0, 0]] * 3)
        rospy.signal_shutdown("Time limit reached")

    rate.sleep()
